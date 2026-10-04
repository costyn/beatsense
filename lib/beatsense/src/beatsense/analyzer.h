// SPDX-License-Identifier: EUPL-1.2
//
// beatsense analysis core: raw I2S samples in, tempo / beat phase / band levels out.
// Portable C++ (no Arduino or ESP-IDF includes), no heap allocation, so it runs unchanged in native tests and on the ESP32-S3.
// Pipeline and tuning constants: docs/design.md.
#ifndef BEATSENSE_ANALYZER_H
#define BEATSENSE_ANALYZER_H

#include <stddef.h>
#include <stdint.h>
#include "fft.h"
#include "beatsense_protocol.h"

// Select the FFT backend (see fft.h for the seam). Must provide a default constructor and forward(in, re, im) for 512 points.
#ifndef BEATSENSE_FFT_CLASS
#define BEATSENSE_FFT_CLASS ::beatsense::RealFft<512>
#endif

namespace beatsense {

// Everything the host can see; mirrors proto::Frame (see toFrame()).
struct Features {
  float bpm = 0;            // 0 until a tempo has been found
  uint16_t beatPhase = 0;   // 0-65535 within the current beat, at the time the last sample was captured (latency compensated)
  uint8_t beatCount = 0;    // wraps; increments each beat
  uint8_t beatInBar = 0;    // 0-3, best guess
  uint8_t confidence = 0;   // 0-255
  uint8_t levels[3] = {0, 0, 0}; // low / mid / high, AGC normalised
  uint8_t onsets[3] = {0, 0, 0}; // peak-hold since the last takeFeatures() (cleared on read), or this hop's value from features()
  uint8_t energy = 128;     // slow energy relative to the long-term average, 128 = average
  uint8_t status = 0;       // proto::kStatus* bits
};

struct Config {
  // Compile-time sizes (the FFT is a template parameter): 512-point FFT, 50% overlap
  static constexpr size_t kFftSize = 512;
  static constexpr size_t kHop = 256;
  static constexpr size_t kBands = 8;
  static constexpr size_t kGroups = 3; // low (kick) / mid (snare, body) / high (hats)

  float sampleRate = 22050.0f;

  // --- Input ---
  float dcCutoffHz = 20.0f;      // DC-blocking high-pass (the INMP441 has a DC offset)
  float clipLevel = 0.985f;      // fraction of full scale that counts as clipping
  float clipHoldS = 0.5f;

  // --- Bands: 8 log-spaced bands from these edges (Hz); the last edge 0 = Nyquist. With fs = 22050 the edges land on FFT bins
  // 1,3,5,8,14,25,50,100,256. Groups: bands 0-1 low, 2-5 mid, 6-7 high.
  float bandEdgesHz[Config::kBands + 1] = {40.0f, 130.0f, 215.0f, 345.0f, 600.0f, 1075.0f, 2150.0f, 4300.0f, 0.0f};

  // --- Onset strength ---
  float logGain = 1000.0f;       // flux works on ln(1 + logGain * bandRms); band RMS is in full-scale-sine units. Also acts as a soft
                                 // gate: bands below ~1/logGain (-60 dBFS) barely move in the log domain.
  float onsetMeanTauS = 0.6f;    // slow mean subtracted from the flux (adaptive threshold)
  float tempoWeight[3] = {1.0f, 0.6f, 0.0f}; // how much each group's onsets feed tempo/phase estimation (hats off: they tick at 8ths/16ths)
  float onsetPeakReleaseS = 6.0f; // onset output is normalised by a slow peak follower
  float onsetPeakFloor = 1.0f;    // ...but never by less than this much flux (nepers), so noise doesn't fill the range

  // --- Noise gate / signal present ---
  float gateDb = -66.0f;         // RMS in dBFS (sine-referenced); host-adjustable
  float gateHysteresisDb = 4.0f; // signal turns on at gate + hysteresis, off below gate
  float gateHoldS = 0.6f;        // stays "present" this long after dropping below

  // --- AGC for the levels ---
  uint8_t agcMode = proto::kAgcNormal;
  float levelReleaseS = 0.12f;       // VU-style smoothing of the raw group level (attack is instantaneous)
  float agcAttackS = 0.010f;         // peak follower
  float agcReleaseS[4] = {8.0f, 3.0f, 20.0f, 8.0f}; // per AgcMode: normal / vivid / lazy / (off: unused)
  float agcFixedGain = 4.0f;         // AGC off: level = group amplitude * this
  float agcFloorDb = -54.0f;         // gain clamp: the AGC peak never goes below this (dBFS), so a band that is nearly empty (leakage,
                                     // noise) is never blown up to full scale; max gain = +54 dB

  // --- Tempo ---
  float bpmMin = 80.0f;
  float bpmMax = 170.0f;
  float bpmPrior = 120.0f;       // octave prior: log-normal around this
  float bpmPriorSigmaOct = 0.55f;
  uint16_t tempoEveryFrames = 22; // re-estimate every ~0.25 s
  float tempoMinWindowS = 3.0f;  // don't estimate before this much onset history exists
  float combWeights[4] = {1.0f, 0.8f, 0.6f, 0.5f}; // autocorrelation at 1, 2, 3, 4 beats
  float confLow = 0.08f;         // comb score mapped to confidence 0
  float confHigh = 0.40f;        // ...and 1
  float confRiseS = 1.0f;        // confidence smoothing: up
  float confFallS = 1.5f;        //                       down
  float lockOn = 0.5f;           // beat-locked hysteresis
  float lockOff = 0.25f;
  float tempoAcceptConf = 0.15f; // ignore tempo candidates below this raw confidence
  float tempoSmoothing = 0.3f;   // tempo estimate follows agreeing candidates by this fraction per update
  float tempoAgreeFraction = 0.03f; // candidates within this fraction of the current tempo are "the same"
  uint8_t tempoJumpCount = 3;    // a different tempo must win this many updates in a row before it replaces the current one
  // Metrical-level errors: a candidate at a simple non-octave ratio to the current tempo (3:2, 4:3, 5:4, 5:3 either way) is usually the
  // same music read at another level (a 3-against-2 pattern), not a new tempo, so it needs far more evidence.
  uint8_t tempoHarmonicJumpCount = 32;    // ...this many updates in a row (~8 s), or
  uint8_t tempoUnsupportedJumpCount = 12; // ...this many (~3 s) if the current tempo has lost its support (raw score < confLow) and the
  float tempoStrongRaw = 0.40f;           // candidate is strong (raw score >= this)
  float tempoHarmonicTol = 0.04f;         // ratio tolerance for the simple ratios above
  float continuityBoost = 0.25f;          // candidates at the current tempo (and its 2x / 0.5x) score up to this much higher...
  float continuitySigmaOct = 0.03f;       // ...falling off with this width (octaves) around them
  // Breakdown: the low band is well below its recent average (the kick dropped out). Tempo changes are not considered, the beat clock
  // free-runs, and `locked` is held, so a pattern in the remaining percussion cannot take over.
  float breakdownShortS = 1.0f;     // low-band amplitude average
  float breakdownLongS = 12.0f;     // ...against its slow average (outside a breakdown)
  float breakdownHeldLongS = 60.0f; // slow average time constant inside a breakdown, so a permanent change ends it after ~40 s
  float breakdownEnter = 0.4f;      // short / long amplitude ratio below which a breakdown starts (-8 dB)
  float breakdownExit = 0.6f;       // ...and above which it ends

  // --- Beat phase ---
  uint16_t phaseEveryFrames = 4; // measurement + PLL correction cadence (~46 ms)
  uint8_t phasePulses = 8;       // pulses in the correlation train
  float phasePulseDecay = 0.8f;  // older pulses count less (tempo error smears them)
  float pllGain = 0.10f;         // fraction of the phase error corrected per measurement
  float pllSnapError = 0.2f;     // |error| in beats beyond which...
  uint8_t pllSnapCount = 10;     // ...this many consecutive measurements make the clock snap to the measurement
  float pllTrimGain = 0.0003f;   // second-order term: integrates the phase error into a fractional correction of the beat period
  float pllTrimMax = 0.02f;      // ...limited to +-2% of the tempo estimate
  float pllTrimBoost = 2.0f;     // ...times this for the first pllTrimBoostS seconds locked after a new tempo
  float pllTrimBoostS = 12.0f;
  float clockFollow = 0.005f;     // per tempo update (0.25 s): how far the clock tempo moves towards the autocorrelation estimate
  // Time from a sound's attack until the onset peaks at the end of the frame that detects it. Measured with the synthetic kick in
  // test_tempo (see docs/design.md); beat phase is reported relative to the attack. Mostly (hop / 2 + window attack time).
  float onsetLatencySamples = 200.0f;

  // --- Bar position and energy ---
  float beatStrengthRate = 0.15f; // per-slot EMA of low-band onset strength at each beat of the bar
  float energyShortS = 15.0f;     // ~8 bars at 125 BPM
  float energyLongS = 180.0f;
};

class Analyzer {
public:
  explicit Analyzer(const Config &cfg = Config());

  // Feed raw 32-bit I2S slots from the INMP441 (24-bit data left-justified, i.e. value >> 8 is the signed 24-bit sample). Any n.
  void process(const int32_t *samples, size_t n);

  // Current features without clearing anything (onsets = the last hop's value). Timestamp = the last sample processed.
  const Features &features() const { return out_; }
  // Features as the host reads them: onsets are the peak since the previous call, which clears them.
  Features takeFeatures();

  // Host-driven controls (proto write registers)
  void setAgcMode(uint8_t mode);
  void setGateDb(float db) { cfg_.gateDb = db; }
  void setHostLatencyMs(int8_t ms) { hostLatencyMs_ = ms; }
  // "A beat is happening now". Seeds the clock phase; two taps within 0.35-0.75 s also seed the tempo.
  void tap();

  const Config &config() const { return cfg_; }
  // Hops processed so far
  uint32_t frameCount() const { return frame_; }
  // Per-band log spectrum of the last frame and per-group raw flux, for tooling
  const float *debugBandLog() const { return bandLog_; }
  const float *debugFlux() const { return flux_; }
  // Analysis latency in seconds from a sound's attack to the moment the beat clock can know about it (see docs/design.md)
  float onsetLatencySeconds() const { return cfg_.onsetLatencySamples / cfg_.sampleRate; }

  static constexpr size_t kOdfLen = 512;  // onset history: 512 frames = 5.9 s at 22.05 kHz / 256
  static constexpr size_t kMaxLag = 272;  // autocorrelation lags needed for 4 beats at bpmMin

private:
  void processFrame();
  void updateSignal(float rmsDb);
  void updateLevels(const float groupAmp[3]);
  void updateTempo();
  void estimateTempo(float &bpm, float &score, float &rawScore, float &currentRaw);
  void updateBreakdown(float lowAmp);
  float continuityGain(float bpm) const;
  bool isHarmonicRatio(float cand, float cur) const;
  void measurePhase();
  void commitBeat();
  void advanceClock();
  void buildOutput();
  float odfAt(float age) const;
  void setTempo(float bpm);
  float clockBpm() const { return clockBpm_; } // the beat clock's tempo (reported): the estimate, refined by the PLL

  Config cfg_;
  BEATSENSE_FFT_CLASS fft_;

  // Input stage
  float dcR_;
  float x1_ = 0, y1_ = 0;
  float buf_[Config::kFftSize] = {0};
  size_t fill_ = 0;
  float hopSumSq_ = 0;
  bool hopClipped_ = false;
  float hann_[Config::kFftSize];
  float win_[Config::kFftSize];
  float re_[Config::kFftSize / 2 + 1];
  float im_[Config::kFftSize / 2 + 1];

  // Bands and onset strength
  size_t bandLo_[Config::kBands];
  size_t bandHi_[Config::kBands]; // [lo, hi) FFT bins
  float bandLog_[Config::kBands] = {0};
  float prevBandLog_[Config::kBands] = {0};
  bool havePrevBands_ = false;
  float flux_[Config::kGroups] = {0};     // mean positive log-increase of the group's bands (nepers)
  float fluxMean_[Config::kGroups] = {0}; // slow mean of the flux
  float onsetCur_[Config::kGroups] = {0}; // flux above its mean (>= 0), this hop
  float onsetPeak_[Config::kGroups];      // slow peak follower used to scale the u8 onset output
  uint8_t onsetHold_[Config::kGroups] = {0};

  // Signal, clipping, levels
  bool signal_ = false;
  int holdLeft_ = 0;
  int clipLeft_ = 0;
  float levelSm_[Config::kGroups] = {0};
  float agcPeak_[Config::kGroups];
  float energyShort_ = 0, energyLong_ = 0;
  uint32_t energyCount_ = 0;

  // Onset detection function (low + mid onsets) history and tempo
  float odf_[kOdfLen] = {0};
  size_t odfHead_ = 0; // index of the newest frame
  float lin_[kOdfLen];
  float rn_[kMaxLag + 2];
  float scan_[400];
  bool haveTempo_ = false;
  float bpmAuto_ = 0;
  uint32_t trimFrames_ = 0; // frames of integral action since the last setTempo()
  float clockBpm_ = 0; // tempo the beat clock runs at: follows bpmAuto_ slowly, trimmed by the PLL's integral term
  float pendingBpm_ = 0;
  uint8_t pendingCount_ = 0;
  float confTarget_ = 0;
  float confSm_ = 0;
  bool locked_ = false;
  bool breakdown_ = false;
  float lowShort_ = 0, lowLong_ = 0;

  // Beat clock (phase in beats, position within the current beat)
  float phase_ = 0;
  float prevPhase_ = 0;
  bool phaseValid_ = false;
  uint8_t beatCount_ = 0;
  uint32_t beatIdx_ = 0;
  uint8_t snapCount_ = 0;
  float beatPeak_ = 0;
  float barStrength_[4] = {0, 0, 0, 0};
  uint32_t lastTapFrame_ = 0;
  bool haveTap_ = false;

  int8_t hostLatencyMs_ = 0;
  uint32_t frame_ = 0;
  float frameRate_;
  Features out_;

  // Derived time constants (per hop)
  float levelDecay_, agcAttack_, agcDecay_, fluxMeanAlpha_, onsetPeakDecay_, energyShortAlpha_, confRiseAlpha_, confFallAlpha_, bdShortAlpha_, bdLongAlpha_, bdHeldAlpha_;
};

// Frame as sent over I2C
inline proto::Frame toFrame(const Features &f) {
  proto::Frame fr;
  fr.status = f.status;
  const float q = f.bpm * 256.0f + 0.5f;
  fr.bpmQ88 = q <= 0 ? 0 : (q >= 65535.0f ? 65535 : (uint16_t)q);
  fr.beatPhase = f.beatPhase;
  fr.beatCount = f.beatCount;
  fr.beatInBar = f.beatInBar;
  fr.confidence = f.confidence;
  for (int i = 0; i < 3; i++) {
    fr.level[i] = f.levels[i];
    fr.onset[i] = f.onsets[i];
  }
  fr.energy = f.energy;
  return fr;
}

} // namespace beatsense

#endif // BEATSENSE_ANALYZER_H
