// SPDX-License-Identifier: EUPL-1.2
#include "analyzer.h"
#include <math.h>
#include <string.h>

namespace beatsense {

namespace {

constexpr float kTwoPi = 6.28318530717958647692f;
constexpr float kScanStepBpm = 0.25f;
constexpr size_t kScanMax = 400;

inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
inline float wrapSigned(float x) { // to (-0.5, 0.5]
  x -= floorf(x);
  return x > 0.5f ? x - 1.0f : x;
}
// Per-hop smoothing coefficient for a time constant
inline float alphaFor(float tauS, float hopS) { return 1.0f - expf(-hopS / tauS); }
inline uint8_t toU8(float v01) { return (uint8_t)(clampf(v01, 0.0f, 1.0f) * 255.0f + 0.5f); }

} // namespace

Analyzer::Analyzer(const Config &cfg) : cfg_(cfg) {
  const float hopS = (float)Config::kHop / cfg_.sampleRate;
  frameRate_ = 1.0f / hopS;
  dcR_ = 1.0f - kTwoPi * cfg_.dcCutoffHz / cfg_.sampleRate;
  for (size_t i = 0; i < Config::kFftSize; i++) {
    hann_[i] = 0.5f - 0.5f * cosf(kTwoPi * (float)i / (float)Config::kFftSize); // periodic Hann
  }
  // Band edges to FFT bins; bands are never empty and never overlap
  const float binHz = cfg_.sampleRate / (float)Config::kFftSize;
  size_t prev = 1;
  for (size_t b = 0; b < Config::kBands; b++) {
    const float hz = cfg_.bandEdgesHz[b];
    size_t lo = b == 0 ? (size_t)(hz / binHz + 0.5f) : prev;
    if (lo < 1) lo = 1;
    float hiHz = cfg_.bandEdgesHz[b + 1];
    size_t hi = hiHz <= 0 ? Config::kFftSize / 2 : (size_t)(hiHz / binHz + 0.5f);
    if (hi <= lo) hi = lo + 1;
    if (hi > Config::kFftSize / 2) hi = Config::kFftSize / 2;
    bandLo_[b] = lo;
    bandHi_[b] = hi;
    prev = hi;
  }
  bandHi_[Config::kBands - 1] = Config::kFftSize / 2; // up to (excluding) Nyquist
  levelDecay_ = expf(-hopS / cfg_.levelReleaseS);
  agcAttack_ = alphaFor(cfg_.agcAttackS, hopS);
  fluxMeanAlpha_ = alphaFor(cfg_.onsetMeanTauS, hopS);
  onsetPeakDecay_ = expf(-hopS / cfg_.onsetPeakReleaseS);
  energyShortAlpha_ = alphaFor(cfg_.energyShortS, hopS);
  confRiseAlpha_ = alphaFor(cfg_.confRiseS, hopS);
  confFallAlpha_ = alphaFor(cfg_.confFallS, hopS);
  setAgcMode(cfg_.agcMode);
  const float floorAmp = powf(10.0f, cfg_.agcFloorDb / 20.0f);
  for (size_t g = 0; g < Config::kGroups; g++) {
    agcPeak_[g] = floorAmp;
    onsetPeak_[g] = cfg_.onsetPeakFloor;
  }
  memset(lin_, 0, sizeof(lin_));
  memset(rn_, 0, sizeof(rn_));
  memset(scan_, 0, sizeof(scan_));
}

void Analyzer::setAgcMode(uint8_t mode) {
  if (mode > proto::kAgcOff) mode = proto::kAgcNormal;
  cfg_.agcMode = mode;
  agcDecay_ = expf(-((float)Config::kHop / cfg_.sampleRate) / cfg_.agcReleaseS[mode]);
}

void Analyzer::process(const int32_t *samples, size_t n) {
  const float clip = cfg_.clipLevel;
  for (size_t i = 0; i < n; i++) {
    // 32-bit slot, 24-bit data left-justified: arithmetic shift gives the signed 24-bit value
    const float s = (float)(samples[i] >> 8) * (1.0f / 8388608.0f);
    if (s >= clip || s <= -clip) {
      hopClipped_ = true;
    }
    const float y = s - x1_ + dcR_ * y1_; // y[n] = x[n] - x[n-1] + R y[n-1]
    x1_ = s;
    y1_ = y;
    hopSumSq_ += y * y;
    buf_[Config::kHop + fill_] = y;
    if (++fill_ == Config::kHop) {
      processFrame();
      memmove(buf_, buf_ + Config::kHop, Config::kHop * sizeof(float));
      fill_ = 0;
      hopSumSq_ = 0;
      hopClipped_ = false;
    }
  }
}

void Analyzer::updateSignal(float rmsDb) {
  const int holdFrames = (int)(cfg_.gateHoldS * frameRate_);
  if (rmsDb > cfg_.gateDb + cfg_.gateHysteresisDb) {
    signal_ = true;
    holdLeft_ = holdFrames;
  } else if (signal_) {
    if (rmsDb > cfg_.gateDb) {
      holdLeft_ = holdFrames;
    } else if (--holdLeft_ <= 0) {
      signal_ = false;
    }
  }
}

void Analyzer::updateLevels(const float groupAmp[3]) {
  const float floorAmp = powf(10.0f, cfg_.agcFloorDb / 20.0f);
  for (size_t g = 0; g < Config::kGroups; g++) {
    float s = levelSm_[g] * levelDecay_;
    if (groupAmp[g] > s) s = groupAmp[g];
    levelSm_[g] = s;
    if (!signal_) {
      out_.levels[g] = 0;
      continue;
    }
    float v;
    if (cfg_.agcMode == proto::kAgcOff) {
      v = s * cfg_.agcFixedGain;
    } else {
      float pk = agcPeak_[g];
      pk = s > pk ? pk + agcAttack_ * (s - pk) : pk * agcDecay_;
      if (pk < floorAmp) pk = floorAmp; // gain clamp: never amplify beyond the gain limit
      agcPeak_[g] = pk;
      v = s / pk;
    }
    out_.levels[g] = toU8(v);
  }
}

void Analyzer::processFrame() {
  frame_++;
  const float hopS = (float)Config::kHop / cfg_.sampleRate;

  // --- Spectrum ---
  for (size_t i = 0; i < Config::kFftSize; i++) {
    win_[i] = buf_[i] * hann_[i];
  }
  fft_.forward(win_, re_, im_);
  // Normalise so a full-scale sine reads 1.0 in its bin: Hann sum is N/2, a sine's peak bin is A * sum / 2
  const float norm = 1.0f / (0.25f * (float)Config::kFftSize);
  const float normSq = norm * norm;

  static const uint8_t kGroupOf[Config::kBands] = {0, 0, 1, 1, 1, 1, 2, 2};
  float groupPow[Config::kGroups] = {0, 0, 0};
  float groupBands[Config::kGroups] = {0, 0, 0};
  for (size_t b = 0; b < Config::kBands; b++) {
    float pw = 0;
    for (size_t k = bandLo_[b]; k < bandHi_[b]; k++) {
      pw += (re_[k] * re_[k] + im_[k] * im_[k]) * normSq;
    }
    const float rms = sqrtf(pw / (float)(bandHi_[b] - bandLo_[b]));
    bandLog_[b] = logf(1.0f + cfg_.logGain * rms);
    groupPow[kGroupOf[b]] += pw;
    groupBands[kGroupOf[b]] += 1.0f;
  }

  // --- Signal, clipping, level ---
  const float meanSq = hopSumSq_ / (float)Config::kHop;
  const float rmsDb = 10.0f * log10f(2.0f * meanSq + 1e-12f);
  updateSignal(rmsDb);
  if (hopClipped_) clipLeft_ = (int)(cfg_.clipHoldS * frameRate_);
  else if (clipLeft_ > 0) clipLeft_--;

  float groupAmp[3];
  for (size_t g = 0; g < Config::kGroups; g++) groupAmp[g] = sqrtf(groupPow[g]);
  updateLevels(groupAmp);

  // --- Onset strength: log-compressed spectral flux per group ---
  float odf = 0;
  for (size_t g = 0; g < Config::kGroups; g++) flux_[g] = 0;
  if (havePrevBands_) {
    for (size_t b = 0; b < Config::kBands; b++) {
      const float d = bandLog_[b] - prevBandLog_[b];
      if (d > 0) flux_[kGroupOf[b]] += d;
    }
    for (size_t g = 0; g < Config::kGroups; g++) flux_[g] /= groupBands[g];
  }
  memcpy(prevBandLog_, bandLog_, sizeof(bandLog_));
  havePrevBands_ = true;
  for (size_t g = 0; g < Config::kGroups; g++) {
    float o = signal_ ? flux_[g] - fluxMean_[g] : 0.0f; // above the slow mean, so steady noise-like flux doesn't count
    if (o < 0) o = 0;
    fluxMean_[g] += fluxMeanAlpha_ * (flux_[g] - fluxMean_[g]);
    onsetCur_[g] = o;
    odf += cfg_.tempoWeight[g] * o;
    // u8 output scaled by a slow peak follower so it spans 0-255 whatever the music
    float pk = onsetPeak_[g] * onsetPeakDecay_;
    if (o > pk) pk = o;
    if (pk < cfg_.onsetPeakFloor) pk = cfg_.onsetPeakFloor;
    onsetPeak_[g] = pk;
    out_.onsets[g] = toU8(o / pk);
    if (out_.onsets[g] > onsetHold_[g]) onsetHold_[g] = out_.onsets[g];
  }
  odfHead_ = (odfHead_ + 1) % kOdfLen;
  odf_[odfHead_] = odf;

  // --- Energy: ~8 bars vs. the long-term average ---
  if (signal_) {
    const float amp = sqrtf(meanSq);
    if (energyCount_ == 0) {
      energyShort_ = energyLong_ = amp;
    } else {
      energyShort_ += energyShortAlpha_ * (amp - energyShort_);
      const float longA = fmaxf(1.0f / (float)energyCount_, hopS / cfg_.energyLongS); // plain average until the window has filled
      energyLong_ += longA * (amp - energyLong_);
    }
    energyCount_++;
    const float ratio = energyShort_ / fmaxf(energyLong_, 1e-9f);
    out_.energy = toU8((128.0f + 64.0f * log2f(fmaxf(ratio, 1e-3f))) / 255.0f);
  }

  // --- Tempo and phase ---
  if (haveTempo_) advanceClock();
  if (frame_ % cfg_.tempoEveryFrames == 0) updateTempo();
  {
    const float target = signal_ ? confTarget_ : 0.0f;
    confSm_ += (target > confSm_ ? confRiseAlpha_ : confFallAlpha_) * (target - confSm_);
    if (!locked_ && confSm_ > cfg_.lockOn && haveTempo_) locked_ = true;
    else if (locked_ && (confSm_ < cfg_.lockOff || !haveTempo_)) locked_ = false;
  }
  if (haveTempo_ && frame_ % cfg_.phaseEveryFrames == 0) measurePhase();

  // --- Bar position: strength of the low-band onset around each beat ---
  if (haveTempo_) {
    if (phase_ >= 0.75f || phase_ < 0.25f) beatPeak_ = fmaxf(beatPeak_, onsetCur_[0]);
    if (prevPhase_ < 0.25f && phase_ >= 0.25f) commitBeat();
  }
  prevPhase_ = phase_;

  buildOutput();
}

float Analyzer::odfAt(float age) const {
  if (age < 0.0f || age > (float)(kOdfLen - 2)) return 0.0f;
  const size_t i = (size_t)age;
  const float f = age - (float)i;
  const float a = odf_[(odfHead_ + kOdfLen - i) % kOdfLen];
  const float b = odf_[(odfHead_ + kOdfLen - i - 1) % kOdfLen];
  return a + f * (b - a);
}

// Autocorrelation of the onset signal over the last ~6 s, scored by a comb over 1-4 beat lags at fine BPM steps (the comb gives
// 4x the resolution of the single-lag peak), weighted by the octave prior, then parabolic interpolation of the best candidate.
void Analyzer::estimateTempo(float &bpm, float &score, float &rawScore) {
  bpm = score = rawScore = 0;
  const size_t n = frame_ < kOdfLen ? (size_t)frame_ : kOdfLen;
  for (size_t i = 0; i < n; i++) lin_[i] = odf_[(odfHead_ + kOdfLen - (n - 1 - i)) % kOdfLen]; // oldest first
  float mean = 0;
  for (size_t i = 0; i < n; i++) mean += lin_[i];
  mean /= (float)n;
  for (size_t i = 0; i < n; i++) lin_[i] -= mean;
  // 3-tap smoothing: onsets land on frame boundaries, a lag of m beats is rarely an integer
  float p = lin_[0];
  for (size_t i = 1; i + 1 < n; i++) {
    const float cur = lin_[i];
    lin_[i] = 0.25f * p + 0.5f * cur + 0.25f * lin_[i + 1];
    p = cur;
  }
  float r0 = 0;
  for (size_t i = 0; i < n; i++) r0 += lin_[i] * lin_[i];
  if (r0 < 1e-9f) return;
  const size_t maxLag = n - 1 < kMaxLag ? n - 1 : kMaxLag;
  for (size_t l = 0; l <= kMaxLag; l++) {
    if (l > maxLag) {
      rn_[l] = 0;
      continue;
    }
    float s = 0;
    for (size_t i = l; i < n; i++) s += lin_[i] * lin_[i - l];
    rn_[l] = (s / (float)(n - l)) / (r0 / (float)n); // correlation coefficient at this lag
  }
  rn_[kMaxLag + 1] = 0;

  size_t count = (size_t)((cfg_.bpmMax - cfg_.bpmMin) / kScanStepBpm) + 1;
  if (count > kScanMax) count = kScanMax;
  float wsum = 0;
  for (int m = 0; m < 4; m++) wsum += cfg_.combWeights[m];
  size_t best = 0;
  float bestW = -1e9f;
  for (size_t i = 0; i < count; i++) {
    const float b = cfg_.bpmMin + (float)i * kScanStepBpm;
    const float lag = 60.0f * frameRate_ / b;
    float s = 0, w = 0;
    for (int m = 1; m <= 4; m++) {
      const float l = lag * (float)m;
      if (l >= (float)maxLag) continue;
      const size_t li = (size_t)l;
      const float f = l - (float)li;
      s += cfg_.combWeights[m - 1] * (rn_[li] + f * (rn_[li + 1] - rn_[li]));
      w += cfg_.combWeights[m - 1];
    }
    s = w > 0 ? s / wsum : 0.0f; // missing terms count as zero, which favours tempi whose lags fit the window
    scan_[i] = s;
    const float oct = log2f(b / cfg_.bpmPrior) / cfg_.bpmPriorSigmaOct;
    const float ws = s * expf(-0.5f * oct * oct);
    if (ws > bestW) {
      bestW = ws;
      best = i;
    }
  }
  // Parabolic interpolation on the prior-weighted scores around the best candidate
  float delta = 0;
  if (best > 0 && best + 1 < count) {
    auto weighted = [&](size_t i) {
      const float o = log2f((cfg_.bpmMin + (float)i * kScanStepBpm) / cfg_.bpmPrior) / cfg_.bpmPriorSigmaOct;
      return scan_[i] * expf(-0.5f * o * o);
    };
    const float a = weighted(best - 1), b = weighted(best), c = weighted(best + 1);
    const float den = a - 2.0f * b + c;
    if (den < -1e-9f) delta = clampf(0.5f * (a - c) / den, -1.0f, 1.0f);
  }
  bpm = cfg_.bpmMin + ((float)best + delta) * kScanStepBpm;
  score = bestW;
  rawScore = scan_[best];
}

void Analyzer::setTempo(float bpm) {
  bpmAuto_ = bpm;
  haveTempo_ = true;
  pendingCount_ = 0;
}

void Analyzer::updateTempo() {
  if (!signal_ || frame_ < (uint32_t)(cfg_.tempoMinWindowS * frameRate_)) {
    confTarget_ = 0;
    return;
  }
  float cand, score, raw;
  estimateTempo(cand, score, raw);
  confTarget_ = clampf((raw - cfg_.confLow) / (cfg_.confHigh - cfg_.confLow), 0.0f, 1.0f);
  if (cand <= 0 || confTarget_ < cfg_.tempoAcceptConf) return;
  if (!haveTempo_) {
    setTempo(cand);
    return;
  }
  if (fabsf(cand - bpmAuto_) < cfg_.tempoAgreeFraction * bpmAuto_) {
    bpmAuto_ += cfg_.tempoSmoothing * (cand - bpmAuto_);
    pendingCount_ = 0;
    return;
  }
  // A different tempo has to keep winning before it replaces the current one (no flip-flopping between octave candidates)
  if (pendingCount_ > 0 && fabsf(cand - pendingBpm_) < cfg_.tempoAgreeFraction * pendingBpm_) {
    pendingCount_++;
    pendingBpm_ += 0.5f * (cand - pendingBpm_);
  } else {
    pendingBpm_ = cand;
    pendingCount_ = 1;
  }
  if (pendingCount_ >= cfg_.tempoJumpCount) {
    setTempo(pendingBpm_);
    for (int i = 0; i < 4; i++) barStrength_[i] = 0; // the bar grid belongs to the old tempo
    snapCount_ = cfg_.pllSnapCount; // the phase is meaningless at the new tempo: re-measure and snap on the next chance
  }
}

// Phase: correlate the onset signal with a pulse train at the current beat period (the most recent beats weigh most) to find where
// the last beat was, then pull the free-running beat clock towards it (PLL). Between measurements the clock predicts.
void Analyzer::measurePhase() {
  const float period = 60.0f * frameRate_ / bpmAuto_; // frames per beat
  const size_t slots = (size_t)ceilf(period);
  if (slots > 128) return;
  float best = 0, sum = 0;
  float s[128];
  size_t bi = 0;
  for (size_t t = 0; t < slots; t++) {
    float v = 0, w = 1.0f;
    for (uint8_t j = 0; j < cfg_.phasePulses; j++) {
      const float a = (float)t + (float)j * period;
      v += w * (0.25f * odfAt(a - 1.0f) + 0.5f * odfAt(a) + 0.25f * odfAt(a + 1.0f));
      w *= cfg_.phasePulseDecay;
    }
    s[t] = v;
    sum += v;
    if (v > best) {
      best = v;
      bi = t;
    }
  }
  if (best <= 1e-6f) return;
  // Parabolic refinement (circular neighbours)
  const float a = s[(bi + slots - 1) % slots], c = s[(bi + 1) % slots];
  const float den = a - 2.0f * best + c;
  float frac = (float)bi;
  if (den < -1e-9f) frac += clampf(0.5f * (a - c) / den, -1.0f, 1.0f);
  const float latencyFrames = cfg_.onsetLatencySamples / (float)Config::kHop;
  const float meas = (frac + latencyFrames) / period; // beats since the beat, as of the newest frame
  const float err = wrapSigned(meas - phase_);

  float delta = 0;
  if (!phaseValid_ || snapCount_ >= cfg_.pllSnapCount) {
    delta = err; // first measurement, or a new tempo: take it as is
    phaseValid_ = true;
    snapCount_ = 0;
  } else if (fabsf(err) > cfg_.pllSnapError) {
    // Far off: only act if it persists (a lone wrong measurement must not throw the clock)
    if (++snapCount_ >= cfg_.pllSnapCount) {
      delta = err;
      snapCount_ = 0;
    }
  } else {
    snapCount_ = 0;
    delta = cfg_.pllGain * err;
  }
  phase_ += delta;
  if (phase_ >= 1.0f) {
    phase_ -= 1.0f;
    beatCount_++;
    beatIdx_++;
  } else if (phase_ < 0.0f) {
    phase_ += 1.0f;
    beatCount_--;
    beatIdx_--;
  }
}

void Analyzer::advanceClock() {
  phase_ += bpmAuto_ / 60.0f / frameRate_;
  while (phase_ >= 1.0f) {
    phase_ -= 1.0f;
    beatCount_++;
    beatIdx_++;
  }
}

void Analyzer::commitBeat() {
  float &slot = barStrength_[beatIdx_ & 3];
  slot += cfg_.beatStrengthRate * (beatPeak_ - slot);
  beatPeak_ = 0;
}

void Analyzer::tap() {
  const uint32_t interval = frame_ - lastTapFrame_;
  if (haveTap_ && interval >= (uint32_t)(0.35f * frameRate_) && interval <= (uint32_t)(0.75f * frameRate_)) {
    float bpm = 60.0f * frameRate_ / (float)interval;
    if (haveTempo_ && fabsf(bpm - bpmAuto_) > 0.03f * bpmAuto_) setTempo(bpm);
    else if (!haveTempo_) setTempo(bpm);
  }
  lastTapFrame_ = frame_;
  haveTap_ = true;
  if (haveTempo_) {
    phase_ = 0; // the tap is a beat
    prevPhase_ = 0;
    phaseValid_ = true;
    snapCount_ = 0;
    beatCount_++;
    beatIdx_++;
    buildOutput();
  }
}

void Analyzer::buildOutput() {
  Features &o = out_;
  o.bpm = haveTempo_ ? bpmAuto_ : 0.0f;
  float p = phase_;
  uint8_t count = beatCount_;
  uint32_t idx = beatIdx_;
  if (haveTempo_ && hostLatencyMs_ != 0) {
    p += (float)hostLatencyMs_ * 0.001f * bpmAuto_ / 60.0f;
    const float fl = floorf(p);
    p -= fl;
    count = (uint8_t)(count + (int)fl);
    idx += (int32_t)fl;
  }
  o.beatPhase = (uint16_t)clampf(p * 65536.0f, 0.0f, 65535.0f);
  o.beatCount = count;
  int downbeat = 0;
  for (int i = 1; i < 4; i++) {
    if (barStrength_[i] > barStrength_[downbeat]) downbeat = i;
  }
  o.beatInBar = (uint8_t)((idx - (uint32_t)downbeat) & 3);
  o.confidence = toU8(confSm_);
  o.status = (signal_ ? proto::kStatusSignal : 0) | (locked_ ? proto::kStatusLocked : 0) | (clipLeft_ > 0 ? proto::kStatusClipping : 0);
}

Features Analyzer::takeFeatures() {
  Features f = out_;
  for (size_t g = 0; g < Config::kGroups; g++) {
    f.onsets[g] = onsetHold_[g];
    onsetHold_[g] = 0;
  }
  return f;
}

} // namespace beatsense
