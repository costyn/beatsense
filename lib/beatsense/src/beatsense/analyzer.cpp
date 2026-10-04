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
  bdShortAlpha_ = alphaFor(cfg_.breakdownShortS, hopS);
  bdLongAlpha_ = alphaFor(cfg_.breakdownLongS, hopS);
  bdHeldAlpha_ = alphaFor(cfg_.breakdownHeldLongS, hopS);
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
  const float invHeadroom = 1.0f / cfg_.levelHeadroom;
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
      v = s * cfg_.agcFixedGain * invHeadroom;
    } else {
      float pk = agcPeak_[g];
      pk = s > pk ? pk + agcAttack_ * (s - pk) : pk * agcDecay_;
      if (pk < floorAmp) pk = floorAmp; // gain clamp: never amplify beyond the gain limit
      agcPeak_[g] = pk;
      v = s / pk * invHeadroom;
    }
    out_.levels[g] = toU8(v);
  }
}

// Breakdown = the low band (kick) is well below its recent average while there is still signal. The slow average barely moves during
// a breakdown, so the kick returning ends it at once; after a permanent change it catches up and the breakdown ends by itself.
void Analyzer::updateBreakdown(float lowAmp) {
  if (!signal_) {
    breakdown_ = false;
    lowShort_ = lowLong_ = 0;
    return;
  }
  if (lowLong_ <= 0) {
    lowShort_ = lowLong_ = lowAmp;
    return;
  }
  lowShort_ += bdShortAlpha_ * (lowAmp - lowShort_);
  lowLong_ += (breakdown_ ? bdHeldAlpha_ : bdLongAlpha_) * (lowAmp - lowLong_);
  const float ratio = lowShort_ / fmaxf(lowLong_, 1e-9f);
  if (!breakdown_ && ratio < cfg_.breakdownEnter) breakdown_ = true;
  else if (breakdown_ && ratio > cfg_.breakdownExit) breakdown_ = false;
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
  updateBreakdown(groupAmp[0]);

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
    else if (locked_ && ((confSm_ < cfg_.lockOff && !breakdown_) || !haveTempo_)) locked_ = false; // a breakdown keeps the lock
  }
  if (haveTempo_ && !breakdown_ && frame_ % cfg_.phaseEveryFrames == 0) measurePhase(); // breakdown: free-run on prediction

  // --- Bar position: strength of the low-band onset around each beat ---
  if (haveTempo_) {
    if (phase_ >= 0.75f || phase_ < 0.25f) {
      float on, lv;
      accentNow(on, lv);
      beatOnset_ += on;
      beatLevel_ = fmaxf(beatLevel_, lv);
    }
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
void Analyzer::estimateTempo(float &bpm, float &score, float &rawScore, float &currentRaw) {
  bpm = score = rawScore = currentRaw = 0;
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
      // Catmull-Rom cubic through the 4 nearest lags. Linear interpolation of a peak that is only 3-4 samples wide has its maximum
      // at the integer lags, and all four comb terms round the same way (37 frames/beat for 36.9 = 139.68 BPM): a bias of up to
      // half a frame per beat (-0.17% at 140 BPM). The cubic follows the peak shape between the samples.
      const float y0 = rn_[li - 1], y1 = rn_[li], y2 = rn_[li + 1], y3 = rn_[li + 2];
      const float v = y1 + 0.5f * f * (y2 - y0 + f * (2.0f * y0 - 5.0f * y1 + 4.0f * y2 - y3 + f * (3.0f * (y1 - y2) + y3 - y0)));
      s += cfg_.combWeights[m - 1] * v;
      w += cfg_.combWeights[m - 1];
    }
    s = w > 0 ? s / wsum : 0.0f; // missing terms count as zero, which favours tempi whose lags fit the window
    scan_[i] = s;
    const float oct = log2f(b / cfg_.bpmPrior) / cfg_.bpmPriorSigmaOct;
    float ws = s * expf(-0.5f * oct * oct);
    if (haveTempo_) ws *= continuityGain(b);
    if (ws > bestW) {
      bestW = ws;
      best = i;
    }
  }
  // Parabolic interpolation on the RAW comb scores around the best candidate. The octave prior and the continuity gain slope across
  // the peak (the prior falls ~1% per 1% of tempo at 140 BPM), so interpolating the weighted scores pulled the estimate towards the
  // prior centre (-0.17% at 140). The prior only picks the peak; the position within it comes from the raw scores. The raw maximum
  // within +-3 steps of the picked index is used, since the weights may have picked a shoulder of the same peak.
  size_t peak = best;
  for (size_t i = (best >= 3 ? best - 3 : 0); i <= best + 3 && i < count; i++) {
    if (scan_[i] > scan_[peak]) peak = i;
  }
  float delta = 0;
  if (peak > 0 && peak + 1 < count && scan_[peak - 1] > 1e-6f && scan_[peak + 1] > 1e-6f) { // Gaussian (log-parabola) interpolation
    const float a = logf(scan_[peak - 1]), b = logf(scan_[peak]), c = logf(scan_[peak + 1]);
    const float den = a - 2.0f * b + c;
    if (den < -1e-9f) delta = clampf(0.5f * (a - c) / den, -1.0f, 1.0f);
  }
  best = peak;
  bpm = cfg_.bpmMin + ((float)best + delta) * kScanStepBpm;
  score = bestW;
  rawScore = scan_[best];
  if (haveTempo_) { // support of the current tempo: best raw score within the agreement window
    for (size_t i = 0; i < count; i++) {
      const float b = cfg_.bpmMin + (float)i * kScanStepBpm;
      if (fabsf(b - bpmAuto_) <= cfg_.tempoAgreeFraction * bpmAuto_ && scan_[i] > currentRaw) currentRaw = scan_[i];
    }
  }
}

// Continuity prior: tempi at the current one, or an octave away, score a bit higher than unrelated ones
float Analyzer::continuityGain(float bpm) const {
  const float d = log2f(bpm / bpmAuto_);
  float g = 0;
  for (int k = -1; k <= 1; k++) {
    const float x = (d - (float)k) / cfg_.continuitySigmaOct;
    const float v = expf(-0.5f * x * x);
    if (v > g) g = v;
  }
  return 1.0f + cfg_.continuityBoost * g;
}

// Candidate at a simple non-octave ratio to the current tempo: 3:2, 4:3, 5:4, 5:3 (either way)
bool Analyzer::isHarmonicRatio(float cand, float cur) const {
  const float r = cand > cur ? cand / cur : cur / cand;
  static const float kRatios[] = {1.5f, 4.0f / 3.0f, 1.25f, 5.0f / 3.0f};
  for (float k : kRatios) {
    if (fabsf(r - k) < cfg_.tempoHarmonicTol * k) return true;
  }
  return false;
}

void Analyzer::setTempo(float bpm) {
  bpmAuto_ = bpm;
  clockBpm_ = bpm;
  trimFrames_ = 0;
  haveTempo_ = true;
  pendingCount_ = 0;
}

void Analyzer::updateTempo() {
  if (!signal_ || frame_ < (uint32_t)(cfg_.tempoMinWindowS * frameRate_)) {
    confTarget_ = 0;
    return;
  }
  float cand, score, raw, curRaw;
  estimateTempo(cand, score, raw, curRaw);
  // Confidence is the evidence for the tempo we report: the current tempo's support if there is one (a breakdown or a pending change
  // lowers it), otherwise the best candidate's
  const float evidence = haveTempo_ ? curRaw : raw;
  confTarget_ = clampf((evidence - cfg_.confLow) / (cfg_.confHigh - cfg_.confLow), 0.0f, 1.0f);
  const float candConf = clampf((raw - cfg_.confLow) / (cfg_.confHigh - cfg_.confLow), 0.0f, 1.0f);
  if (cand <= 0 || candConf < cfg_.tempoAcceptConf) return;
  if (!haveTempo_) {
    setTempo(cand);
    return;
  }
  if (breakdown_) { // the kick is gone: whatever the rest of the percussion says is neither a new tempo nor a refinement
    pendingCount_ = 0;
    return;
  }
  if (fabsf(cand - bpmAuto_) < cfg_.tempoAgreeFraction * bpmAuto_) {
    bpmAuto_ += cfg_.tempoSmoothing * (cand - bpmAuto_);
    clockBpm_ += cfg_.clockFollow * (bpmAuto_ - clockBpm_); // the clock only leans on the estimate; the PLL decides its tempo
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
  uint8_t need = cfg_.tempoJumpCount;
  if (isHarmonicRatio(pendingBpm_, bpmAuto_)) {
    need = cfg_.tempoHarmonicJumpCount;
    if (curRaw < cfg_.confLow && raw >= cfg_.tempoStrongRaw) need = cfg_.tempoUnsupportedJumpCount;
  }
  if (pendingCount_ >= need) {
    setTempo(pendingBpm_);
    resetBar(); // the bar grid belongs to the old tempo
    snapCount_ = cfg_.pllSnapCount; // the phase is meaningless at the new tempo: re-measure and snap on the next chance
  }
}

// Phase: correlate the onset signal with a pulse train at the current beat period (the most recent beats weigh most) to find where
// the last beat was, then pull the free-running beat clock towards it (PLL). Between measurements the clock predicts.
void Analyzer::measurePhase() {
  // Pulse train at the ESTIMATED period, not the clock's: the train's peak is biased by the period error (older pulses drift off the
  // kicks), and if that depended on the clock's own tempo the integral term below would feed back on itself (positive feedback,
  // slow runaway at slow tempi). With the estimate the bias is a constant (about 1 ms), which the integral term ignores.
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
    // Second order: a steady phase error means the beat period is off. Steady state of the first-order loop is err = d / pllGain * (frames
    // per measurement / period) for a tempo error d, so err * period / phaseEveryFrames estimates d / pllGain; the trim integrates it.
    // Only while locked: leftover percussion without a lock must not retune the clock.
    // (The tempo estimate changes clockBpm_ only through clockFollow, so its +-0.1 BPM jitter does not reach the output.)
    if (locked_) {
      // Faster for the first seconds after a (re)start, when the estimate it started from is least accurate
      const float boost = trimFrames_ < (uint32_t)(cfg_.pllTrimBoostS * frameRate_) ? cfg_.pllTrimBoost : 1.0f;
      trimFrames_ += cfg_.phaseEveryFrames;
      clockBpm_ *= 1.0f + boost * cfg_.pllTrimGain * clampf(err, -0.1f, 0.1f) * period / (float)cfg_.phaseEveryFrames;
      clockBpm_ = clampf(clockBpm_, bpmAuto_ * (1.0f - cfg_.pllTrimMax), bpmAuto_ * (1.0f + cfg_.pllTrimMax));
    }
  }
  phase_ += delta;
  if (phase_ >= 1.0f) {
    phase_ -= 1.0f;
    stepBeat(1);
  } else if (phase_ < 0.0f) {
    phase_ += 1.0f;
    stepBeat(-1);
  }
}

void Analyzer::advanceClock() {
  phase_ += clockBpm() / 60.0f / frameRate_;
  while (phase_ >= 1.0f) {
    phase_ -= 1.0f;
    stepBeat(1);
  }
}

// Accent of the current hop, per group: the onset relative to the group's own recent peak (a new element, a crash) and the level
// relative to its peak (the log-compressed onset is blind to how much louder a kick is, the level is not). Low band counts most.
// The onset part is SUMMED over the beat window and the level part maximised: an attack that straddles two hops splits its onset
// between them in a way that depends on the tempo and the hop grid, so a maximum would make some beats of the bar look stronger.
void Analyzer::accentNow(float &onset, float &level) const {
  onset = level = 0;
  for (size_t g = 0; g < Config::kGroups; g++) {
    onset += cfg_.barWeight[g] * fminf(onsetCur_[g] / onsetPeak_[g], 1.0f);
    level += cfg_.barWeight[g] * (float)out_.levels[g] * (1.0f / 255.0f);
  }
}

void Analyzer::resetBar() {
  for (int i = 0; i < 4; i++) {
    barSlot_[i] = 0;
    barObs_[i] = 0;
    barVar_[i] = 0;
  }
  barSet_ = false;
  barCand_ = -1;
  barCandBars_ = 0;
  barPending_ = -1;
}

// One beat of the clock. The bar position steps 0,1,2,3,0... in lockstep with it; the only other thing that moves it is a pending
// re-alignment, applied here, at a beat boundary, once.
void Analyzer::stepBeat(int dir) {
  beatCount_ = (uint8_t)(beatCount_ + dir);
  beatIdx_ += (uint32_t)dir;
  if (dir > 0 && barPending_ >= 0) {
    const uint8_t pos = (uint8_t)((beatIdx_ - (uint32_t)barPending_) & 3);
    if (pos != ((barPos_ + 1) & 3) && barEver_) barRealigns_++;
    barPos_ = pos;
    barEver_ = true;
    barPending_ = -1;
  } else {
    barPos_ = (uint8_t)((barPos_ + dir) & 3);
  }
}

// The accent around each beat is averaged per slot of the absolute beat index (EMA, a few bars). The downbeat is the strongest slot, but
// a stable grid matters more than a correct one: the first choice is taken once every slot has been seen twice, and after that the
// grid only moves when one other slot beats ALL the others by a clear margin at four bar boundaries in a row. Evaluated once per
// bar (on the last beat of the bar), applied at the next beat.
void Analyzer::commitBeat() {
  const float peak = cfg_.barOnsetScale * beatOnset_ + beatLevel_;
  beatOnset_ = beatLevel_ = 0;
  if (!locked_ || breakdown_ || !signal_) return; // no kick, no evidence (and the grid free-runs)
  const size_t slot = beatIdx_ & 3;
  if (barObs_[slot] == 0) {
    barSlot_[slot] = peak;
    barVar_[slot] = 0.01f * peak * peak; // prior: 10% bar-to-bar scatter
  } else {
    const float d = peak - barSlot_[slot];
    barVar_[slot] += cfg_.barVarRate * (d * d - barVar_[slot]);
    barSlot_[slot] += cfg_.barRate * d;
  }
  if (barObs_[slot] < 255) barObs_[slot]++;
  if (barPos_ != 3 || barPending_ >= 0) return; // decide once per bar
  size_t best = 0;
  for (size_t i = 1; i < 4; i++) {
    if (barSlot_[i] > barSlot_[best]) best = i;
  }
  const size_t cur = (beatIdx_ + 1) & 3; // slot of the next beat, which is the current downbeat (barPos_ == 3 now)
  if (!barSet_) {
    for (size_t i = 0; i < 4; i++) {
      if (barObs_[i] < cfg_.barInitObs) return;
    }
    barSet_ = true;
    barPending_ = (int8_t)best; // first choice: no margin, the grid has to start somewhere
    return;
  }
  size_t second = best == 0 ? 1 : 0;
  for (size_t i = 0; i < 4; i++) {
    if (i != best && barSlot_[i] > barSlot_[second]) second = i;
  }
  // "Clearly": by a relative margin (the hop grid biases some beats by a few %) and by several standard deviations of the difference of
  // the two slot averages (an average of independent bars scatters by sigma * sqrt(rate / (2 - rate)))
  const float diff = barSlot_[best] - barSlot_[second];
  const float sigmaDiff = sqrtf((barVar_[best] + barVar_[second]) * cfg_.barRate / (2.0f - cfg_.barRate));
  if (best != cur && barSlot_[best] > cfg_.barMargin * barSlot_[second] && diff > cfg_.barSigmas * sigmaDiff) {
    if (barCand_ == (int8_t)best) barCandBars_++;
    else {
      barCand_ = (int8_t)best;
      barCandBars_ = 1;
    }
    if (barCandBars_ >= cfg_.barSustainBars) {
      barPending_ = (int8_t)best;
      barCand_ = -1;
      barCandBars_ = 0;
    }
  } else {
    barCand_ = -1;
    barCandBars_ = 0;
  }
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
    stepBeat(1);
    buildOutput();
  }
}

void Analyzer::buildOutput() {
  Features &o = out_;
  o.bpm = haveTempo_ ? clockBpm() : 0.0f;
  float p = phase_;
  uint8_t count = beatCount_;
  int ahead = 0; // beats the host's latency offset puts the read ahead of the clock
  if (haveTempo_ && hostLatencyMs_ != 0) {
    p += (float)hostLatencyMs_ * 0.001f * clockBpm() / 60.0f;
    const float fl = floorf(p);
    p -= fl;
    ahead = (int)fl;
    count = (uint8_t)(count + ahead);
  }
  o.beatPhase = (uint16_t)clampf(p * 65536.0f, 0.0f, 65535.0f);
  o.beatCount = count;
  o.beatInBar = (uint8_t)((barPos_ + ahead) & 3);
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
