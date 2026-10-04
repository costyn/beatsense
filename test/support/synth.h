// SPDX-License-Identifier: EUPL-1.2
// Deterministic synthetic test audio (no music is committed): kicks, snares, hats and background noise at a known tempo.
#ifndef BEATSENSE_TEST_SYNTH_H
#define BEATSENSE_TEST_SYNTH_H

#include <math.h>
#include <stdint.h>
#include <vector>
#include <beatsense/analyzer.h>

namespace synth {

constexpr double kFs = 22050.0;

struct Rng { // xorshift32: same sequence on every platform
  uint32_t s;
  explicit Rng(uint32_t seed = 1) : s(seed ? seed : 1) {}
  uint32_t next() {
    s ^= s << 13;
    s ^= s >> 17;
    s ^= s << 5;
    return s;
  }
  double uniform() { return (next() >> 8) * (1.0 / 16777216.0) * 2.0 - 1.0; } // [-1, 1)
};

struct Spec {
  double bpm = 120;
  double seconds = 12;
  double startS = 0.2;        // first kick
  double kickAmp = 0.5;       // peak, fraction of full scale
  double hatAmp = 0.12;
  double snareAmp = 0.0;      // snare on beats 2 and 4 (0 = none)
  double noiseRms = 0.002;    // background noise (-54 dBFS)
  double gainDb = 0;          // applied to everything, noise included
  bool hats = true;           // off-beat hats at 8ths
  uint32_t seed = 1;
};

inline double beatTime(const Spec &s, int n) { return s.startS + n * 60.0 / s.bpm; }

// Mono float signal. Kick: 120 -> 55 Hz sweep with a 70 ms decay and a short click; hat: differentiated noise, 20 ms decay;
// snare: noise + 190 Hz body, 50 ms decay.
inline std::vector<float> render(const Spec &sp) {
  const size_t n = (size_t)(sp.seconds * kFs);
  std::vector<float> x(n, 0.0f);
  Rng rng(sp.seed);
  const double beat = 60.0 / sp.bpm;
  const int beats = (int)((sp.seconds - sp.startS) / beat) + 1;
  for (int b = 0; b < beats; b++) {
    const double t0 = sp.startS + b * beat;
    // kick
    double ph = 0;
    for (size_t i = (size_t)(t0 * kFs); i < n && i < (size_t)((t0 + 0.4) * kFs); i++) {
      const double t = i / kFs - t0;
      if (t < 0) continue;
      ph += 2.0 * M_PI * (55.0 + 65.0 * exp(-t / 0.03)) / kFs;
      x[i] += (float)(sp.kickAmp * (exp(-t / 0.07) * sin(ph) + 0.25 * exp(-t / 0.004) * rng.uniform()));
    }
    if (sp.hats) {
      const double th = t0 + beat * 0.5;
      double prev = 0;
      for (size_t i = (size_t)(th * kFs); i < n && i < (size_t)((th + 0.12) * kFs); i++) {
        const double t = i / kFs - th;
        if (t < 0) continue;
        const double w = rng.uniform();
        x[i] += (float)(sp.hatAmp * exp(-t / 0.02) * (w - prev)); // first difference = high-passed noise
        prev = w;
      }
    }
    if (sp.snareAmp > 0 && (b & 1)) {
      for (size_t i = (size_t)(t0 * kFs); i < n && i < (size_t)((t0 + 0.25) * kFs); i++) {
        const double t = i / kFs - t0;
        if (t < 0) continue;
        x[i] += (float)(sp.snareAmp * exp(-t / 0.05) * (0.7 * rng.uniform() + 0.5 * sin(2 * M_PI * 190 * t)));
      }
    }
  }
  for (size_t i = 0; i < n; i++) x[i] += (float)(sp.noiseRms * 1.7320508 * rng.uniform()); // uniform [-1,1) has rms 1/sqrt(3)
  const float g = (float)pow(10.0, sp.gainDb / 20.0);
  for (size_t i = 0; i < n; i++) x[i] *= g;
  return x;
}

// Float [-1, 1] to the INMP441's 32-bit slot: 24-bit data, left-justified
inline int32_t toSlot(float v) {
  if (v > 0.999999f) v = 0.999999f;
  if (v < -1.0f) v = -1.0f;
  return (int32_t)((int32_t)lrintf(v * 8388607.0f) * 256);
}

inline std::vector<int32_t> toSlots(const std::vector<float> &x) {
  std::vector<int32_t> s(x.size());
  for (size_t i = 0; i < x.size(); i++) s[i] = toSlot(x[i]);
  return s;
}

// Feed in hop-sized blocks; the callback runs after each hop with the time (s) of the last sample processed
template <typename F> inline void run(beatsense::Analyzer &a, const std::vector<float> &x, F onHop, size_t startSample = 0) {
  const size_t hop = beatsense::Config::kHop;
  std::vector<int32_t> slots = toSlots(x);
  for (size_t i = startSample; i + hop <= slots.size(); i += hop) {
    a.process(&slots[i], hop);
    onHop((double)(i + hop) / kFs, a);
  }
}

inline void feed(beatsense::Analyzer &a, const std::vector<float> &x) {
  run(a, x, [](double, beatsense::Analyzer &) {});
}

} // namespace synth

#endif
