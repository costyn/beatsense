// SPDX-License-Identifier: EUPL-1.2
#include <unity.h>
#include <math.h>
#include <stdio.h>
#include <beatsense/analyzer.h>
#include "../support/synth.h"

using namespace beatsense;

void setUp() {}
void tearDown() {}

// Same scenario as test_tempo's breakdown tests: 128 BPM groove, kick (and snare) gone from 30.2 s, kicks back on a beat at 41.7 s.
static synth::Spec breakSpec(double breakLenBeats = 32) {
  synth::Spec sp;
  sp.bpm = 128;
  sp.seconds = 56;
  sp.noiseRms = 0.005;
  sp.snareAmp = 0.25;
  sp.breakFromS = 0.2 + 64 * 60.0 / 128;
  sp.breakToS = 0.2 + (64 + breakLenBeats) * 60.0 / 128;
  return sp;
}

struct Groove {
  int minSteady = 255; // lowest value in the groove (after 10 s, before the break)
  int maxInBreak = 0;  // highest value from `settle` s into the break until the kicks return
  double lowAt = -1;   // s after the kick stopped until the value is <= 0.15 * 255 for good
  double riseAt = -1;  // s after the kicks return until the value reaches 0.85 * 255
};

static Groove run(const synth::Spec &sp, double settle) {
  Groove g;
  Analyzer a;
  bool pastLow = false;
  synth::run(a, synth::render(sp), [&](double t, Analyzer &an) {
    const int v = an.features().groove;
    if (t >= 10.0 && t < sp.breakFromS)
      g.minSteady = v < g.minSteady ? v : g.minSteady;
    if (t >= sp.breakFromS && t < sp.breakToS) {
      if (t >= sp.breakFromS + settle && v > g.maxInBreak)
        g.maxInBreak = v;
      if (v > 0.15 * 255)
        pastLow = false;
      else if (!pastLow) {
        pastLow = true;
        g.lowAt = t - sp.breakFromS;
      }
    }
    if (t >= sp.breakToS && g.riseAt < 0 && v >= 0.85 * 255)
      g.riseAt = t - sp.breakToS;
  });
  return g;
}

// A steady four-on-the-floor groove: after the tempo has been found the kick repeats every beat, so the groove is full (>= 0.85) and
// stays there
void test_groove_full_in_steady_groove() {
  synth::Spec sp;
  sp.bpm = 128;
  sp.seconds = 40;
  sp.noiseRms = 0.005;
  sp.snareAmp = 0.25;
  Analyzer a;
  int minV = 255;
  synth::run(a, synth::render(sp), [&](double t, Analyzer &an) {
    if (t >= 10.0)
      minV = an.features().groove < minV ? an.features().groove : minV;
  });
  printf("    lowest groove after 10 s: %d\n", minV);
  TEST_ASSERT_TRUE_MESSAGE(minV >= 0.85 * 255, "groove below 0.85 in a steady groove");
}

// A two-beat low-band pattern (kick on beats 1 and 3, nothing but a steady bass note on 2 and 4, as in a track whose low band repeats every two beats and
// not every beat): the one-beat correlation is ~0 there, so it only reads as groove through the two-beat lag. Full once the longer window has
// filled (about 12 s), and it stays full.
void test_groove_full_with_a_two_beat_pattern() {
  synth::Spec sp;
  sp.bpm = 120;
  sp.seconds = 50;
  sp.noiseRms = 0.005;
  sp.kickAmp = 0; // kicks are added below, on every other beat only
  sp.bassFromS = 0; // a steady bass note under the pattern, like the tracks have: it keeps the one-second average level
  sp.bassAmp = 0.12;
  sp.breakFromS = sp.seconds + 10;
  sp.breakToS = sp.seconds + 10; // (the drone runs until breakToS; the break itself is never reached)
  std::vector<float> x = synth::render(sp);
  const double beat = 60.0 / sp.bpm;
  for (int b = 0; sp.startS + b * beat < sp.seconds - 0.5; b++) {
    const double t0 = sp.startS + b * beat;
    double ph = 0;
    for (size_t i = (size_t)(t0 * synth::kFs); b % 2 == 0 && i < x.size() && i < (size_t)((t0 + 0.4) * synth::kFs); i++) {
      const double t = (double)i / synth::kFs - t0;
      ph += 2.0 * M_PI * (55.0 + 65.0 * exp(-t / 0.03)) / synth::kFs; // kick: 120 -> 55 Hz sweep
      x[i] += (float)(0.5 * exp(-t / 0.07) * sin(ph));
    }
  }
  Analyzer a;
  int minV = 255;
  synth::run(a, x, [&](double t, Analyzer &an) {
    if (t >= 15.0 && t < sp.seconds - 2.0) // the last kicks are before sp.seconds - 0.5
      minV = an.features().groove < minV ? an.features().groove : minV;
  });
  printf("    lowest groove after 15 s with a two-beat pattern: %d\n", minV);
  TEST_ASSERT_TRUE_MESSAGE(minV >= 0.85 * 255, "groove below 0.85 in a two-beat pattern");
}

// The kickless breakdown (hats and nothing else in the low band): the groove is gone within 1.5 s of the last kick and stays gone.
void test_groove_drops_in_kickless_breakdown() {
  const synth::Spec sp = breakSpec();
  const Groove g = run(sp, 1.5);
  printf("    groove high before the break (min %d), <= 0.15 after %.2f s, max in the break after 1.5 s: %d\n", g.minSteady, g.lowAt,
         g.maxInBreak);
  TEST_ASSERT_TRUE_MESSAGE(g.minSteady >= 0.85 * 255, "groove below 0.85 before the break");
  TEST_ASSERT_TRUE_MESSAGE(g.lowAt >= 0 && g.lowAt <= 1.5, "groove did not fall to 0.15 within 1.5 s of the kick stopping");
  TEST_ASSERT_TRUE_MESSAGE(g.maxInBreak <= 0.15 * 255, "groove above 0.15 during the breakdown");
}

// Second break of the live test: a bass note (steady 60 Hz sine at ~65% of the groove's low amplitude) comes back before the kick does.
// There is low-band energy but nothing repeating at the beat: the groove must stay low, and come back within a second of the kick.
void test_groove_stays_low_with_low_band_remnant() {
  synth::Spec sp = breakSpec(32);
  sp.bassFromS = sp.breakFromS + 6.0;
  sp.bassAmp = 0.042;
  const Groove g = run(sp, 1.5);
  printf("    max groove in the break after 1.5 s: %d (bass note from 6 s into it), back to 0.85 after %.2f s\n", g.maxInBreak, g.riseAt);
  TEST_ASSERT_TRUE_MESSAGE(g.maxInBreak <= 0.15 * 255, "groove above 0.15 while only a bass note plays");
  TEST_ASSERT_TRUE_MESSAGE(g.riseAt >= 0 && g.riseAt <= 1.0, "groove not back to 0.85 within 1 s of the kicks returning");
}

// A quiet rolling bassline (3 pulses per beat, no kick) after a groove: it repeats every beat, but also every third of a beat, which a kick
// does not. Low-band energy that is not the groove: it must stay low. Then the kicks return and the groove comes back.
void test_groove_stays_low_with_a_rolling_bass() {
  synth::Spec sp = breakSpec(32);
  std::vector<float> x = synth::render(sp);
  const double third = 60.0 / sp.bpm / 3.0;
  for (double t = sp.breakFromS + 4.0; t < sp.breakToS - 1.0; t += third) { // from 4 s into the break; 60 Hz pulses, 60 ms decay
    for (size_t i = (size_t)(t * synth::kFs); i < x.size() && i < (size_t)((t + 0.3) * synth::kFs); i++) {
      const double u = (double)i / synth::kFs - t;
      x[i] += (float)(0.15 * exp(-u / 0.06) * sin(2 * M_PI * 60.0 * u));
    }
  }
  Analyzer a;
  int maxV = 0;
  synth::run(a, x, [&](double t, Analyzer &an) {
    if (t >= sp.breakFromS + 7.0 && t < sp.breakToS - 1.0 && an.features().groove > maxV)
      maxV = an.features().groove;
  });
  printf("    max groove during the rolling bass: %d\n", maxV);
  TEST_ASSERT_TRUE_MESSAGE(maxV <= 0.3 * 255, "groove above 0.3 while only a rolling bassline plays");
}

// After the kick returns the groove is back to 0.85 within a second (one beat for the correlation to see a repeat, plus its time constant)
void test_groove_rises_within_a_second_of_the_kick() {
  const synth::Spec sp = breakSpec();
  const Groove g = run(sp, 1.5);
  printf("    groove >= 0.85 after %.2f s\n", g.riseAt);
  TEST_ASSERT_TRUE_MESSAGE(g.riseAt >= 0 && g.riseAt <= 1.0, "groove not back to 0.85 within 1 s of the kicks returning");
}

// A 1-bar drop of the kick in a groove (like the Satory drops): an honest dip, then full again
void test_groove_dips_on_a_one_bar_drop() {
  synth::Spec sp = breakSpec(4);
  sp.snareAmp = 0.25;
  const Groove g = run(sp, 0.8);
  printf("    1-bar drop: max groove from 0.8 s into the drop %d, back to 0.85 after %.2f s\n", g.maxInBreak, g.riseAt);
  TEST_ASSERT_TRUE_MESSAGE(g.maxInBreak <= 0.3 * 255, "groove did not dip during a one-bar drop");
  TEST_ASSERT_TRUE_MESSAGE(g.riseAt >= 0 && g.riseAt <= 1.2, "groove not back after the one-bar drop");
}

// No signal, no groove; and the level does not matter: the groove is full in a steady groove down to the signal gate. The floor of the
// correlation's denominator is relative to the low band's slow average (an absolute one zeroed the groove on the room mic of the lamp).
void test_groove_without_signal_and_level_independent() {
  Analyzer silent;
  synth::Spec quiet;
  quiet.seconds = 4;
  quiet.kickAmp = 0;
  quiet.hats = false;
  quiet.noiseRms = 0;
  synth::feed(silent, synth::render(quiet));
  TEST_ASSERT_EQUAL_UINT8(0, silent.features().groove);

  const double gains[] = {0.0, -25.0, -45.0, -55.0, -65.0};
  int vMin[5], withSignal[5];
  for (int k = 0; k < 5; k++) {
    synth::Spec sp;
    sp.bpm = 128;
    sp.seconds = 30;
    sp.snareAmp = 0.25;
    sp.noiseRms = 0.005;
    sp.gainDb = gains[k];
    Analyzer a;
    int m = 255, sig = 0;
    synth::run(a, synth::render(sp), [&](double t, Analyzer &an) {
      if (t >= 10.0) {
        m = an.features().groove < m ? an.features().groove : m;
        sig += (an.features().status & proto::kStatusSignal) ? 1 : 0;
      }
    });
    vMin[k] = m;
    withSignal[k] = sig;
  }
  printf("    lowest groove after 10 s: %d (0 dB) %d (-25) %d (-45) %d (-55) %d (-65, signal frames %d)\n", vMin[0], vMin[1], vMin[2], vMin[3],
         vMin[4], withSignal[4]);
  TEST_ASSERT_TRUE_MESSAGE(vMin[1] >= 0.85 * 255, "groove below 0.85 at -25 dB");
  TEST_ASSERT_TRUE_MESSAGE(vMin[2] >= 0.85 * 255, "groove below 0.85 at -45 dB");
  TEST_ASSERT_TRUE_MESSAGE(vMin[3] >= 0.85 * 255, "groove below 0.85 at -55 dB");
  // Only where the signal gate lets the music through (the synthetic groove is ~-25 dBFS RMS, the gate opens at -62)
  if (withSignal[4] > 0)
    TEST_ASSERT_TRUE_MESSAGE(vMin[4] >= 0.85 * 255, "groove below 0.85 at -65 dB");
}

// Noise and nothing else, just above the signal gate: white, and low-passed "room tone" (rumble, 6 dB/oct above ~150 Hz): no groove. The
// low band of noise fluctuates a lot, but not at the beat period, so the correlation stays low whatever the floor is.
void test_groove_stays_low_in_noise() {
  for (int k = 0; k < 2; k++) {
    synth::Spec sp;
    sp.seconds = 60;
    sp.kickAmp = 0;
    sp.hats = false;
    sp.noiseRms = 0.001; // -60 dBFS RMS, the gate opens at -62
    std::vector<float> x = synth::render(sp);
    if (k) { // one-pole low-pass at ~150 Hz, scaled back to the same RMS
      const float al = 1.0f - expf(-2.0f * 3.14159265f * 150.0f / 22050.0f);
      float y = 0;
      double e0 = 0, e1 = 0;
      for (float &v : x) {
        e0 += (double)v * v;
        y += al * (v - y);
        v = y;
        e1 += (double)v * v;
      }
      const float g = (float)sqrt(e0 / e1);
      for (float &v : x)
        v *= g;
    }
    Analyzer a;
    int n = 0, high = 0, maxV = 0, sig = 0;
    synth::run(a, x, [&](double t, Analyzer &an) {
      if (t < 5.0)
        return;
      const int v = an.features().groove;
      n++;
      sig += (an.features().status & proto::kStatusSignal) ? 1 : 0;
      high += v > 0.3 * 255 ? 1 : 0;
      maxV = v > maxV ? v : maxV;
    });
    printf("    %s noise: signal %d%% of %d frames, groove above 0.3 in %d frames (%.2f%%), max %d\n", k ? "low-passed" : "white", 100 * sig / n,
           n, high, 100.0 * high / n, maxV);
    TEST_ASSERT_TRUE_MESSAGE(sig > n / 2, "the noise does not pass the signal gate, the test would prove nothing");
    TEST_ASSERT_TRUE_MESSAGE(high <= n / 20, "groove above 0.3 in more than 5% of the frames of pure noise");
  }
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_groove_full_in_steady_groove);
  RUN_TEST(test_groove_full_with_a_two_beat_pattern);
  RUN_TEST(test_groove_drops_in_kickless_breakdown);
  RUN_TEST(test_groove_stays_low_with_low_band_remnant);
  RUN_TEST(test_groove_stays_low_with_a_rolling_bass);
  RUN_TEST(test_groove_rises_within_a_second_of_the_kick);
  RUN_TEST(test_groove_dips_on_a_one_bar_drop);
  RUN_TEST(test_groove_without_signal_and_level_independent);
  RUN_TEST(test_groove_stays_low_in_noise);
  return UNITY_END();
}
