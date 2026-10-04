// SPDX-License-Identifier: EUPL-1.2
#include <unity.h>
#include <math.h>
#include <stdio.h>
#include <vector>
#include <beatsense/analyzer.h>
#include "../support/synth.h"

using namespace beatsense;

void setUp() {}
void tearDown() {}

static std::vector<float> sine(double hz, double amp, double seconds, double dc = 0) {
  std::vector<float> x((size_t)(seconds * synth::kFs));
  for (size_t i = 0; i < x.size(); i++) x[i] = (float)(dc + amp * sin(2.0 * M_PI * hz * i / synth::kFs));
  return x;
}

static size_t argmaxBand(const Analyzer &a) {
  const float *l = a.debugBandLog();
  size_t best = 0;
  for (size_t b = 1; b < Config::kBands; b++) {
    if (l[b] > l[best]) best = b;
  }
  return best;
}

// 65 Hz = band 0 (bins 1-2), 300 Hz = band 2 (5-7), 1 kHz = band 4 (14-24), 3 kHz = band 6 (50-99), 8 kHz = band 7 (100-255)
void test_band_mapping() {
  const double hz[] = {65, 150, 300, 450, 800, 1500, 3000, 8000};
  const size_t expect[] = {0, 1, 2, 3, 4, 5, 6, 7};
  for (int i = 0; i < 8; i++) {
    Analyzer a;
    synth::feed(a, sine(hz[i], 0.3, 0.5));
    char msg[40];
    snprintf(msg, sizeof msg, "band for %.0f Hz", hz[i]);
    TEST_ASSERT_EQUAL_UINT_MESSAGE(expect[i], argmaxBand(a), msg);
  }
}

void test_groups_levels_follow_frequency() {
  const double hz[] = {80, 700, 6000};
  for (int g = 0; g < 3; g++) {
    Analyzer a;
    synth::feed(a, sine(hz[g], 0.3, 2.0));
    const Features &f = a.features();
    for (int k = 0; k < 3; k++) {
      if (k == g) TEST_ASSERT_TRUE(f.levels[k] > 200);
      else TEST_ASSERT_TRUE_MESSAGE(f.levels[k] < 60, "other groups should stay low (Hann sidelobes only)");
    }
  }
}

// The DC blocker: a large DC offset must not change what the analysis sees, and DC alone is "no signal"
void test_dc_blocker() {
  Analyzer plain, offset, dcOnly;
  synth::feed(plain, sine(700, 0.1, 3.0));
  synth::feed(offset, sine(700, 0.1, 3.0, 0.4));
  for (int g = 0; g < 3; g++) TEST_ASSERT_INT_WITHIN(3, plain.features().levels[g], offset.features().levels[g]);
  synth::feed(dcOnly, std::vector<float>((size_t)(3 * synth::kFs), 0.5f));
  TEST_ASSERT_EQUAL_UINT8(0, dcOnly.features().status & proto::kStatusSignal);
}

static double meanLevel(const synth::Spec &sp, int group, double fromS, uint8_t agcMode = proto::kAgcNormal, uint8_t *peak = nullptr) {
  Analyzer a;
  a.setAgcMode(agcMode);
  double sum = 0;
  int n = 0;
  uint8_t pk = 0;
  synth::run(a, synth::render(sp), [&](double t, Analyzer &an) {
    if (t >= fromS) {
      sum += an.features().levels[group];
      n++;
      if (an.features().levels[group] > pk) pk = an.features().levels[group];
    }
  });
  if (peak) *peak = pk;
  return sum / n;
}

// Same music at -30 dB and -6 dB input ends up in the same level range
void test_agc_converges_across_input_gain() {
  // Low group at -30 dB vs -6 dB; mid/high carry less energy, so they get a smaller gap to stay above the AGC's -54 dBFS gain clamp
  for (int g = 0; g < 3; g++) {
    synth::Spec quiet, loud;
    quiet.seconds = loud.seconds = 25;
    quiet.gainDb = g == 0 ? -30 : -18;
    loud.gainDb = -6;
    uint8_t pkQ, pkL;
    const double q = meanLevel(quiet, g, 15, proto::kAgcNormal, &pkQ);
    const double l = meanLevel(loud, g, 15, proto::kAgcNormal, &pkL);
    TEST_ASSERT_TRUE_MESSAGE(q > 15 && l > 15, "levels should not be near zero");
    TEST_ASSERT_TRUE_MESSAGE(fabs(q - l) < 0.12 * fmax(q, l), "mean level differs between -30 dB and -6 dB input");
    TEST_ASSERT_TRUE(pkQ > 200 && pkL > 200);
  }
}

void test_agc_off_is_not_normalised() {
  synth::Spec quiet, loud;
  quiet.seconds = loud.seconds = 15;
  quiet.gainDb = -30;
  loud.gainDb = -6;
  const double q = meanLevel(quiet, 0, 8, proto::kAgcOff);
  const double l = meanLevel(loud, 0, 8, proto::kAgcOff);
  TEST_ASSERT_TRUE(l > 4 * fmax(q, 1.0));
}

void test_noise_gate_in_silence() {
  synth::Spec sp;
  sp.kickAmp = 0;
  sp.hatAmp = 0;
  sp.noiseRms = 6e-5; // -84 dBFS: roughly the INMP441 noise floor
  sp.seconds = 6;
  Analyzer a;
  synth::feed(a, synth::render(sp));
  const Features &f = a.features();
  TEST_ASSERT_EQUAL_UINT8(0, f.status & proto::kStatusSignal);
  for (int g = 0; g < 3; g++) {
    TEST_ASSERT_EQUAL_UINT8(0, f.levels[g]);
    TEST_ASSERT_EQUAL_UINT8(0, f.onsets[g]);
  }
  TEST_ASSERT_EQUAL_UINT8(0, f.confidence);
}

void test_gate_threshold_is_adjustable() {
  Analyzer a;
  a.setGateDb(-20.0f);
  synth::feed(a, sine(700, 0.05, 2.0)); // -29 dBFS peak
  TEST_ASSERT_EQUAL_UINT8(0, a.features().status & proto::kStatusSignal);
  Analyzer b;
  synth::feed(b, sine(700, 0.05, 2.0));
  TEST_ASSERT_EQUAL_UINT8(proto::kStatusSignal, b.features().status & proto::kStatusSignal);
}

// Kicks produce low-band onsets; a steady tone doesn't (after its own first attack)
void test_onsets_fire_on_kicks_not_on_steady_tone() {
  synth::Spec sp;
  sp.hats = false;
  sp.seconds = 8;
  Analyzer a;
  const double beat = 0.5;
  std::vector<double> peakAfterKick(16, 0);
  double idleMax = 0;
  synth::run(a, synth::render(sp), [&](double t, Analyzer &an) {
    if (t < 2.0) return;
    const double rel = t - sp.startS;
    const int n = (int)floor(rel / beat);
    const double since = rel - n * beat;
    if (since < 0.04) peakAfterKick[n] = fmax(peakAfterKick[n], an.features().onsets[0]); // within 40 ms of the kick
    if (since > 0.3) idleMax = fmax(idleMax, an.features().onsets[0]);
  });
  int kicks = 0, seen = 0;
  for (int n = 4; n < 15; n++) {
    kicks++;
    if (peakAfterKick[n] > 150) seen++;
  }
  TEST_ASSERT_EQUAL_INT_MESSAGE(kicks, seen, "kick missed by the low onset");
  TEST_ASSERT_TRUE_MESSAGE(idleMax < 40, "low onset fires between kicks");

  Analyzer tone;
  double toneMax = 0;
  synth::run(tone, sine(440, 0.2, 8.0), [&](double t, Analyzer &an) {
    if (t > 2.0) {
      for (int g = 0; g < 3; g++) toneMax = fmax(toneMax, an.features().onsets[g]);
    }
  });
  TEST_ASSERT_TRUE_MESSAGE(toneMax < 10, "steady tone produced onsets");
}

void test_clipping_flag() {
  synth::Spec loud;
  loud.kickAmp = 1.6; // clipped by the slot conversion
  loud.seconds = 3;
  Analyzer a;
  bool seen = false;
  synth::run(a, synth::render(loud), [&](double, Analyzer &an) { seen |= (an.features().status & proto::kStatusClipping) != 0; });
  TEST_ASSERT_TRUE(seen);
  Analyzer b;
  synth::Spec normal;
  normal.seconds = 3;
  bool bad = false;
  synth::run(b, synth::render(normal), [&](double, Analyzer &an) { bad |= (an.features().status & proto::kStatusClipping) != 0; });
  TEST_ASSERT_FALSE(bad);
}

// Feeding in odd-sized chunks must give exactly the same result as hop-sized blocks
void test_chunking_does_not_matter() {
  synth::Spec sp;
  sp.seconds = 8;
  const auto slots = synth::toSlots(synth::render(sp));
  Analyzer a, b;
  for (size_t i = 0; i + 256 <= slots.size(); i += 256) a.process(&slots[i], 256);
  size_t i = 0, chunk = 1;
  const size_t usable = slots.size() / 256 * 256;
  while (i < usable) {
    const size_t n = chunk < usable - i ? chunk : usable - i;
    b.process(&slots[i], n);
    i += n;
    chunk = chunk % 333 + 7;
  }
  TEST_ASSERT_EQUAL_UINT32(a.frameCount(), b.frameCount());
  TEST_ASSERT_EQUAL_FLOAT(a.features().bpm, b.features().bpm);
  TEST_ASSERT_EQUAL_UINT16(a.features().beatPhase, b.features().beatPhase);
  TEST_ASSERT_EQUAL_UINT8(a.features().levels[0], b.features().levels[0]);
}

void test_energy_tracks_loudness_changes() {
  // 40 s at one level, then 25 s at -12 dB: energy drops below average; steady music sits near 128
  synth::Spec a1;
  a1.seconds = 40;
  synth::Spec a2;
  a2.seconds = 25;
  a2.gainDb = -12;
  auto x = synth::render(a1);
  auto y = synth::render(a2);
  x.insert(x.end(), y.begin(), y.end());
  Analyzer an;
  int atSteady = 0, atEnd = 0;
  synth::run(an, x, [&](double t, Analyzer &a) {
    if (t > 39.9 && t < 40.0) atSteady = a.features().energy;
    atEnd = a.features().energy;
  });
  TEST_ASSERT_INT_WITHIN(15, 128, atSteady);
  TEST_ASSERT_TRUE_MESSAGE(atEnd < atSteady - 25, "energy should fall after the music gets quieter");
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_band_mapping);
  RUN_TEST(test_groups_levels_follow_frequency);
  RUN_TEST(test_dc_blocker);
  RUN_TEST(test_agc_converges_across_input_gain);
  RUN_TEST(test_agc_off_is_not_normalised);
  RUN_TEST(test_noise_gate_in_silence);
  RUN_TEST(test_gate_threshold_is_adjustable);
  RUN_TEST(test_onsets_fire_on_kicks_not_on_steady_tone);
  RUN_TEST(test_clipping_flag);
  RUN_TEST(test_chunking_does_not_matter);
  RUN_TEST(test_energy_tracks_loudness_changes);
  return UNITY_END();
}
