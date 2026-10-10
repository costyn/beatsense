// SPDX-License-Identifier: EUPL-1.2
#include <unity.h>
#include <math.h>
#include <stdio.h>
#include "../support/wav.h"
#include "../support/synth.h"

void setUp() {}
void tearDown() {}

static const char *kPath = "test_wav_tmp.wav";

void test_round_trip_22050_mono() {
  std::vector<float> x(2000);
  for (size_t i = 0; i < x.size(); i++) x[i] = 0.5f * sinf(0.05f * i);
  TEST_ASSERT_TRUE(wav::write16(kPath, x, 22050));
  wav::Audio a;
  TEST_ASSERT_EQUAL_STRING("", wav::read(kPath, a).c_str());
  TEST_ASSERT_EQUAL_UINT(x.size(), a.mono.size());
  for (size_t i = 0; i < x.size(); i++) TEST_ASSERT_FLOAT_WITHIN(1e-4f, x[i], a.mono[i]);
  remove(kPath);
}

void test_stereo_is_averaged() {
  std::vector<float> x(500, 0.25f);
  TEST_ASSERT_TRUE(wav::write16(kPath, x, 22050, 2));
  wav::Audio a;
  TEST_ASSERT_EQUAL_STRING("", wav::read(kPath, a).c_str());
  TEST_ASSERT_EQUAL_UINT16(2, a.channels);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.25f, a.mono[100]);
  remove(kPath);
}

// 44.1 kHz input is decimated to 22.05 kHz: a 1 kHz tone keeps its frequency and level, a 15 kHz tone (above the new Nyquist) is removed
void test_44100_decimation() {
  std::vector<float> lo(44100), hi(44100);
  for (size_t i = 0; i < lo.size(); i++) {
    lo[i] = 0.5f * sinf(2 * M_PI * 1000.0 * i / 44100.0);
    hi[i] = 0.5f * sinf(2 * M_PI * 15000.0 * i / 44100.0);
  }
  wav::Audio a;
  TEST_ASSERT_TRUE(wav::write16(kPath, lo, 44100));
  TEST_ASSERT_EQUAL_STRING("", wav::read(kPath, a).c_str());
  TEST_ASSERT_UINT_WITHIN(2, 22050, a.mono.size());
  double peak = 0;
  for (size_t i = 1000; i < 2000; i++) peak = fmax(peak, fabs(a.mono[i]));
  TEST_ASSERT_FLOAT_WITHIN(0.03f, 0.5f, (float)peak);
  TEST_ASSERT_TRUE(wav::write16(kPath, hi, 44100));
  TEST_ASSERT_EQUAL_STRING("", wav::read(kPath, a).c_str());
  peak = 0;
  for (size_t i = 1000; i < 2000; i++) peak = fmax(peak, fabs(a.mono[i]));
  TEST_ASSERT_TRUE_MESSAGE(peak < 0.02, "alias not suppressed");
  remove(kPath);
}

void test_rejects_unsupported_rate_and_garbage() {
  TEST_ASSERT_TRUE(wav::write16(kPath, std::vector<float>(100, 0.f), 48000));
  wav::Audio a;
  TEST_ASSERT_TRUE(wav::read(kPath, a).size() > 0);
  FILE *f = fopen(kPath, "wb");
  fputs("this is not a wav file at all", f);
  fclose(f);
  TEST_ASSERT_TRUE(wav::read(kPath, a).size() > 0);
  remove(kPath);
  TEST_ASSERT_TRUE(wav::read("/nonexistent.wav", a).size() > 0);
}

// The full path a real song takes: WAV file -> slots -> Analyzer
void test_wav_file_through_analyzer() {
  synth::Spec sp;
  sp.bpm = 128;
  sp.seconds = 14;
  TEST_ASSERT_TRUE(wav::write16(kPath, synth::render(sp), 22050));
  wav::Audio a;
  TEST_ASSERT_EQUAL_STRING("", wav::read(kPath, a).c_str());
  beatsense::Analyzer an;
  synth::feed(an, a.mono);
  TEST_ASSERT_FLOAT_WITHIN(1.0f, 128.0f, an.features().bpm);
  remove(kPath);
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_round_trip_22050_mono);
  RUN_TEST(test_stereo_is_averaged);
  RUN_TEST(test_44100_decimation);
  RUN_TEST(test_rejects_unsupported_rate_and_garbage);
  RUN_TEST(test_wav_file_through_analyzer);
  return UNITY_END();
}
