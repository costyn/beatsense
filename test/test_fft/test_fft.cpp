// SPDX-License-Identifier: EUPL-1.2
#include <unity.h>
#include <math.h>
#include <beatsense/fft.h>
#include "../support/synth.h"

void setUp() {}
void tearDown() {}

using namespace beatsense;

static RealFft<512> fft;

static void naiveDft(const float *x, size_t n, double *re, double *im) {
  for (size_t k = 0; k <= n / 2; k++) {
    double sr = 0, si = 0;
    for (size_t i = 0; i < n; i++) {
      const double a = 2.0 * M_PI * (double)k * (double)i / (double)n;
      sr += x[i] * cos(a);
      si -= x[i] * sin(a);
    }
    re[k] = sr;
    im[k] = si;
  }
}

void test_fft_matches_naive_dft() {
  synth::Rng rng(7);
  float x[512];
  for (auto &v : x) v = (float)rng.uniform();
  float re[257], im[257];
  fft.forward(x, re, im);
  double rr[257], ri[257];
  naiveDft(x, 512, rr, ri);
  double maxErr = 0;
  for (int k = 0; k <= 256; k++) {
    maxErr = fmax(maxErr, fabs(re[k] - rr[k]));
    maxErr = fmax(maxErr, fabs(im[k] - ri[k]));
  }
  TEST_ASSERT_TRUE_MESSAGE(maxErr < 1e-3, "FFT differs from the naive DFT");
}

void test_fft_sine_lands_in_its_bin() {
  float x[512], re[257], im[257];
  for (int i = 0; i < 512; i++) x[i] = (float)sin(2.0 * M_PI * 40.0 * i / 512.0);
  fft.forward(x, re, im);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 256.0f, sqrtf(re[40] * re[40] + im[40] * im[40])); // A * N / 2
  TEST_ASSERT_TRUE(fabsf(re[41]) + fabsf(im[41]) < 0.01f);
}

void test_fft_impulse_is_flat() {
  float x[512] = {0}, re[257], im[257];
  x[0] = 1;
  fft.forward(x, re, im);
  for (int k = 0; k <= 256; k++) {
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, re[k]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, im[k]);
  }
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_fft_matches_naive_dft);
  RUN_TEST(test_fft_sine_lands_in_its_bin);
  RUN_TEST(test_fft_impulse_is_flat);
  return UNITY_END();
}
