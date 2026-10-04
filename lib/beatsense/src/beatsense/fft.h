// SPDX-License-Identifier: EUPL-1.2
//
// Small radix-2 real FFT. Written from scratch (no esp-dsp / arduinoFFT dependency; arduinoFFT is GPL).
//
// Seam for a faster backend: the Analyzer only uses the interface below (constructor, forward()). To switch to esp-dsp on the S3,
// provide a class with the same two members (e.g. wrapping dsps_fft2r_fc32 + dsps_bit_rev2r_fc32 + dsps_cplx2reC_fc32) and select it
// with the BEATSENSE_FFT_CLASS macro in analyzer.h. Nothing else changes.
#ifndef BEATSENSE_FFT_H
#define BEATSENSE_FFT_H

#include <stddef.h>
#include <stdint.h>
#include <math.h>

namespace beatsense {

// Real FFT of N points via one complex FFT of N/2 points plus a split step (half the work of a complex FFT with zero imaginary part).
// All tables and scratch are members: no heap, nothing after construction.
template <size_t N> class RealFft {
  static_assert(N >= 8 && (N & (N - 1)) == 0, "N must be a power of two >= 8");

public:
  static constexpr size_t kSize = N;
  static constexpr size_t kBins = N / 2 + 1;

  RealFft() {
    for (size_t k = 0; k <= H; k++) {
      const double a = 2.0 * M_PI * (double)k / (double)N;
      cos_[k] = (float)cos(a);
      sin_[k] = (float)sin(a);
    }
    size_t bits = 0;
    while ((size_t(1) << bits) < H) {
      bits++;
    }
    for (size_t i = 0; i < H; i++) {
      size_t r = 0;
      for (size_t b = 0; b < bits; b++) {
        if (i & (size_t(1) << b)) {
          r |= size_t(1) << (bits - 1 - b);
        }
      }
      rev_[i] = (uint16_t)r;
    }
  }

  // in: N real samples. re/im: N/2+1 bins each (bin k = k * fs / N). Unnormalised (a full-scale sine of amplitude A gives |X| = A*N/2
  // without a window).
  void forward(const float *in, float *re, float *im) {
    for (size_t n = 0; n < H; n++) {
      zr_[rev_[n]] = in[2 * n];
      zi_[rev_[n]] = in[2 * n + 1];
    }
    for (size_t len = 2; len <= H; len <<= 1) {
      const size_t half = len >> 1;
      const size_t step = N / len; // twiddle exp(-2 pi i j / len) = table index j * N / len
      for (size_t i = 0; i < H; i += len) {
        for (size_t j = 0; j < half; j++) {
          const float wr = cos_[j * step];
          const float wi = -sin_[j * step];
          const size_t a = i + j;
          const size_t b = a + half;
          const float tr = zr_[b] * wr - zi_[b] * wi;
          const float ti = zr_[b] * wi + zi_[b] * wr;
          zr_[b] = zr_[a] - tr;
          zi_[b] = zi_[a] - ti;
          zr_[a] += tr;
          zi_[a] += ti;
        }
      }
    }
    // Split: Z = FFT(even + i*odd). E_k = (Z_k + conj(Z_{H-k}))/2, O_k = (Z_k - conj(Z_{H-k}))/(2i), X_k = E_k + exp(-2 pi i k/N) O_k
    for (size_t k = 0; k <= H; k++) {
      const size_t k1 = k & (H - 1);
      const size_t k2 = (H - k) & (H - 1);
      const float ar = zr_[k1], ai = zi_[k1];
      const float br = zr_[k2], bi = -zi_[k2]; // conj(Z_{H-k})
      const float er = 0.5f * (ar + br), ei = 0.5f * (ai + bi);
      const float dr = ar - br, di = ai - bi;
      const float orr = 0.5f * di, oi = -0.5f * dr; // D / (2i)
      const float wr = cos_[k], wi = -sin_[k];
      re[k] = er + orr * wr - oi * wi;
      im[k] = ei + orr * wi + oi * wr;
    }
  }

private:
  static constexpr size_t H = N / 2;
  float cos_[H + 1];
  float sin_[H + 1];
  uint16_t rev_[H];
  float zr_[H];
  float zi_[H];
};

} // namespace beatsense

#endif // BEATSENSE_FFT_H
