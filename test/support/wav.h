// SPDX-License-Identifier: EUPL-1.2
// Minimal WAV reader/writer for tests and tools/analyze_wav (host only; uses the heap).
//
// Reads PCM 16/24/32-bit and 32-bit float, mono or stereo (stereo is averaged to mono), at 22050 Hz (used as is) or 44100 Hz
// (low-pass filtered and decimated by 2). Other rates are rejected: resample first, e.g. `ffmpeg -i in.mp3 -ac 1 -ar 22050 out.wav`.
// Output is the Analyzer's input format: 32-bit slots with 24-bit data left-justified.
#ifndef BEATSENSE_TEST_WAV_H
#define BEATSENSE_TEST_WAV_H

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <string>
#include <vector>

namespace wav {

struct Audio {
  std::vector<float> mono; // at 22050 Hz, [-1, 1]
  uint32_t sourceRate = 0;
  uint16_t channels = 0;
};

inline uint32_t rd32(const uint8_t *p) { return p[0] | (p[1] << 8) | (p[2] << 16) | ((uint32_t)p[3] << 24); }
inline uint16_t rd16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }

// 2:1 decimation with a 31-tap Hamming-windowed sinc low-pass at 0.45 * 22050 Hz
inline std::vector<float> decimateBy2(const std::vector<float> &x) {
  const int taps = 31, mid = taps / 2;
  float h[taps];
  double sum = 0;
  for (int i = 0; i < taps; i++) {
    const double t = i - mid;
    const double fc = 0.225; // cutoff in cycles/sample at the input rate
    const double s = t == 0 ? 2 * fc : sin(2 * M_PI * fc * t) / (M_PI * t);
    h[i] = (float)(s * (0.54 - 0.46 * cos(2 * M_PI * i / (taps - 1))));
    sum += h[i];
  }
  for (auto &v : h) v = (float)(v / sum);
  std::vector<float> y(x.size() / 2);
  for (size_t o = 0; o < y.size(); o++) {
    double acc = 0;
    for (int i = 0; i < taps; i++) {
      const long idx = (long)(2 * o) + i - mid;
      if (idx >= 0 && idx < (long)x.size()) acc += h[i] * x[idx];
    }
    y[o] = (float)acc;
  }
  return y;
}

// Returns an empty error string on success
inline std::string read(const char *path, Audio &out) {
  FILE *fp = fopen(path, "rb");
  if (!fp) return std::string("cannot open ") + path;
  std::vector<uint8_t> d;
  uint8_t buf[65536];
  size_t n;
  while ((n = fread(buf, 1, sizeof buf, fp)) > 0) d.insert(d.end(), buf, buf + n);
  fclose(fp);
  if (d.size() < 12 || memcmp(d.data(), "RIFF", 4) || memcmp(d.data() + 8, "WAVE", 4)) return "not a RIFF/WAVE file";
  uint16_t fmt = 0, ch = 0, bits = 0;
  uint32_t rate = 0;
  const uint8_t *data = nullptr;
  size_t dataLen = 0;
  for (size_t p = 12; p + 8 <= d.size();) {
    const uint32_t len = rd32(&d[p + 4]);
    const uint8_t *body = &d[p + 8];
    if (!memcmp(&d[p], "fmt ", 4) && len >= 16) {
      fmt = rd16(body);
      ch = rd16(body + 2);
      rate = rd32(body + 4);
      bits = rd16(body + 14);
      if (fmt == 0xFFFE && len >= 26) fmt = rd16(body + 24); // WAVE_FORMAT_EXTENSIBLE: sub-format code
    } else if (!memcmp(&d[p], "data", 4)) {
      data = body;
      dataLen = p + 8 + len > d.size() ? d.size() - p - 8 : len;
      break;
    }
    p += 8 + len + (len & 1);
  }
  if (!data || !ch) return "missing fmt/data chunk";
  if (ch > 2) return "only mono or stereo is supported";
  if (rate != 22050 && rate != 44100) return "sample rate must be 22050 or 44100 Hz (resample with ffmpeg -ar 22050)";
  const bool isFloat = fmt == 3 && bits == 32;
  if (!(fmt == 1 && (bits == 16 || bits == 24 || bits == 32)) && !isFloat) return "unsupported sample format";
  const size_t bytes = bits / 8;
  const size_t frames = dataLen / (bytes * ch);
  std::vector<float> mono(frames);
  for (size_t i = 0; i < frames; i++) {
    double acc = 0;
    for (int c = 0; c < ch; c++) {
      const uint8_t *s = data + (i * ch + c) * bytes;
      double v;
      if (isFloat) {
        float f;
        memcpy(&f, s, 4);
        v = f;
      } else if (bits == 16) {
        v = (int16_t)rd16(s) / 32768.0;
      } else if (bits == 24) {
        int32_t x = (int32_t)((uint32_t)s[0] << 8 | (uint32_t)s[1] << 16 | (uint32_t)s[2] << 24) >> 8;
        v = x / 8388608.0;
      } else {
        v = (int32_t)rd32(s) / 2147483648.0;
      }
      acc += v;
    }
    mono[i] = (float)(acc / ch);
  }
  out.sourceRate = rate;
  out.channels = ch;
  out.mono = rate == 44100 ? decimateBy2(mono) : mono;
  return "";
}

// 16-bit mono PCM, for tiny generated fixtures
inline bool write16(const char *path, const std::vector<float> &x, uint32_t rate, uint16_t channels = 1) {
  FILE *fp = fopen(path, "wb");
  if (!fp) return false;
  auto w32 = [&](uint32_t v) { fwrite(&v, 4, 1, fp); };
  auto w16 = [&](uint16_t v) { fwrite(&v, 2, 1, fp); };
  const uint32_t dataBytes = (uint32_t)(x.size() * 2 * channels);
  fwrite("RIFF", 1, 4, fp);
  w32(36 + dataBytes);
  fwrite("WAVEfmt ", 1, 8, fp);
  w32(16);
  w16(1);
  w16(channels);
  w32(rate);
  w32(rate * 2 * channels);
  w16((uint16_t)(2 * channels));
  w16(16);
  fwrite("data", 1, 4, fp);
  w32(dataBytes);
  for (float v : x) {
    const int16_t s = (int16_t)lrintf(fmaxf(-1.0f, fminf(0.99997f, v)) * 32768.0f);
    for (int c = 0; c < channels; c++) w16((uint16_t)s);
  }
  fclose(fp);
  return true;
}

} // namespace wav

#endif
