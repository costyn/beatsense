// SPDX-License-Identifier: EUPL-1.2
// Run a WAV file through the Analyzer exactly as the firmware would and print one CSV row per hop (11.6 ms).
//
// Build (from the repo root):
//   g++ -std=c++14 -O2 -Iinclude -Ilib/beatsense/src -Itest/support tools/analyze_wav/main.cpp lib/beatsense/src/beatsense/analyzer.cpp -o analyze_wav
// Run:
//   ./analyze_wav song.wav > song.csv        (22050 or 44100 Hz, 16/24/32-bit PCM or float, mono or stereo)
//   ./analyze_wav song.wav --gain-db -20     (simulate a quieter room)
//   python3 tools/plot.py song.csv
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <beatsense/analyzer.h>
#include "wav.h"

int main(int argc, char **argv) {
  if (argc < 2) {
    fprintf(stderr, "usage: %s file.wav [--gain-db dB]\n", argv[0]);
    return 2;
  }
  float gainDb = 0;
  for (int i = 2; i + 1 < argc; i++) {
    if (!strcmp(argv[i], "--gain-db")) gainDb = (float)atof(argv[i + 1]);
  }
  wav::Audio audio;
  const std::string err = wav::read(argv[1], audio);
  if (!err.empty()) {
    fprintf(stderr, "%s: %s\n", argv[1], err.c_str());
    return 1;
  }
  const float g = powf(10.0f, gainDb / 20.0f);
  std::vector<int32_t> slots(audio.mono.size());
  for (size_t i = 0; i < slots.size(); i++) {
    float v = audio.mono[i] * g;
    v = v > 0.999999f ? 0.999999f : (v < -1.0f ? -1.0f : v);
    slots[i] = (int32_t)lrintf(v * 8388607.0f) * 256; // 24-bit data left-justified in a 32-bit slot, like the INMP441
  }

  beatsense::Analyzer a;
  const size_t hop = beatsense::Config::kHop;
  const double fs = a.config().sampleRate;
  printf("time_s,onset_low,onset_mid,onset_high,bpm,beat_phase,beat_count,beat_in_bar,confidence,level_low,level_mid,level_high,energy,"
         "status\n");
  for (size_t i = 0; i + hop <= slots.size(); i += hop) {
    a.process(&slots[i], hop);
    const beatsense::Features &f = a.features();
    printf("%.4f,%u,%u,%u,%.2f,%u,%u,%u,%u,%u,%u,%u,%u,%u\n", (double)(i + hop) / fs, f.onsets[0], f.onsets[1], f.onsets[2], f.bpm,
           f.beatPhase, f.beatCount, f.beatInBar, f.confidence, f.levels[0], f.levels[1], f.levels[2], f.energy, f.status);
  }
  return 0;
}
