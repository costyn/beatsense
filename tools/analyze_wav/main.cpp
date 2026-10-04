// SPDX-License-Identifier: EUPL-1.2
// Run a WAV file through the Analyzer exactly as the firmware would and print one CSV row per hop (11.6 ms).
//
// Build (from the repo root):
//   g++ -std=c++14 -O2 -Iinclude -Ilib/beatsense/src -Itest/support tools/analyze_wav/main.cpp lib/beatsense/src/beatsense/analyzer.cpp -o analyze_wav
// Run:
//   ./analyze_wav song.wav > song.csv        (22050 or 44100 Hz, 16/24/32-bit PCM or float, mono or stereo)
//   ./analyze_wav song.wav --gain-db -20     (simulate a quieter room)
//   ./analyze_wav song.wav --summary         (short text report instead of the CSV)
//   ./analyze_wav song.wav --clicks out.wav  (the song with a click on every predicted beat, to check sync by ear)
//   ./analyze_wav song.wav --summary --csv song.csv   (both)
//   python3 tools/plot.py song.csv
#include <algorithm>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <vector>
#include <beatsense/analyzer.h>
#include "wav.h"

namespace {

struct Row {
  double t;
  beatsense::Features f;
};

// Percentile (0-100) of a sample set, nearest rank
double pct(std::vector<double> v, double p) {
  if (v.empty()) return 0;
  std::sort(v.begin(), v.end());
  size_t i = (size_t)(p / 100.0 * (double)(v.size() - 1) + 0.5);
  return v[i];
}

struct Segment {
  double start, end;
  std::vector<double> bpm;
  double median() const { return pct(bpm, 50); }
};

// Tempo segments: runs of frames within 3% of the segment's running mean; runs shorter than 2 s are absorbed (wobbles, and the
// transitions of a tempo change), then neighbours with a similar median are merged. A real excursion of a few seconds stays visible.
std::vector<Segment> tempoSegments(const std::vector<Row> &rows) {
  std::vector<Segment> segs;
  double mean = 0;
  for (const Row &r : rows) {
    if (r.f.bpm <= 0) continue;
    if (segs.empty() || fabs(r.f.bpm - mean) > 0.03 * mean) {
      segs.push_back(Segment{r.t, r.t, {}});
      mean = r.f.bpm;
    }
    Segment &s = segs.back();
    s.end = r.t;
    s.bpm.push_back(r.f.bpm);
    mean += (r.f.bpm - mean) / (double)s.bpm.size();
  }
  std::vector<Segment> kept;
  for (const Segment &s : segs) {
    if (s.end - s.start >= 2.0) kept.push_back(s);
  }
  std::vector<Segment> out;
  for (const Segment &s : kept) {
    if (!out.empty() && fabs(s.median() - out.back().median()) < 0.03 * out.back().median()) {
      out.back().end = s.end;
      out.back().bpm.insert(out.back().bpm.end(), s.bpm.begin(), s.bpm.end());
    } else {
      out.push_back(s);
    }
  }
  return out;
}

void summary(const std::vector<Row> &rows) {
  using namespace beatsense;
  if (rows.empty()) {
    printf("no audio\n");
    return;
  }
  const double dur = rows.back().t;
  double firstLock = -1;
  size_t locked = 0, withSignal = 0;
  for (const Row &r : rows) {
    if (firstLock < 0 && (r.f.status & proto::kStatusLocked)) firstLock = r.t;
    if (r.f.status & proto::kStatusLocked) locked++;
    if (r.f.status & proto::kStatusSignal) withSignal++;
  }
  printf("duration %.1f s, %zu frames, signal present %.0f%% of the time\n", dur, rows.size(), 100.0 * withSignal / rows.size());
  if (firstLock >= 0) printf("first lock at %.1f s\n", firstLock);
  else printf("never locked\n");
  printf("locked %.0f%% of the time\n", 100.0 * locked / rows.size());
  printf("tempo segments (start-end s, median BPM):\n");
  const std::vector<Segment> segs = tempoSegments(rows);
  if (segs.empty()) printf("  none\n");
  for (const Segment &s : segs) printf("  %6.1f - %6.1f  %6.2f\n", s.start, s.end, s.median());
  std::vector<double> conf;
  for (const Row &r : rows) {
    if (firstLock >= 0 && r.t >= firstLock) conf.push_back(r.f.confidence);
  }
  if (!conf.empty()) printf("confidence after first lock (0-255): min %.0f, median %.0f\n", pct(conf, 0), pct(conf, 50));

  // Output ranges over the frames with signal present (silence is all zeros and would hide the real distribution)
  printf("u8 outputs over frames with signal: %% at 255, p50, p95\n");
  const char *names[] = {"onset_low", "onset_mid", "onset_high", "level_low", "level_mid", "level_high"};
  for (int k = 0; k < 6; k++) {
    std::vector<double> v;
    size_t at255 = 0;
    for (const Row &r : rows) {
      if (!(r.f.status & proto::kStatusSignal)) continue;
      const uint8_t x = k < 3 ? r.f.onsets[k] : r.f.levels[k - 3];
      v.push_back(x);
      if (x == 255) at255++;
    }
    printf("  %-10s %5.1f%%  p50 %3.0f  p95 %3.0f\n", names[k], v.empty() ? 0.0 : 100.0 * at255 / v.size(), pct(v, 50), pct(v, 95));
  }
  std::vector<double> en;
  for (const Row &r : rows) {
    if (r.f.status & proto::kStatusSignal) en.push_back(r.f.energy);
  }
  printf("energy: min %.0f, median %.0f, max %.0f\n", pct(en, 0), pct(en, 50), pct(en, 100));
}

// Click track: a short decaying sine on every predicted beat, louder with a higher pitch on beatInBar 0
void addClick(std::vector<float> &x, double fs, double timeS, bool accent) {
  const double hz = accent ? 1760.0 : 1100.0, amp = accent ? 0.7 : 0.35;
  const long start = (long)(timeS * fs + 0.5);
  const long len = (long)(0.03 * fs);
  for (long i = 0; i < len; i++) {
    const long idx = start + i;
    if (idx < 0 || idx >= (long)x.size()) continue;
    const double t = (double)i / fs;
    x[idx] += (float)(amp * exp(-t / 0.008) * sin(2.0 * M_PI * hz * t));
  }
}

} // namespace

int main(int argc, char **argv) {
  const char *in = nullptr, *clicksPath = nullptr, *csvPath = nullptr;
  float gainDb = 0;
  bool wantSummary = false;
  for (int i = 1; i < argc; i++) {
    if (!strcmp(argv[i], "--gain-db") && i + 1 < argc) gainDb = (float)atof(argv[++i]);
    else if (!strcmp(argv[i], "--clicks") && i + 1 < argc) clicksPath = argv[++i];
    else if (!strcmp(argv[i], "--csv") && i + 1 < argc) csvPath = argv[++i];
    else if (!strcmp(argv[i], "--summary")) wantSummary = true;
    else if (argv[i][0] != '-' && !in) in = argv[i];
    else {
      fprintf(stderr, "unknown argument %s\n", argv[i]);
      in = nullptr;
      break;
    }
  }
  if (!in) {
    fprintf(stderr, "usage: %s file.wav [--gain-db dB] [--summary] [--csv out.csv|-] [--clicks out.wav]\n", argv[0]);
    return 2;
  }
  wav::Audio audio;
  const std::string err = wav::read(in, audio);
  if (!err.empty()) {
    fprintf(stderr, "%s: %s\n", in, err.c_str());
    return 1;
  }
  const float g = powf(10.0f, gainDb / 20.0f);
  std::vector<int32_t> slots(audio.mono.size());
  for (size_t i = 0; i < slots.size(); i++) {
    float v = audio.mono[i] * g;
    v = v > 0.999999f ? 0.999999f : (v < -1.0f ? -1.0f : v);
    slots[i] = (int32_t)lrintf(v * 8388607.0f) * 256; // 24-bit data left-justified in a 32-bit slot, like the INMP441
  }

  // CSV goes to stdout unless a summary was asked for (then only if --csv says where)
  FILE *csv = nullptr;
  if (csvPath) csv = !strcmp(csvPath, "-") ? stdout : fopen(csvPath, "w");
  else if (!wantSummary) csv = stdout;
  if (csvPath && !csv) {
    fprintf(stderr, "cannot write %s\n", csvPath);
    return 1;
  }

  beatsense::Analyzer a;
  const size_t hop = beatsense::Config::kHop;
  const double fs = a.config().sampleRate;
  std::vector<float> clicks = audio.original;
  std::vector<Row> rows;
  uint8_t prevCount = 0;
  bool havePrev = false;
  if (csv) {
    fprintf(csv, "time_s,onset_low,onset_mid,onset_high,bpm,beat_phase,beat_count,beat_in_bar,confidence,level_low,level_mid,level_high,"
                 "energy,status\n");
  }
  for (size_t i = 0; i + hop <= slots.size(); i += hop) {
    a.process(&slots[i], hop);
    const beatsense::Features &f = a.features();
    const double t = (double)(i + hop) / fs;
    if (csv) {
      fprintf(csv, "%.4f,%u,%u,%u,%.2f,%u,%u,%u,%u,%u,%u,%u,%u,%u\n", t, f.onsets[0], f.onsets[1], f.onsets[2], f.bpm, f.beatPhase,
              f.beatCount, f.beatInBar, f.confidence, f.levels[0], f.levels[1], f.levels[2], f.energy, f.status);
    }
    if (wantSummary) rows.push_back(Row{t, f});
    // A beat the host would see: beatCount advanced, so the beat happened phase * period ago (phase is at the last sample)
    if (clicksPath && f.bpm > 0 && havePrev && (uint8_t)(f.beatCount - prevCount) == 1) {
      const double ago = f.beatPhase / 65536.0 * 60.0 / f.bpm;
      addClick(clicks, audio.sourceRate, t - ago, f.beatInBar == 0);
    }
    prevCount = f.beatCount;
    havePrev = true;
  }
  if (csv && csv != stdout) fclose(csv);
  if (wantSummary) summary(rows);
  if (clicksPath) {
    for (float &v : clicks) v = v > 1.0f ? 1.0f : (v < -1.0f ? -1.0f : v);
    if (!wav::write16(clicksPath, clicks, audio.sourceRate)) {
      fprintf(stderr, "cannot write %s\n", clicksPath);
      return 1;
    }
  }
  return 0;
}
