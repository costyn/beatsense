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

// Signed distance (ms) from the clock's predicted last beat (as of time t) to the nearest true beat
static double beatErrorMs(const Features &f, double t, const synth::Spec &sp) {
  const double since = f.beatPhase / 65536.0 * 60.0 / f.bpm;
  const double last = t - since;
  const double n = round((last - sp.startS) / (60.0 / sp.bpm));
  return (last - (sp.startS + n * 60.0 / sp.bpm)) * 1000.0;
}

struct Result {
  double bpmAt8 = 0, bpmEnd = 0, maxBpmDev = 0, maxPhaseErr = 0, meanPhaseErr = 0, lockTime = -1;
  bool octaveError = false;
  int inBarCorrect = 0, inBarTotal = 0;
};

static Result track(const synth::Spec &sp, double checkFrom = 8.0) {
  Result r;
  Analyzer a;
  int n = 0;
  double sum = 0;
  synth::run(a, synth::render(sp), [&](double t, Analyzer &an) {
    const Features &f = an.features();
    if (r.lockTime < 0 && (f.status & proto::kStatusLocked)) r.lockTime = t;
    if (t >= 8.0 && r.bpmAt8 == 0) r.bpmAt8 = f.bpm;
    r.bpmEnd = f.bpm;
    if (t >= checkFrom && f.bpm > 0) {
      const double e = beatErrorMs(f, t, sp);
      r.maxPhaseErr = fmax(r.maxPhaseErr, fabs(e));
      sum += e;
      n++;
    }
  });
  r.meanPhaseErr = n ? sum / n : 0;
  return r;
}

// Expected reported tempo for a true tempo: inside 80-170 as is; outside, folded by octaves
static double expectedBpm(double bpm) {
  while (bpm > 170) bpm /= 2;
  while (bpm < 80) bpm *= 2;
  return bpm;
}

void test_tempo_within_one_bpm_after_8s_with_noise() {
  const double bpms[] = {90, 120, 128, 140};
  for (double bpm : bpms) {
    synth::Spec sp;
    sp.bpm = bpm;
    sp.seconds = 16;
    sp.noiseRms = 0.01; // -40 dBFS background noise
    const Result r = track(sp);
    char msg[64];
    snprintf(msg, sizeof msg, "tempo %.0f at 8 s: got %.2f", bpm, r.bpmAt8);
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(1.0f, (float)bpm, (float)r.bpmAt8, msg);
    snprintf(msg, sizeof msg, "tempo %.0f at end: got %.2f", bpm, r.bpmEnd);
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(1.0f, (float)bpm, (float)r.bpmEnd, msg);
  }
}

// 174 BPM is outside the 80-170 range: the tracker reports the half-time tempo, 87 BPM, with the beat on every second kick
void test_tempo_174_is_reported_as_87() {
  synth::Spec sp;
  sp.bpm = 174;
  sp.seconds = 16;
  sp.noiseRms = 0.01;
  const Result r = track(sp);
  TEST_ASSERT_FLOAT_WITHIN(1.0f, 87.0f, (float)r.bpmAt8);
  TEST_ASSERT_FLOAT_WITHIN(1.0f, 87.0f, (float)r.bpmEnd);
}

// 120 must never be reported as 60 or 240 (or 80 / 180), at any point after lock
void test_octave_preference_120() {
  for (uint32_t seed = 1; seed <= 3; seed++) {
    synth::Spec sp;
    sp.bpm = 120;
    sp.seconds = 20;
    sp.seed = seed;
    sp.snareAmp = seed == 2 ? 0.25 : 0; // seed 2 adds a snare on 2 and 4
    Analyzer a;
    double minB = 1e9, maxB = 0;
    synth::run(a, synth::render(sp), [&](double t, Analyzer &an) {
      if (t > 6.0 && an.features().bpm > 0) {
        minB = fmin(minB, an.features().bpm);
        maxB = fmax(maxB, an.features().bpm);
      }
    });
    TEST_ASSERT_TRUE_MESSAGE(minB > 118.5 && maxB < 121.5, "120 BPM drifted or hit another octave");
  }
}

// Fast and slow tempi in range keep their own octave
void test_octave_range_edges() {
  const double bpms[] = {82, 100, 160};
  for (double bpm : bpms) {
    synth::Spec sp;
    sp.bpm = bpm;
    sp.seconds = 20;
    const Result r = track(sp);
    char msg[64];
    snprintf(msg, sizeof msg, "tempo %.0f: got %.2f", bpm, r.bpmEnd);
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(1.5f, (float)expectedBpm(bpm), (float)r.bpmEnd, msg);
  }
}

// After lock, the predicted beat is within 20 ms of the true kick (typically a few ms on this clean synthetic)
void test_phase_within_20ms() {
  const double bpms[] = {90, 120, 128, 140};
  for (double bpm : bpms) {
    synth::Spec sp;
    sp.bpm = bpm;
    sp.seconds = 24;
    sp.noiseRms = 0.01;
    sp.snareAmp = 0.2;
    const Result r = track(sp, 10.0);
    char msg[80];
    snprintf(msg, sizeof msg, "phase error at %.0f BPM: max %.1f ms mean %.1f ms", bpm, r.maxPhaseErr, r.meanPhaseErr);
    printf("    %s\n", msg);
    TEST_ASSERT_TRUE_MESSAGE(r.maxPhaseErr < 20.0, msg);
  }
}

// 174 BPM: the clock tracks the 87 BPM grid; its beats must coincide with kicks (every second one)
void test_phase_174_lands_on_kicks() {
  synth::Spec sp;
  sp.bpm = 174;
  sp.seconds = 24;
  sp.noiseRms = 0.01;
  Analyzer a;
  double maxErr = 0;
  synth::run(a, synth::render(sp), [&](double t, Analyzer &an) {
    const Features &f = an.features();
    if (t < 10.0 || f.bpm <= 0) return;
    const double last = t - f.beatPhase / 65536.0 * 60.0 / f.bpm;
    const double n = round((last - sp.startS) / (60.0 / 174.0));
    maxErr = fmax(maxErr, fabs(last - (sp.startS + n * 60.0 / 174.0)) * 1000.0);
  });
  TEST_ASSERT_TRUE_MESSAGE(maxErr < 20.0, "87 BPM beats are not on kicks");
}

void test_beat_count_increments_once_per_beat() {
  synth::Spec sp;
  sp.seconds = 20;
  Analyzer a;
  int last = -1, jumps = 0, increments = 0;
  double firstT = 0, lastT = 0;
  synth::run(a, synth::render(sp), [&](double t, Analyzer &an) {
    if (t < 10.0) return;
    const int c = an.features().beatCount;
    if (last >= 0 && c != last) {
      if (((c - last) & 255) == 1) increments++;
      else jumps++;
      if (!firstT) firstT = t;
      lastT = t;
    }
    last = c;
  });
  TEST_ASSERT_EQUAL_INT(0, jumps);
  TEST_ASSERT_INT_WITHIN(1, (int)round((lastT - firstT) / 0.5), increments - 1);
}

void test_beat_in_bar_is_stable_and_cycles() {
  synth::Spec sp;
  sp.seconds = 30;
  Analyzer a;
  int mismatches = 0, checks = 0;
  int prevCount = -1, prevInBar = -1;
  synth::run(a, synth::render(sp), [&](double t, Analyzer &an) {
    if (t < 12.0) return;
    const Features &f = an.features();
    if (prevCount >= 0 && f.beatCount != prevCount) {
      checks++;
      if (f.beatInBar != ((prevInBar + 1) & 3)) mismatches++;
    }
    prevCount = f.beatCount;
    prevInBar = f.beatInBar;
  });
  TEST_ASSERT_TRUE(checks > 20);
  TEST_ASSERT_EQUAL_INT_MESSAGE(0, mismatches, "beatInBar should step 0-3 in order once the bar position is settled");
}

// Downbeat: kick accented on the first beat of a 4-beat bar -> beatInBar 0 lands on the accented beat
void test_downbeat_follows_accent() {
  synth::Spec sp;
  sp.seconds = 40;
  sp.hats = false;
  auto x = synth::render(sp);
  // Boost every fourth kick (beats 0,4,8..) by re-adding a second kick burst on it
  synth::Spec accent = sp;
  accent.kickAmp = 0.5;
  accent.bpm = sp.bpm / 4;
  accent.hats = false;
  accent.noiseRms = 0;
  accent.seed = 5;
  const auto extra = synth::render(accent);
  for (size_t i = 0; i < x.size(); i++) x[i] += extra[i];
  Analyzer a;
  int good = 0, total = 0;
  synth::run(a, x, [&](double t, Analyzer &an) {
    if (t < 25.0) return;
    const Features &f = an.features();
    const double since = f.beatPhase / 65536.0 * 0.5;
    const double last = t - since;
    const double n = round((last - sp.startS) / 0.5);
    if (since > 0.05) return;
    total++;
    if ((((int)n) & 3) == f.beatInBar) good++;
  });
  TEST_ASSERT_TRUE(total > 10);
  TEST_ASSERT_TRUE_MESSAGE(good > total * 9 / 10, "beatInBar is not aligned to the accented beat");
}

// 120 -> 128: re-locks within 10 s of the change, with phase back on the kicks
void test_tempo_change_relocks() {
  synth::Spec s1;
  s1.bpm = 120;
  s1.seconds = 16;
  s1.noiseRms = 0.005;
  synth::Spec s2 = s1;
  s2.bpm = 128;
  s2.startS = 0.35;
  s2.seed = 9;
  auto x = synth::render(s1);
  const double change = 16.0;
  const auto y = synth::render(s2);
  x.insert(x.end(), y.begin(), y.end());
  Analyzer a;
  double relockAt = -1;
  double maxErrAfter = 0;
  synth::run(a, x, [&](double t, Analyzer &an) {
    const Features &f = an.features();
    if (t > change + 0.5 && relockAt < 0 && fabs(f.bpm - 128.0) < 1.0) relockAt = t - change;
    if (t > change + 12.0) {
      synth::Spec ref = s2;
      ref.startS = change + s2.startS;
      maxErrAfter = fmax(maxErrAfter, fabs(beatErrorMs(f, t, ref)));
    }
  });
  printf("    re-locked %.1f s after the change; max phase error after 12 s: %.1f ms\n", relockAt, maxErrAfter);
  TEST_ASSERT_TRUE_MESSAGE(relockAt > 0 && relockAt < 10.0, "did not re-lock to 128 BPM within 10 s");
  TEST_ASSERT_TRUE_MESSAGE(maxErrAfter < 20.0, "phase not back on the kicks after the tempo change");
  TEST_ASSERT_FLOAT_WITHIN(1.0f, 128.0f, a.features().bpm);
}

// A breakdown without the kick, where the remaining percussion is a strong 3:2 pattern (stab on every third 16th = 93.3 BPM
// when read as a beat). The established 140 must be held, and the clock must still be on the kicks when the groove returns.
void test_breakdown_holds_tempo_and_phase() {
  synth::Spec sp;
  sp.bpm = 140;
  sp.seconds = 52;
  sp.noiseRms = 0.005;
  sp.snareAmp = 0.25;
  sp.breakFromS = 0.2 + 70 * 60.0 / 140; // beat 70 = 30.2 s, so the stabs start on a beat
  sp.breakToS = 0.2 + 90 * 60.0 / 140;   // 8.6 s breakdown, kicks back on a beat
  sp.stabAmp = 0.35;
  Analyzer a;
  double minB = 1e9, maxB = 0, recoverAt = -1, maxErrAfter = 0, lockedNum = 0, lockedDen = 0;
  synth::run(a, synth::render(sp), [&](double t, Analyzer &an) {
    const Features &f = an.features();
    if (t >= sp.breakFromS && t < sp.breakToS + 0.5) {
      minB = fmin(minB, f.bpm);
      maxB = fmax(maxB, f.bpm);
    }
    if (t >= sp.breakFromS && t < sp.breakToS) {
      lockedDen++;
      if (f.status & proto::kStatusLocked) lockedNum++;
    }
    if (t >= sp.breakToS + 2.0) maxErrAfter = fmax(maxErrAfter, fabs(beatErrorMs(f, t, sp)));
    if (t >= sp.breakToS && recoverAt < 0 && fabs(beatErrorMs(f, t, sp)) <= 20.0) recoverAt = t - sp.breakToS;
  });
  printf("    breakdown bpm %.2f..%.2f, locked %.0f%%, phase ok %.2f s after the kick returns, max error after 2 s: %.1f ms\n", minB, maxB,
         100.0 * lockedNum / lockedDen, recoverAt, maxErrAfter);
  TEST_ASSERT_TRUE_MESSAGE(minB > 139.0 && maxB < 141.0, "tempo left 140 BPM during the breakdown");
  TEST_ASSERT_TRUE_MESSAGE(maxErrAfter <= 20.0, "phase error > 20 ms 2 s after the kick returned");
}

// The reported tempo converges to the true one: the comb's linear interpolation used to round every lag towards whole frames (140 was
// reported as 139.76), and a first-order PLL cannot correct a period error that the autocorrelation hides.
void test_tempo_accuracy_after_15s() {
  const double bpms[] = {90, 120, 128, 140, 174};
  for (double bpm : bpms) {
    synth::Spec sp;
    sp.bpm = bpm;
    sp.seconds = 25;
    Analyzer a;
    double worst = 0;
    const double truth = expectedBpm(bpm > 170 ? bpm / 2 : bpm);
    synth::run(a, synth::render(sp), [&](double t, Analyzer &an) {
      if (t >= 15.0) worst = fmax(worst, fabs(an.features().bpm - truth));
    });
    char msg[80];
    snprintf(msg, sizeof msg, "tempo %.0f: worst error after 15 s %.3f BPM", bpm, worst);
    printf("    %s\n", msg);
    TEST_ASSERT_TRUE_MESSAGE(worst <= 0.05, msg);
  }
}

// During a breakdown the clock free-runs on its tempo: a 0.17% tempo error (139.76 for 140) is 14 ms over 8 s
void test_free_running_drift_over_breakdown() {
  synth::Spec sp;
  sp.bpm = 140;
  sp.seconds = 44;
  sp.breakFromS = 0.2 + 70 * 60.0 / 140; // 30.2 s
  sp.breakToS = sp.breakFromS + 10.0;    // no kick for 10 s, hats at half level
  Analyzer a;
  double before = 0, after = 0;
  synth::run(a, synth::render(sp), [&](double t, Analyzer &an) {
    if (t >= sp.breakFromS - 0.05 && before == 0) before = beatErrorMs(an.features(), t, sp);
    if (t >= sp.breakToS - 0.05 && after == 0) after = beatErrorMs(an.features(), t, sp);
  });
  char msg[80];
  snprintf(msg, sizeof msg, "free-running drift over 10 s: %.1f ms (%.1f -> %.1f)", after - before, before, after);
  printf("    %s\n", msg);
  TEST_ASSERT_TRUE_MESSAGE(fabs(after - before) <= 5.0, msg);
}

void test_silence_drops_confidence_and_signal() {
  synth::Spec sp;
  sp.seconds = 12;
  auto x = synth::render(sp);
  x.resize(x.size() + (size_t)(10 * synth::kFs), 0.0f); // digital silence
  Analyzer a;
  bool lockedBefore = false, signalBefore = false;
  double signalGoneAt = -1, unlockedAt = -1;
  uint8_t confAt4 = 255;
  synth::run(a, x, [&](double t, Analyzer &an) {
    const Features &f = an.features();
    if (t > 11.9 && t < 12.0) {
      lockedBefore = f.status & proto::kStatusLocked;
      signalBefore = f.status & proto::kStatusSignal;
    }
    if (t > 12.0 && signalGoneAt < 0 && !(f.status & proto::kStatusSignal)) signalGoneAt = t - 12.0;
    if (t > 12.0 && unlockedAt < 0 && !(f.status & proto::kStatusLocked)) unlockedAt = t - 12.0;
    if (t > 15.9 && t < 16.0) confAt4 = f.confidence;
  });
  TEST_ASSERT_TRUE(lockedBefore && signalBefore);
  printf("    signal cleared %.1f s, unlocked %.1f s after silence\n", signalGoneAt, unlockedAt);
  TEST_ASSERT_TRUE_MESSAGE(signalGoneAt > 0 && signalGoneAt < 2.0, "signal-present flag should clear within 2 s of silence");
  TEST_ASSERT_TRUE_MESSAGE(unlockedAt > 0 && unlockedAt < 4.0, "lock should drop within 4 s of silence");
  TEST_ASSERT_TRUE_MESSAGE(confAt4 < 40, "confidence should decay in silence");
  for (int g = 0; g < 3; g++) TEST_ASSERT_EQUAL_UINT8(0, a.features().levels[g]);
}

void test_tap_sets_phase_and_tempo() {
  Analyzer a;
  const std::vector<int32_t> quiet(256, 0);
  // Two taps 0.5 s apart (43 hops) -> ~120 BPM, phase 0 at the second tap
  for (int i = 0; i < 43; i++) {
    if (i == 0) a.tap();
    a.process(quiet.data(), 256);
  }
  a.tap();
  TEST_ASSERT_FLOAT_WITHIN(3.0f, 120.0f, a.features().bpm);
  TEST_ASSERT_TRUE(a.features().beatPhase < 100);
  for (int i = 0; i < 21; i++) a.process(quiet.data(), 256);
  TEST_ASSERT_UINT16_WITHIN(6000, 32768, a.features().beatPhase); // about half a beat later
}

void test_host_latency_offset_shifts_phase() {
  synth::Spec sp;
  sp.seconds = 12;
  const auto x = synth::render(sp);
  Analyzer a, b;
  b.setHostLatencyMs(25);
  synth::feed(a, x);
  synth::feed(b, x);
  // 25 ms at 120 BPM = 0.05 beat = 3277 counts
  const int diff = ((int)b.features().beatPhase - (int)a.features().beatPhase + 65536) % 65536;
  TEST_ASSERT_INT_WITHIN(60, 3277, diff);
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_tempo_within_one_bpm_after_8s_with_noise);
  RUN_TEST(test_tempo_174_is_reported_as_87);
  RUN_TEST(test_octave_preference_120);
  RUN_TEST(test_octave_range_edges);
  RUN_TEST(test_phase_within_20ms);
  RUN_TEST(test_phase_174_lands_on_kicks);
  RUN_TEST(test_beat_count_increments_once_per_beat);
  RUN_TEST(test_beat_in_bar_is_stable_and_cycles);
  RUN_TEST(test_downbeat_follows_accent);
  RUN_TEST(test_tempo_change_relocks);
  RUN_TEST(test_breakdown_holds_tempo_and_phase);
  RUN_TEST(test_tempo_accuracy_after_15s);
  RUN_TEST(test_free_running_drift_over_breakdown);
  RUN_TEST(test_silence_drops_confidence_and_signal);
  RUN_TEST(test_tap_sets_phase_and_tempo);
  RUN_TEST(test_host_latency_offset_shifts_phase);
  return UNITY_END();
}
