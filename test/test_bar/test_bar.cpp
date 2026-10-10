// SPDX-License-Identifier: EUPL-1.2
// Bar position (beatInBar): a stable grid matters more than a correct downbeat. See docs/design.md, "Bar position".
#include <unity.h>
#include <math.h>
#include <stdio.h>
#include <vector>
#include <beatsense/analyzer.h>
#include "../support/synth.h"

using namespace beatsense;

void setUp() {}
void tearDown() {}

struct BarRun {
  int steps = 0;          // beat steps seen after `from`
  int badSteps = 0;       // steps where beatInBar did not advance by exactly 1 although no re-alignment was counted
  int realigns = 0;       // Analyzer::barRealignments() at the end
  int realignsAfter = 0;  // ...of which counted after `checkFrom`
  double lastRealignT = 0;
  int good = 0, total = 0; // samples near a beat where beatInBar equals the true bar position (after checkFrom)
};

// Run a spec; `from` = start of step checking, `checkFrom` = start of the alignment check against the true downbeats
static BarRun runBar(const synth::Spec &sp, double from, double checkFrom, int expectShift = 0) {
  BarRun r;
  Analyzer a;
  int prevCount = -1, prevInBar = -1;
  unsigned prevRealigns = 0;
  const double period = 60.0 / sp.bpm;
  synth::run(a, synth::render(sp), [&](double t, Analyzer &an) {
    const Features &f = an.features();
    const unsigned re = an.barRealignments();
    if (re != prevRealigns) {
      r.lastRealignT = t;
      if (t >= checkFrom) r.realignsAfter++;
    }
    if (t >= from && prevCount >= 0 && f.beatCount != prevCount) {
      r.steps++;
      if (f.beatInBar != ((prevInBar + 1) & 3) && re == prevRealigns) r.badSteps++;
    }
    if (t >= from && prevCount >= 0 && f.beatCount == prevCount && f.beatInBar != prevInBar && re == prevRealigns) r.badSteps++; // mid-beat jump
    prevCount = f.beatCount;
    prevInBar = f.beatInBar;
    prevRealigns = re;
    if (t >= checkFrom && f.bpm > 0) {
      const double since = f.beatPhase / 65536.0 * period;
      if (since < 0.05) {
        const int n = (int)round((t - since - sp.startS) / period);
        const int shift = n >= sp.downShiftAtBeat ? sp.downShiftBy : 0;
        r.total++;
        if ((((n - shift) % 4 + 4) % 4) == f.beatInBar) r.good++;
      }
    }
  });
  r.realigns = (int)a.barRealignments();
  (void)expectShift;
  return r;
}

// (a) Identical kicks and hats: the downbeat is genuinely ambiguous, so the grid must not move at all after the first choice
void test_identical_kicks_never_realign() {
  for (int variant = 0; variant < 3; variant++) {
    synth::Spec sp;
    sp.seconds = 60;
    sp.seed = 1 + variant;
    sp.kickJitter = variant == 0 ? 0.0 : (variant == 1 ? 0.25 : 0.5); // also with kick velocities wobbling +-25 / 50%
    sp.noiseRms = variant == 2 ? 0.01 : 0.002;
    sp.snareAmp = variant == 0 ? 0.0 : 0.25; // a backbeat makes beats 2 and 4 louder: the downbeat is a toss-up between two slots
    const BarRun r = runBar(sp, 12.0, 12.0);
    char msg[100];
    snprintf(msg, sizeof msg, "variant %d: %d re-alignments, %d bad steps of %d", variant, r.realigns, r.badSteps, r.steps);
    printf("    %s\n", msg);
    TEST_ASSERT_TRUE_MESSAGE(r.steps > 60, msg);
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, r.badSteps, msg);
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, r.realigns, msg);
  }
}

// (b) An accented downbeat (louder kick, or a crash with equal kicks): locks to the right offset, then never flips
void test_accented_downbeat_locks_and_holds() {
  for (int variant = 0; variant < 3; variant++) {
    synth::Spec sp;
    sp.seconds = 60;
    sp.hats = variant != 2;
    sp.bpm = variant == 1 ? 128 : 124;
    sp.seed = 3 + variant;
    sp.kickJitter = 0.1;
    if (variant == 0) sp.downKick = 1.8;         // louder kick on beat 0
    else if (variant == 1) sp.crashAmp = 0.35;   // crash on beat 0, equal kicks
    else {
      sp.downKick = 1.3; // mild accent and no hats
      sp.crashAmp = 0.2;
    }
    const BarRun r = runBar(sp, 25.0, 25.0);
    char msg[110];
    snprintf(msg, sizeof msg, "variant %d: %d/%d aligned, %d re-alignments, %d bad steps", variant, r.good, r.total, r.realigns, r.badSteps);
    printf("    %s\n", msg);
    TEST_ASSERT_TRUE_MESSAGE(r.total > 40, msg);
    TEST_ASSERT_TRUE_MESSAGE(r.good == r.total, msg);
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, r.badSteps, msg);
    TEST_ASSERT_TRUE_MESSAGE(r.realigns <= 1, msg);
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, r.realignsAfter, msg);
  }
}

// (c) The accents move once by two beats mid-song (a new section): at most one re-alignment, within about 8 bars; no other change
void test_section_change_realigns_once() {
  for (int variant = 0; variant < 2; variant++) {
    synth::Spec sp;
    sp.seconds = 100;
    sp.seed = 7 + variant;
    sp.kickJitter = 0.1;
    sp.downKick = variant == 0 ? 1.8 : 1.0;
    sp.crashAmp = variant == 0 ? 0.0 : 0.35;
    const int shiftBeat = 80; // at 120 BPM: 40.2 s
    sp.downShiftAtBeat = shiftBeat;
    sp.downShiftBy = 2;
    const double shiftT = synth::beatTime(sp, shiftBeat);
    const double bar = 4 * 60.0 / sp.bpm;
    const BarRun r = runBar(sp, 20.0, shiftT + 12 * bar);
    char msg[120];
    snprintf(msg, sizeof msg, "variant %d: %d re-alignments (last at +%.1f bars), %d bad steps, %d/%d aligned at the end", variant,
             r.realigns, (r.lastRealignT - shiftT) / bar, r.badSteps, r.good, r.total);
    printf("    %s\n", msg);
    TEST_ASSERT_EQUAL_INT_MESSAGE(1, r.realigns, msg); // the initial choice is not counted; the section change is one
    TEST_ASSERT_TRUE_MESSAGE(r.lastRealignT > shiftT && r.lastRealignT < shiftT + 8 * bar + 1.0, msg);
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, r.badSteps, msg);
    TEST_ASSERT_TRUE_MESSAGE(r.total > 20 && r.good == r.total, msg);
  }
}

// A kickless breakdown: the grid free-runs with the clock and is still right (and nothing was re-aligned) when the groove returns
void test_bar_grid_survives_breakdown() {
  synth::Spec sp;
  sp.seconds = 70;
  sp.seed = 11;
  sp.kickJitter = 0.1;
  sp.downKick = 1.8;
  sp.breakFromS = synth::beatTime(sp, 50); // 25.2 s
  sp.breakToS = sp.breakFromS + 12.0;      // 6 bars without kick, snare or crash
  const BarRun r = runBar(sp, 14.0, sp.breakToS + 3.0);
  char msg[100];
  snprintf(msg, sizeof msg, "%d/%d aligned after the breakdown, %d re-alignments, %d bad steps", r.good, r.total, r.realigns, r.badSteps);
  printf("    %s\n", msg);
  TEST_ASSERT_TRUE_MESSAGE(r.total > 40 && r.good == r.total, msg);
  TEST_ASSERT_EQUAL_INT_MESSAGE(0, r.badSteps, msg);
  TEST_ASSERT_EQUAL_INT_MESSAGE(0, r.realigns, msg);
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_identical_kicks_never_realign);
  RUN_TEST(test_accented_downbeat_locks_and_holds);
  RUN_TEST(test_section_change_realigns_once);
  RUN_TEST(test_bar_grid_survives_breakdown);
  return UNITY_END();
}
