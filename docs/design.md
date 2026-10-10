# Design: the analysis pipeline as implemented

Plan and rationale: Lumifera `docs/extras/audio-processing-plan.md` (2e, 2f). This file records what the code does and why each
constant has its value. All tunables are fields of `beatsense::Config` (`lib/beatsense/src/beatsense/analyzer.h`).

## Timing

22050 Hz, 512-point FFT, hop 256: **11.6 ms per frame (86.13 frames/s)**, window 23.2 ms. The hop is the timing resolution of an onset.

Latency, from a sound's attack to the moment the Analyzer can know about it:

| Part | Time |
|---|---|
| Hop quantisation (attack lands anywhere in the 11.6 ms hop; frame is processed at its end) | 0 - 11.6 ms, mean 5.8 ms |
| Window build-up (log flux reacts as soon as the attack enters the Hann window's tail) | ~3 ms |
| Total, measured on the synthetic kick (`Config::onsetLatencySamples`) | **200 samples = 9.1 ms** |

`beatPhase` is compensated for this: the beat clock is shifted so that phase 0 is the *attack*, not the frame that noticed it, and the
phase refers to the **last captured sample**. Measured on synthetic audio, the predicted beat sits within +-6 ms of the true kick
(mean error -0.5 to +4 ms depending on tempo; the remainder is hop quantisation). Real instruments have slower attacks than the
synthetic kick; use the host-side latency register (`kRegLatencyMs`) or `onsetLatencySamples` to trim by ear.

On the firmware, the frame also carries the I2S DMA delay (one hop, 11.6 ms, before `i2s_read` returns) and the I2C poll
interval; `onRequest()` advances `beatPhase` by the time since the frame was produced, so the host sees "now" as of the read.
Add the host's own render latency through `kRegLatencyMs` (int8 ms, positive = phase runs ahead).

## Stages

1. **Input.** 32-bit slot >> 8 = signed 24-bit, scaled to +-1. DC-blocking high-pass `y = x - x1 + R y1` with R set for 20 Hz. Clipping
   flag: any raw sample >= 98.5% full scale, held 0.5 s.
2. **Spectrum.** Periodic Hann window, own radix-2 real FFT (`fft.h`: N/2-point complex FFT plus split step; verified against a
   naive DFT). Normalised so a full-scale sine reads 1.0 in its bin.
3. **Bands.** 8 log-spaced bands, edges 40, 130, 215, 345, 600, 1075, 2150, 4300 Hz and Nyquist, which are FFT bins
   1,3,5,8,14,25,50,100,256 at 22.05 kHz. Groups: **low** = bands 0-1 (43-190 Hz, kick), **mid** = 2-5 (215 Hz-2.1 kHz, snare, body),
   **high** = 6-7 (2.1 kHz up, hats). Band value = RMS over its bins.
4. **Onset strength.** `L = ln(1 + 1000 * bandRms)`; flux = sum of positive `L[t] - L[t-1]` over a group's bands, divided by the number
   of bands. The log makes onsets independent of gain (`ln(gA) - ln(gB) = ln(A/B)`); the constant 1000 means bands below about -60 dBFS
   barely move, which doubles as a soft noise gate. A slow mean (0.6 s) is subtracted and the result half-wave rectified, so steady
   texture does not count. The u8 `onsets` are that value divided by a slow peak follower (6 s release, floor 1.0 nepers so noise
   cannot fill the range), peak-held between I2C reads.
5. **Signal gate.** Frame RMS in dBFS (sine-referenced) against `gateDb` = -66 dBFS (INMP441 noise floor is about -87). On at gate + 4 dB,
   off after 0.6 s below gate. Without signal: levels and onsets are 0 and confidence decays.
6. **Levels + AGC.** Group amplitude, VU-smoothed (instant attack, 120 ms release), divided by a per-group peak follower (attack 10 ms,
   release 8 s; vivid 3 s, lazy 20 s). The follower never drops below -54 dBFS, which **limits the gain to +54 dB**: nearly empty bands
   (window leakage, noise) are not blown up to full scale. Consequence: a band whose peaks stay below -54 dBFS reads low, by design.
   `kAgcOff` uses a fixed gain of 4 instead.
7. **Tempo** (every 22 frames, 0.25 s). The low+mid(x0.6) onset signal of the last 512 frames (5.9 s) is mean-removed and smoothed
   (3 taps); its autocorrelation is computed for lags up to 272 frames. Candidates 80-170 BPM in 0.25 BPM steps are scored by a **comb
   over the lags of 1, 2, 3 and 4 beats** (weights 1, .8, .6, .5; cubic interpolation between lags), which gives four times the
   resolution of a single-lag peak. The score is weighted by a log-normal octave prior (centre 120 BPM, sigma 0.55 octave); the best
   candidate is refined by a log-parabola (Gaussian) through its raw neighbours. Confidence = raw comb score mapped 0.08 -> 0, 0.40 -> 1.
   A tempo different from the current one must win 3 updates in a row before it replaces it (no octave flip-flop); agreeing candidates
   (within 3%) are blended in with factor 0.3.
   *Holding the tempo (breakdowns, metrical levels).* Found on a real techno track: when the kick drops out, a 3-against-2 pattern in
   the remaining percussion (dotted-eighth stabs) scores higher than the real tempo and the output flipped to 140 x 2/3 = 93 BPM for
   seconds, breaking the phase. Four measures, all in `Config`:
   (a) *Breakdown detector.* Low-band amplitude averaged over 1 s against its own slow average (12 s; 60 s while in a breakdown, so
   the kick returning ends it at once and a permanent change ends it by itself after ~40 s). Ratio < 0.4 (-8 dB) enters, > 0.6 leaves,
   only while there is signal. In a breakdown no tempo change is considered (pending evidence is cleared), the beat clock free-runs
   on its prediction (no PLL correction from the leftover percussion), and `locked` is held.
   (b) *Harmonic ratios need evidence.* A candidate at 3:2, 4:3, 5:4 or 5:3 (either way, +-4%) of the current tempo is the same music
   read at another level, so it must win 32 updates in a row (~8 s), or 12 (~3 s) when the current tempo has no support (raw score
   < `confLow`) and the candidate is strong (raw >= 0.4). Other changes (e.g. 120 -> 128) and octaves keep the 3-update rule.
   (c) *Continuity prior.* While a tempo is held, candidates near it or its 2x / 0.5x score up to 25% higher (width 0.03 octave),
   so a near tie resolves to the established tempo.
   (d) *Confidence is the support of the reported tempo*: the best raw comb score within +-3% of the current tempo, not of the best
   candidate. A breakdown or a pending change therefore lowers confidence while `locked` stays on.
   Test: `test_breakdown_holds_tempo_and_phase` (30 s groove, 8.6 s kickless breakdown with stabs on every third 16th, groove back;
   140 +-1 throughout, phase within 20 ms within 2 s of the kick returning). It failed before the fix (93.3 BPM).
   *Tempo accuracy.* Two causes made 140 BPM read 139.76 (-0.17%) on synthetic audio and on a real track. (1) The comb interpolated the
   autocorrelation linearly; a peak that is 3-4 lags wide has its maximum at the integer lags, and all four comb terms round the same
   way (36.91 frames/beat becomes 37 = 139.68 BPM), so the estimate was pulled towards a whole number of frames per beat. The comb now
   interpolates with a Catmull-Rom cubic, and the final parabola uses the raw (not prior-weighted) scores around the picked peak, since
   the prior slopes across the peak. (2) What remains is about +-0.1% of estimator error (the peak is skewed), which the phase loop
   below removes. Test: `test_tempo_accuracy_after_15s` (90/120/128/140/174->87 within 0.05 BPM from 15 s; before: 120 was off by 0.18).
   *Octaves.* The range is 80-170 BPM, so only 80-85 is ambiguous with double time; the prior settles those. Tempi outside the range are
   folded in: **174 BPM is reported as 87** (beat on every second kick), 60 as 120, 180 as 90. Hosts that want double time can multiply.
8. **Beat phase.** Every 4 frames: the onset signal is correlated with a pulse train at the current beat period (8 pulses, older ones
   weighted 0.8^j, each pulse 3 taps wide); the peak (parabolic refinement) gives the time since the last beat, plus the onset latency.
   The beat clock runs at `clockBpm_` (the reported tempo) and predicts continuously. The PLL is second order: a proportional term moves
   the phase 10% of the error per measurement (about 11 measurements per beat at 120 BPM), and an integral term (`pllTrimGain`, 3x
   stronger for the first 12 s after a new tempo) turns a steady phase error into a correction of `clockBpm_` (limited to +-2% of the
   estimate; only while locked and outside breakdowns). The tempo estimate from stage 7 moves `clockBpm_` only slowly (`clockFollow`,
   0.5% per update), so the +-0.1 BPM jitter of the estimate does not reach the output, but a clock that was started on a slightly
   wrong estimate converges to the true period from the phase errors (time constant ~10 s). The pulse train itself is built from the
   *estimated* period, not the clock's: its peak is biased by the period error (older pulses drift off the kicks), and if that bias
   depended on `clockBpm_` the integral term would feed back on itself and slowly run away at slow tempi. In a breakdown the
   estimate and the clock tempo are frozen, so the clock free-runs at the learned tempo: `test_free_running_drift_over_breakdown`
   (140 BPM, 10 s without kick) drifts at most 5 ms; before the change 17 ms.
   Errors above 0.2 beat must persist for 10 measurements before the clock snaps; a tempo jump snaps at the next measurement.
   `beatCount` increments when the clock wraps.
9. **Confidence + lock.** The raw confidence is smoothed (rise 1 s, fall 1.5 s). Locked turns on at 0.5 and off below 0.25 (hysteresis); a breakdown (stage 7) holds it.
10. **Energy.** RMS smoothed over 15 s (about 8 bars at 125 BPM) divided by a long-term average (cumulative, then 180 s); output
    `128 + 64 * log2(ratio)`, so 128 = average, 0 / 255 = 4x quieter / louder.
11. **beatInBar** (bar position). For the LED host a *stable* bar grid matters far more than a *correct* downbeat: a wrong but steady
    downbeat looks fine, a flipping one breaks bar-level effects. So the bar position is a counter that steps 0,1,2,3,0,... in
    lockstep with the beat clock (`stepBeat`), and nothing else moves it except a counted re-alignment. Evidence: for each beat
    the **accent** in the window from 0.25 beat before to 0.25 beat after is recorded in one of four slots (of the absolute beat index;
    EMA 0.4 per bar). The accent is, per group (weights low 1, mid 0.5, high 0.5), the summed onset relative to the group's own peak
    (new elements and crashes enter on downbeats) plus the maximum level relative to its peak (a louder kick: the log-compressed onset
    is blind to loudness; summing the onsets keeps the hop grid from favouring some beats). Evidence is only collected while locked
    and outside breakdowns. Decisions are taken once per bar, on its last beat, and applied at the next beat:
    - *First choice*: once every slot has been seen twice (2 bars), the strongest slot becomes the downbeat, however small the lead.
      In pure four-on-the-floor with identical kicks this choice is arbitrary, and then the grid never moves.
    - *Re-alignment*: one other slot must beat all others by 12% and by 2.0 standard deviations of the difference of two slot averages
      (bar-to-bar scatter is tracked per slot) at 4 bar boundaries in a row. Then the counter is shifted once, at a beat boundary, and
      `Analyzer::barRealignments()` is incremented. An accent that moves by two beats takes about 7 bars (3 for the averages to cross,
      4 sustained). A tempo jump forgets the evidence and starts again with a first choice.
    - Found on a real track: the old logic took the argmax of four noisy, nearly equal slot averages on every read, so it flipped
      between two candidates (low-high-low-high), repeated or skipped values and even changed in the middle of a beat. It failed
      the new test `test_identical_kicks_never_realign` (60 bad steps in 96 beats). Stress run: 40 synthetic songs of 4 minutes with
      random kick velocities (+-25%) and a backbeat, no re-alignment; with +-50% velocity, 1 of 40.
12. **Tap.** `tap()` sets the phase to 0 (a tap is a beat). Two taps 0.35-0.75 s apart also set the tempo (if it differs by more than 3%).
    The autocorrelation overrides it again if the music disagrees for 3 updates.

## Output scaling

The u8 onsets are divided by slow peak followers (6 s release), so a typical peak lands near 255 by construction; on a real techno
track 1-1.5% of the frames are at 255 and the 95th percentile is 87-126, which is fine. The levels are divided by **1.15 x** their
follower (8 s release) peak (`Config::levelHeadroom`, also for the fixed-gain mode), so typical peaks land at 255 / 1.15 = 222 and a
frame at full scale means a real overshoot above the recent peak. Before, the low level sat at 255 in 7% of the frames of a real track
(p95 = 255) and 2.6% of the synthetic groove; now 0% on the synthetic (`test_levels_keep_headroom`). Long plots of a real track
overplot 86 frames/s into ~1200 px, which makes sparse peaks look like a solid band; zoom with `plot.py --from/--to`.

## Cost

`sizeof(Analyzer)` is about 20 KB (static). Per hop: FFT + bands (~0.1 ms on the S3, estimated), the autocorrelation every 22 frames
is ~140k multiply-adds (~2 ms), phase every 4 frames ~1.5k interpolations. Budget is 11.6 ms per hop; measure on hardware.

## FFT backend seam

`Analyzer` uses `BEATSENSE_FFT_CLASS` (default `RealFft<512>`): a default-constructible class with `forward(const float *in, float *re,
float *im)` writing 257 bins. An esp-dsp wrapper (Apache-2.0) only needs to provide that.

## What the tests establish, and what they cannot

All tests use synthetic audio (decaying 60 Hz sine bursts, noise hats, optional snare, background noise): they prove the algorithms do
what they claim on clean, steady four-on-the-floor material. They say nothing about real music (syncopation, swing, tempo drift,
vocals, no kick). Use `tools/analyze_wav` with real songs to tune `tempoWeight`, `onsetMeanTauS`, `logGain`, `pllGain` and the
confidence mapping.
