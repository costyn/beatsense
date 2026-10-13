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
   texture does not count. The u8 `onsets` are that value divided by a slow peak follower (6 s release, per-group floor
   `onsetOutFloor` = 0.5 / 0.2 / 0.04 nepers for low / mid / high, so noise cannot fill the range; see "Output scaling"), peak-held
   between I2C reads. The bar accent (stage 11) and the tempo/phase input are not affected: they use the raw onset and a separate
   follower with the 1.0 floor (`onsetPeakFloor`).
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
   the kick returning ends it at once and a permanent change ends it by itself after ~40 s). Ratio < 0.4 (-8 dB) enters, > 0.8 leaves,
   only while there is signal. Both averages are plain (cumulative) averages until their windows have filled, like the energy average
   (changed with the groove level): the slow one used to start from a single hop, which on a signal that begins with a kick is a peak, so
   it sat above the real mean and the ratio read 0.5-0.7 for ~12 s (found at -25 dB, where the first kick is what turns the signal gate
   on). In a breakdown no tempo change is considered (pending evidence is cleared), the beat clock free-runs
   on its prediction (no PLL correction from the leftover percussion), and `locked` is held. The state is reported as status bit 3
   (`kStatusBreakdown`; added without a protocol version bump, the layout is unchanged) and the ratio as `Features::breakdownRatio`
   (not in the frame; 1.0 while there is no signal), which the debug CSV and `analyze_wav` print as `breakdown` and `bd_ratio`.
   *Exit threshold 0.8 (was 0.6).* Live test 2026-10-13, room mic (StampS3 + INMP441) next to a small speaker, techno track "Concrete"
   (Mamin), 128 BPM, two breaks by ear at 2:00-2:30 and 4:45-5:15 (in the second only the bass drum stops, other percussion stays).
   With exit 0.6 the detector gave 2:02.1-2:29.8 and 4:47.5-5:03.6: the second break ended 11 s early because the ratio sat at
   0.62-0.71 from 5:03 to 5:15 (something returns in the low band before the kick). In the groove the ratio stays ~0.9-1.1 with brief
   dips not below ~0.8. Replaying the recorded ratio with exit 0.8 gives 2:02.2-2:30.1 and 4:47.5-5:14.8 (matches by ear). Short raw
   detections during fills (3:14.4 for 0.6 s, 4:28.4 for 1.6 s) remain; they are left to the host (Lumifera fades fx intensity
   smoothly, so brief ones are harmless). The lock held through both breaks; confidence fell to 4-72 during the first. Tests:
   `test_breakdown_persists_with_low_band_remnant` (bass note returning at ~70% of the groove's low amplitude before the kick; fails with
   exit 0.6), `test_breakdown_ends_after_permanent_change` (kick never returns: the held average ends the breakdown after ~40 s).
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

13. **Groove level** (`Features::groove`, frame byte 16). The host (an LED lamp) scales all its beat effects by this: full in the groove,
    about 10% in a breakdown, partly during a build. It does its own easing and fires its own "drop" event on the first kick, so this is
    a **raw, honest 0-255 measure of how much kick-driven groove is present right now**, not smoothed or debounced, reacting within about a
    second. A state like `levels` and `energy`, not peak-held like the onsets. 0 without signal or without a tempo. It is the product of
    two ramps, per hop, O(1):
    - *The low band is back*: the breakdown ratio of stage 7 (low-band amplitude averaged over 1 s / its slow average), linear
      `grooveRatioLow` 0.3 -> 0, `grooveRatioHigh` 0.9 -> 1. This separates the groove from a passage that still repeats at the beat but
      has lost its kick (a bass line, the other percussion; Concrete's second break has such a remnant).
    - *The low band repeats at the beat period* (steadiness): the correlation coefficient between the low-band amplitude (the group amplitude
      before the AGC, so level-independent) and the same signal one beat earlier (`60 / clockBpm`, linear interpolation), both with a slow
      mean removed (`grooveHighpassS` 0.25 s, leaves the kick modulation). Exponentially weighted sums `Rxy`, `Rxx`, `Ryy` (`grooveCorrS`
      0.5 s, about two beats at 120 BPM); `corr = max(Rxy, 0) / sqrt(Rxx * Ryy)`, mapped `grooveCorrLow` 0.3 -> 0, `grooveCorrHigh` 0.7 -> 1.
      The denominator never goes below `grooveFloorAmp`^2 (0.002 = -54 dBFS, like `agcFloorDb`), so a nearly empty low band cannot make a
      correlation out of noise. A steady kick, with the bass that goes with it, repeats every beat (0.75-0.95 on the three test tracks); a
      rumble, a held bass note (no modulation: correlation 0) or a 3-against-2 pattern (the rolling bass of Satory's build, period 2/3 beat)
      does not.
    - *Why not confidence*: it is smoothed (rise 1 s, fall 1.5 s) and measures the tempo comb over 6 s, so it lags a break by seconds, it
      fell to 4-72 during Concrete's break while the lock held, and it needs 3-4 s to recover after one. *Why not on-grid onset energy*
      (tried first, on the CSV columns): the low-band onset is noisy on real music and the wide onsets of a swell fill the beat grid as
      well as a kick does (steady kick 0.4-0.6 vs rumble 0.1-0.5, no usable threshold); the normalised correlation of the amplitude
      separated the cases (steady kick 0.65-0.95, rumble and build 0.0-0.2) and needs no threshold on the onset size. No 2-hop smoothing of
      the amplitude: the frame-to-frame alternation of a low bass tone (an FFT-window effect) is part of what keeps a bass-only passage from
      correlating at the beat lag, and smoothing it raised the false groove in the Satory build from 0.43 to 0.70.
    - *Why a product, and the ramps*: either factor missing means no groove. The host eases slowly down and quickly up, so a brief false
      high would get through and a brief false low would not: false highs weigh more than rise time. A correlation ramp of 0.2 -> 0.6
      rises ~0.3 s faster after a break but let the Satory build reach 0.59 (a few seconds of bass pattern at 4:56-4:59); the chosen
      0.3 -> 0.7 peaks at 0.43 there. A shorter `grooveCorrS` (0.35 s) did not rise noticeably faster, because the correlation needs a repeat:
      one beat of the new kick before the first pair exists.
    - *Measured* (three real tracks, converted to 22.05 kHz mono; identical at `--gain-db -25`: the sums are normalised and the ratio is
      relative). Steady groove: median 1.00, p10 0.82-1.00; below 0.5 for 6-8% of the time on Concrete (see below), 0-1% on Vapour, 0-4% on
      Satory. Long breaks: <= 0.15 within 0.5-0.8 s of the kick stopping, max 0.09 afterwards (Concrete 2:01 and 4:45, Vapour 3:00-4:02).
      Satory build 4:35-5:12 (bass and drums, no steady kick): median 0, max 0.43, above 0.3 for ~0.3 s in total. Rise after the kick
      returns (0.5 / 0.85): Vapour 4:02 0.58 / 0.82 s; Concrete 2:30 0.45 / 0.53 s; Concrete 5:15 1.46 / 1.77 s (kick, bass and the other
      percussion all return together, and the correlation of that mix needs two repeats). Satory: full by 5:16, after a short rolling-bass
      stretch from 5:14.9 (the amplitude shows the steady kick from about 5:16; the ear-and-confidence estimate was 5:20, confidence lags).
      One-bar kick drops of Satory (1.6-2.9 s): G < 0.5 for 2.2-3.7 s, about a second longer than the hole, which is the correlation's
      memory. Concrete dips below 0.5 for 1-2 s every 15 s (8 bars: 1:15, 1:30, 1:45, 3:45, 4:00, 5:30, 5:45) where the low-band pattern has a
      hole or a fill (a half-second hole at 1:45 is visible in the amplitude); they are left in as honest.
    - *Startup*: the correlation needs the beat period, so the groove is 0 until a tempo has been found (about 3 s).
    Tests (`test/test_groove`): >= 0.85 in a steady synthetic groove and at -25 dB, <= 0.15 within 1.5 s of the kick stopping and for the
    rest of the break, still low with a 60 Hz bass note playing (energy but no repetition), back to 0.85 within 1 s of the kick returning,
    a clear dip in a one-bar drop, 0 without signal.

## Protocol

`include/beatsense_protocol.h` (MIT). Version 2 is an 18-byte frame: `groove` at byte 16, CRC-8 over bytes 0-16 at byte 17. Version 1 was
17 bytes with the CRC at byte 16 (no `groove`). The version byte is checked together with the CRC, so a v2 host never accepts a v1 frame
and the other way round. Status bit 3 (breakdown) is unchanged and keeps its meaning (tempo hold); `groove` is a separate, finer signal.
The firmware copies `groove` from the last hop like `levels` and `energy` (a state: `onRequest()` neither holds nor resets it).

## Output scaling

**Onsets on a quiet (microphone) signal.** The original single floor of 1.0 nepers was tuned on line-level WAVs. Through a room
microphone (techno at 138 BPM, locked) the per-second maxima on the lamp were high 14-21, mid 52-82, low 46-152: the real hats move the
high bands by only ~0.06-0.08 nepers, because bands near -60 dBFS sit in the linear part of `ln(1 + 1000 x)` (flux is then proportional
to amplitude), so the floor, not the peak follower, set the divisor and the output never got near 255. Fix: the u8 output has its own
follower with a per-group floor (`Config::onsetOutFloor` = 0.5 / 0.2 / 0.04). The floor is deliberately not zero: in the linear regime
it is an amplitude threshold, which is what keeps noise from being scaled up. `logGain` is unchanged (it would change the raw flux and
with it tempo, phase and the noise rejection), and so is the follower behind the bar accent (`onsetPeakFloor` 1.0): the accent
compares a group with its own peak and its behaviour is the same as before. Reasons for the values, measured on the synthetic groove
(138 BPM) attenuated by 25-35 dB with an INMP441-like noise floor (-87 dBFS), and on noise alone:
- high 0.04: the quietest hats of the test still reach 255 with margin; white noise at -64 / -60 / -55 dBFS (above the gate) gives
  at most 86 / 132 / 219, i.e. no 160 trigger at -64 and -60. A floor of 0.03 or lower started to trigger on -55 dBFS hiss.
- mid 0.2, low 0.5: room rumble and a loud steady tone (`test_onsets_fire_on_kicks_not_on_steady_tone`: leakage ripple of 0.02 nepers
  in the low band, 8 of 255) stay low; real kicks are 0.2-0.6 nepers on the lamp.
- Gate hit counts (>= 160 trigger, < 80 re-arm) on the attenuated synthetic hats: 44-50 triggers for 50 hats. Some hats stay below 160
  because an attack that straddles two hops splits its flux (this is the same on line-level audio).
- Limit: a flux of 0.04 nepers cannot be told from noise by size alone. Noise at -45 dBFS white (far above any room) does fill the range
  of the high band. Tests: `test_quiet_mic_onsets_span_the_range`, `test_noise_above_gate_does_not_fill_the_range`.

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
