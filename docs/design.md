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
   over the lags of 1, 2, 3 and 4 beats** (weights 1, .8, .6, .5; linear interpolation between lags), which gives four times the
   resolution of a single-lag peak. The score is weighted by a log-normal octave prior (centre 120 BPM, sigma 0.55 octave); the best
   candidate is refined by a parabola through its neighbours. Confidence = raw comb score mapped 0.08 -> 0, 0.40 -> 1.
   A tempo different from the current one must win 3 updates in a row before it replaces it (no octave flip-flop); agreeing candidates
   (within 3%) are blended in with factor 0.3.
   *Octaves.* The range is 80-170 BPM, so only 80-85 is ambiguous with double time; the prior settles those. Tempi outside the range are
   folded in: **174 BPM is reported as 87** (beat on every second kick), 60 as 120, 180 as 90. Hosts that want double time can multiply.
8. **Beat phase.** Every 4 frames: the onset signal is correlated with a pulse train at the current beat period (8 pulses, older ones
   weighted 0.8^j, each pulse 3 taps wide); the peak (parabolic refinement) gives the time since the last beat, plus the onset latency.
   The beat clock runs at the tempo estimate and predicts continuously; a first-order PLL moves its phase 10% of the error per
   measurement (about 11 measurements per beat at 120 BPM). Errors above 0.2 beat must persist for 10 measurements
   before the clock snaps; a tempo jump snaps at the next measurement. `beatCount` increments when the clock wraps.
9. **Confidence + lock.** The raw confidence is smoothed (rise 1 s, fall 1.5 s). Locked turns on at 0.5 and off below 0.25 (hysteresis).
10. **Energy.** RMS smoothed over 15 s (about 8 bars at 125 BPM) divided by a long-term average (cumulative, then 180 s); output
    `128 + 64 * log2(ratio)`, so 128 = average, 0 / 255 = 4x quieter / louder.
11. **beatInBar.** For each beat the largest low-band onset from 0.25 beat before to 0.25 beat after is recorded in one of four
    slots (EMA 0.15 per bar); the strongest slot is taken as the downbeat. Rough by design (needs an accented first beat; reset on
    a tempo jump).
12. **Tap.** `tap()` sets the phase to 0 (a tap is a beat). Two taps 0.35-0.75 s apart also set the tempo (if it differs by more than 3%).
    The autocorrelation overrides it again if the music disagrees for 3 updates.

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
