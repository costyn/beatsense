# beatsense - Claude instructions

Audio-analysis daughterboard for the Lumifera LED controller: M5Stack StampS3 (ESP32-S3) + INMP441 I2S mic, reports tempo, beat phase
and band levels over I2C. Design background (read-only, other repo): `Lumifera/docs/extras/audio-processing-plan.md`, sections 2c-2f.

## Layout

- `lib/beatsense/src/beatsense/` - the **portable DSP core** (`namespace beatsense`): `fft.h`, `analyzer.h/.cpp`. Pure C++14, no
  Arduino / ESP-IDF includes, no heap allocation after construction. Keep it that way: it must build with plain g++ for the tests and
  tools. Hardware code goes in `src/` only.
- `include/beatsense_protocol.h` - I2C protocol (MIT, so hosts of any license can include it). Everything else is EUPL-1.2.
- `src/` - Arduino firmware (`main.cpp`) and all pins (`config.h`).
- `test/test_*/` - native Unity suites; `test/support/` - synthetic audio (`synth.h`) and WAV I/O (`wav.h`).
- `tools/analyze_wav/` - host CLI that runs a WAV through the Analyzer and prints per-hop CSV; `tools/plot.py` plots it.
- `docs/design.md` - the pipeline as implemented and why each constant has its value. Update it when you change a constant.

## Build and test

```
pio test -e native        # all suites, ~5 s
pio run -e stamps3        # firmware (first build downloads the toolchain)
```

CI (`.github/workflows/ci.yml`) runs both on every push and PR. Run the native tests before every commit.

## Rules

- Tests must pass for honest reasons: tune the DSP, don't loosen thresholds. If a target can't be met, say so.
- Tunables live in `Config` (analyzer.h). New calculation-heavy code gets a native test.
- Code style: clang-format (LLVM, Attach braces, 140 columns, `SortIncludes: Never`). Comments explain why. Memory conscious.
- Source files carry `// SPDX-License-Identifier: EUPL-1.2` (the protocol header: MIT).
- Don't add dependencies on esp-dsp or arduinoFFT (GPL). An esp-dsp backend can be swapped in behind `BEATSENSE_FFT_CLASS`.
- Commits: conventional commits (`feat(dsp):`, `fix(firmware):`, `test:`, `docs:`, `chore:`).
- The firmware cannot be tested here; it must compile, and hardware behaviour is verified by the user.
