# beatsense

Firmware and DSP for an audio-analysis daughterboard: an **M5Stack StampS3** (ESP32-S3FN8) listens through an **INMP441** I2S microphone and reports
**tempo, beat phase, band levels, onsets and a slow energy value** over **I2C** to an LED controller (built for Lumifera, usable by any I2C host).
The analysis is a portable C++ library with no Arduino dependency, so it is unit-tested and tuned on a laptop, with synthetic audio or real WAV files.

State, not events: the host polls a 17-byte frame and gets the beat phase *as of the moment of the read*, so the poll rate only affects smoothness.

## Hardware

- M5Stack StampS3 (ESP32-S3FN8, 8 MB flash, no PSRAM). WiFi and Bluetooth are switched off.
- INMP441 breakout (24-bit I2S, fixed gain).
- 2 x 4.7 kOhm resistors: I2C pull-ups to the StampS3's 3V3. **The host has none**, they belong on this board.
- Power: the host's 5 V into the StampS3's 5V pad (its regulator makes the 3V3).

### Wiring

```
 INMP441                StampS3                         Lumifera (host)
 -------                -------                         ---------------
 VDD  ----------------- 3V3
 GND  ----------------- GND  ------------------------- GND   (shared!)
 SCK  ----------------- G5                              
 WS   ----------------- G6                  +5V  ------ 5V / VIN pad
 SD   ----------------- G7
 L/R  ----------------- GND  (left channel)
                        G13 (SDA) ---+-------------- SDA
                                     |
                                  4.7k to 3V3
                        G15 (SCL) ---+-------------- SCL
                                     |
                                  4.7k to 3V3
```

| INMP441 | StampS3 |
|---|---|
| VDD | 3V3 |
| GND | GND |
| SCK | G5 |
| WS | G6 |
| SD | G7 |
| L/R | GND (left channel) |

| StampS3 | Host | Notes |
|---|---|---|
| G13 | SDA | 4.7 kOhm pull-up to the StampS3 3V3 |
| G15 | SCL | 4.7 kOhm pull-up to the StampS3 3V3 |
| 5V / VIN pad | +5 V | |
| GND | GND | must be shared |

Host side:

| Lumifera board | 5 V | GND | SDA | SCL |
|---|---|---|---|---|
| **v1 / v1 mini**: I2C connector (Molex 502352-0400) | pin 2 (+5V) | pin 1 | pin 3 = IO2 | pin 4 = IO32 |
| **v0.5** (ESP32-DevKitC V4, wires soldered to the header) | 5V header pin | GND header pin | IO33 | IO16 (IO17 on a WROOM module; 16/17 are used by PSRAM on WROVER) |

The v1 boards have 47 Ohm series resistors on the data lines. Pins avoided on the StampS3: G0 (boot button), G3/G45/G46 (strapping),
G19/G20 (USB), G43/G44 (UART0), G21 (internal RGB LED). All pins are in [`src/config.h`](src/config.h).

Notes:

- GND must be common between the three boards.
- **Do not feed the host's 5 V into the StampS3 while it is also powered over USB-C** unless you have checked the board has reverse-current protection. Disconnect the 5 V wire when debugging on USB.
- Keep the microphone wires short (under 10 cm). I2S at about 1.4 MHz BCLK is fine on short wires and picks up noise on long ones.
- On Lumifera v1, SDA is IO2, a strapping pin: unplug the daughterboard when flashing the host over USB (see the Lumifera audio plan).
- Do not plug 3.3 V-only Qwiic/STEMMA modules into the Lumifera connector (it carries 5 V).

## I2C protocol (version 1)

Slave, 7-bit address **0x42** (configurable in `src/config.h`), 400 kHz. Defined in [`include/beatsense_protocol.h`](include/beatsense_protocol.h)
(MIT licensed, with encode/decode/CRC helpers). **Any read returns this frame from offset 0**; multi-byte fields are little-endian.

| Offset | Field | Type | Meaning |
|---|---|---|---|
| 0 | `version` | u8 | protocol version (1) |
| 1 | `status` | u8 | bit0 signal present, bit1 beat locked, bit2 clipping |
| 2 | `bpm` | u16 Q8.8 | tempo |
| 4 | `beatPhase` | u16 | 0-65535 = position within the current beat at the time of the read (0 = the beat) |
| 6 | `beatCount` | u8 | wraps; increments each beat |
| 7 | `beatInBar` | u8 | 0-3, best guess |
| 8 | `confidence` | u8 | 0-255 |
| 9 | `levelLow/Mid/High` | 3 x u8 | AGC-normalised levels |
| 12 | `onsetLow/Mid/High` | 3 x u8 | peak onset strength since the previous read (cleared on read) |
| 15 | `energy` | u8 | slow (~8 bar) energy relative to the long-term average, 128 = average |
| 16 | `crc` | u8 | CRC-8 (poly 0x07, init 0) over bytes 0-15 |

Write `[register, value]`:

| Register | Value |
|---|---|
| 0x10 AGC mode | 0 normal, 1 vivid (3 s release), 2 lazy (20 s), 3 off (fixed gain) |
| 0x11 noise gate | threshold as -dBFS RMS (66 = -66 dBFS); 0 = default |
| 0x12 latency offset | int8 ms, shifts `beatPhase` ahead (positive) to cover the host's display latency |
| 0x13 tap | "a beat is now": sets the phase, two taps set the tempo |

The CRC and version byte let a host reject garbage when no board is fitted. Tempo is folded into 80-170 BPM (174 is reported as 87).

## Build and test

```
pio test -e native          # unit tests on the host (Unity), ~5 s
pio run -e stamps3          # firmware; first build downloads the ESP32-S3 toolchain
pio run -e stamps3 -t upload && pio device monitor
```

The firmware prints the features as CSV at about 20 Hz over USB serial (`BEATSENSE_DEBUG_CSV` in `src/config.h`).

### Tuning on real songs

```
g++ -std=c++14 -O2 -Iinclude -Ilib/beatsense/src -Itest/support tools/analyze_wav/main.cpp lib/beatsense/src/beatsense/analyzer.cpp -o analyze_wav
./analyze_wav song.wav > song.csv            # per-hop CSV: time, onsets, bpm, phase, confidence, levels, energy, status
./analyze_wav song.wav --gain-db -20 > quiet.csv
./analyze_wav song.wav --summary             # short text report instead of the CSV (see below)
./analyze_wav song.wav --clicks clicks.wav   # the song with a click on every predicted beat
./analyze_wav song.wav --summary --csv song.csv   # report on stdout, CSV to a file ("-" = stdout)
python3 tools/plot.py song.csv [song.png]    # needs matplotlib
python3 tools/plot.py song.csv --from 110 --to 125   # zoom; windows under 30 s also mark the predicted beats
```

- `--clicks out.wav` writes the input (channels averaged, original sample rate, 16-bit) with a short click on each beat the analyzer
  predicts, using `beatCount` / `beatPhase` / `beatInBar` exactly as a host reading the frame would see them: a high, loud click on
  beatInBar 0 and a lower, quieter one on 1-3. Listen to check sync, tempo holds through breakdowns and where the downbeat lands.
- `--summary` prints: time to first lock, % of time locked, tempo segments (start-end, median BPM; wobbles under 2 s are merged),
  confidence min / median after the first lock, for each onset and level output the % of frames at 255 and the 50th / 95th percentile
  (frames with signal only), and energy min / median / max. Short enough to paste into a chat.
- `plot.py --from S --to S` zooms into a window. Onsets and levels are thin, semi-transparent lines drawn low, mid, high; in windows
  under 30 s the predicted beats (where `beat_phase` wraps) are vertical lines in every panel.

WAV input: 22050 Hz (used as is) or 44100 Hz (decimated by 2), 16/24/32-bit PCM or 32-bit float, mono or stereo (averaged).
Convert anything else: `ffmpeg -i song.mp3 -ac 1 -ar 22050 song.wav`. Put recordings in `test/audio/` (git-ignored).

## Layout

```
include/beatsense_protocol.h   I2C protocol (MIT)
lib/beatsense/                 portable DSP core: FFT, Analyzer (EUPL-1.2)
src/                           firmware (main.cpp) and pins (config.h)
test/                          native Unity suites; support/ has the synthetic audio and WAV I/O
tools/                         analyze_wav CLI, plot.py
docs/design.md                 the pipeline as implemented, with tuning constants
```

## License

[EUPL-1.2](LICENSE), except [`include/beatsense_protocol.h`](include/beatsense_protocol.h), which is MIT so that hosts under any license can include it.
