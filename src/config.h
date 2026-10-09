// SPDX-License-Identifier: EUPL-1.2
// Board configuration for the M5Stack StampS3 (ESP32-S3FN8, 8 MB flash, no PSRAM). All pins live here.
//
// Avoided: G0 (boot button), G3/G45/G46 (strapping), G19/G20 (USB), G43/G44 (UART0), G21 (internal RGB LED).
#ifndef BEATSENSE_CONFIG_H
#define BEATSENSE_CONFIG_H

#include <stdint.h>
#include "beatsense_protocol.h"

// --- INMP441 (I2S RX, left channel: L/R pin tied to GND) ---
constexpr int PIN_I2S_SCK = 5; // bit clock (INMP441 SCK)
constexpr int PIN_I2S_WS = 3;  // word select (INMP441 WS)
constexpr int PIN_I2S_SD = 7;  // data in (INMP441 SD)

// --- I2C slave towards the LED controller (host). 4.7k pull-ups to this board's 3V3 are on the daughterboard. ---
constexpr int PIN_I2C_SDA = 13;
constexpr int PIN_I2C_SCL = 15;
constexpr uint8_t I2C_ADDRESS = beatsense::proto::kDefaultI2cAddress;
constexpr uint32_t I2C_FREQUENCY = 400000;

// --- Audio ---
constexpr uint32_t SAMPLE_RATE = 22050;
constexpr int HOP_SAMPLES = 256;      // must equal beatsense::Config::kHop
constexpr int DMA_BUF_COUNT = 4;      // each DMA buffer holds one hop, so the task wakes once per hop with 3 hops of slack
constexpr int DMA_BUF_LEN = HOP_SAMPLES; // in samples (frames), 32-bit slots

// --- Debug output over USB serial ---
// 1: print the features as CSV at ~20 Hz (same columns as tools/analyze_wav plus a header line). 0: silent.
#ifndef BEATSENSE_DEBUG_CSV
#define BEATSENSE_DEBUG_CSV 1
#endif
constexpr uint32_t DEBUG_PERIOD_MS = 50;

// Task layout: audio on core 1 (Arduino loop() runs there too but is idle), high priority so DMA is never starved
constexpr int AUDIO_TASK_STACK = 8192;
constexpr int AUDIO_TASK_PRIORITY = 5;

#endif
