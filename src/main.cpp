// SPDX-License-Identifier: EUPL-1.2
// beatsense firmware: INMP441 -> I2S -> Analyzer -> I2C slave frame for the LED controller.
#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <driver/i2s.h>
#include <esp_bt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_timer.h>
#include <beatsense/analyzer.h>
#include "config.h"

static_assert(HOP_SAMPLES == (int)beatsense::Config::kHop, "config.h hop must match the analyzer");

static beatsense::Analyzer analyzer; // ~20 KB, static so it never touches the stack

// Shared between the audio task (writer) and the I2C callbacks (readers/writers); guarded by a spinlock.
static portMUX_TYPE gMux = portMUX_INITIALIZER_UNLOCKED;
static beatsense::proto::Frame gFrame;  // latest frame, as of gFrameTimeUs
static int64_t gFrameTimeUs = 0;        // when the last sample of that frame was captured (task time, after the DMA read returned)
static uint8_t gOnsetHold[3] = {0, 0, 0};
static volatile bool gTapPending = false;
static volatile int gAgcPending = -1;
static volatile int gGatePending = -1;
static volatile int gLatencyPending = 0x7FFF;
static uint8_t gI2cWrite[4];
static volatile uint8_t gI2cWriteLen = 0;

// ---------- I2S ----------
static void i2sInit() {
  i2s_config_t cfg = {};
  cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX);
  cfg.sample_rate = SAMPLE_RATE;
  cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT;
  cfg.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT; // L/R pin low = left slot
  cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  cfg.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  cfg.dma_buf_count = DMA_BUF_COUNT;
  cfg.dma_buf_len = DMA_BUF_LEN;
  cfg.use_apll = false;
  cfg.tx_desc_auto_clear = false;
  cfg.fixed_mclk = 0;
  ESP_ERROR_CHECK(i2s_driver_install(I2S_NUM_0, &cfg, 0, nullptr));
  i2s_pin_config_t pins = {};
  pins.mck_io_num = I2S_PIN_NO_CHANGE;
  pins.bck_io_num = PIN_I2S_SCK;
  pins.ws_io_num = PIN_I2S_WS;
  pins.data_out_num = I2S_PIN_NO_CHANGE;
  pins.data_in_num = PIN_I2S_SD;
  ESP_ERROR_CHECK(i2s_set_pin(I2S_NUM_0, &pins));
  i2s_zero_dma_buffer(I2S_NUM_0);
}

// ---------- I2C slave ----------
// Read: always the 17-byte frame. beatPhase is advanced by the time since the frame was produced, so it refers to "now" at the
// moment of the request. onset fields are the peak since the previous read.
static void onRequest() {
  beatsense::proto::Frame f;
  int64_t t;
  portENTER_CRITICAL(&gMux);
  f = gFrame;
  t = gFrameTimeUs;
  for (int g = 0; g < 3; g++) {
    f.onset[g] = gOnsetHold[g];
    gOnsetHold[g] = 0;
  }
  portEXIT_CRITICAL(&gMux);
  if (f.bpmQ88 > 0) {
    const int64_t dt = esp_timer_get_time() - t;
    const float beats = (float)dt * 1e-6f * (f.bpmQ88 / 256.0f) / 60.0f;
    const uint32_t ph = f.beatPhase + (uint32_t)(beats * 65536.0f);
    f.beatCount = (uint8_t)(f.beatCount + (ph >> 16));
    f.beatInBar = (uint8_t)((f.beatInBar + (ph >> 16)) & 3);
    f.beatPhase = (uint16_t)ph;
  }
  uint8_t buf[beatsense::proto::kFrameSize];
  beatsense::proto::encodeFrame(f, buf);
  Wire.slaveWrite(buf, sizeof buf);
}

// Runs in the I2C task context: only record the command, the audio task applies it (the Analyzer is not thread-safe)
static void onReceive(int len) {
  uint8_t buf[4];
  int n = 0;
  while (Wire.available() && n < (int)sizeof buf) buf[n++] = (uint8_t)Wire.read();
  while (Wire.available()) Wire.read();
  beatsense::proto::WriteCommand cmd;
  if (!beatsense::proto::decodeWrite(buf, n, cmd)) return;
  switch (cmd.reg) {
  case beatsense::proto::kRegAgcMode: gAgcPending = cmd.value; break;
  case beatsense::proto::kRegNoiseGate: gGatePending = cmd.value; break;
  case beatsense::proto::kRegLatencyMs: gLatencyPending = (int8_t)cmd.value; break;
  case beatsense::proto::kRegTap: gTapPending = true; break;
  }
  (void)len;
}

// ---------- Audio task ----------
static void applyPending() {
  if (gAgcPending >= 0) {
    analyzer.setAgcMode((uint8_t)gAgcPending);
    gAgcPending = -1;
  }
  if (gGatePending >= 0) {
    analyzer.setGateDb(gGatePending == 0 ? analyzer.config().gateDb : -(float)gGatePending);
    gGatePending = -1;
  }
  if (gLatencyPending != 0x7FFF) {
    analyzer.setHostLatencyMs((int8_t)gLatencyPending);
    gLatencyPending = 0x7FFF;
  }
  if (gTapPending) {
    analyzer.tap();
    gTapPending = false;
  }
}

static void audioTask(void *) {
  static int32_t samples[HOP_SAMPLES];
  uint32_t lastDebug = 0;
  for (;;) {
    size_t bytes = 0;
    if (i2s_read(I2S_NUM_0, samples, sizeof samples, &bytes, portMAX_DELAY) != ESP_OK || bytes != sizeof samples) continue;
    const int64_t now = esp_timer_get_time();
    applyPending();
    analyzer.process(samples, HOP_SAMPLES);
    const beatsense::Features f = analyzer.features();
    portENTER_CRITICAL(&gMux);
    gFrame = beatsense::toFrame(f);
    gFrameTimeUs = now;
    for (int g = 0; g < 3; g++) {
      if (f.onsets[g] > gOnsetHold[g]) gOnsetHold[g] = f.onsets[g];
    }
    portEXIT_CRITICAL(&gMux);

#if BEATSENSE_DEBUG_CSV
    const uint32_t ms = millis();
    if (ms - lastDebug >= DEBUG_PERIOD_MS) {
      lastDebug = ms;
      Serial.printf("%.3f,%u,%u,%u,%.2f,%u,%u,%u,%u,%u,%u,%u,%u,%u\n", analyzer.frameCount() * 256.0 / SAMPLE_RATE, f.onsets[0], f.onsets[1],
                    f.onsets[2], f.bpm, f.beatPhase, f.beatCount, f.beatInBar, f.confidence, f.levels[0], f.levels[1], f.levels[2], f.energy,
                    f.status);
    }
#else
    (void)lastDebug;
#endif
  }
}

void setup() {
  // Radios off: nothing here needs them and they add noise and current
  WiFi.mode(WIFI_OFF);
  btStop();

#if BEATSENSE_DEBUG_CSV
  Serial.begin(115200);
  Serial.println("time_s,onset_low,onset_mid,onset_high,bpm,beat_phase,beat_count,beat_in_bar,confidence,level_low,level_mid,level_high,energy,status");
#endif

  i2sInit();

  Wire.onReceive(onReceive);
  Wire.onRequest(onRequest);
  Wire.begin((uint8_t)I2C_ADDRESS, PIN_I2C_SDA, PIN_I2C_SCL, I2C_FREQUENCY);

  xTaskCreatePinnedToCore(audioTask, "audio", AUDIO_TASK_STACK, nullptr, AUDIO_TASK_PRIORITY, nullptr, 1);
}

void loop() { vTaskDelay(portMAX_DELAY); }
