// SPDX-License-Identifier: EUPL-1.2
#include <unity.h>
#include <string.h>
#include <beatsense/analyzer.h>

using namespace beatsense;

void setUp() {}
void tearDown() {}

static proto::Frame sample() {
  proto::Frame f;
  f.status = proto::kStatusSignal | proto::kStatusLocked;
  f.bpmQ88 = (uint16_t)(127.5 * 256);
  f.beatPhase = 0xABCD;
  f.beatCount = 250;
  f.beatInBar = 3;
  f.confidence = 200;
  f.level[0] = 1; f.level[1] = 2; f.level[2] = 3;
  f.onset[0] = 4; f.onset[1] = 5; f.onset[2] = 6;
  f.energy = 140;
  return f;
}

void test_frame_round_trip() {
  const proto::Frame f = sample();
  uint8_t buf[proto::kFrameSize];
  proto::encodeFrame(f, buf);
  proto::Frame g;
  TEST_ASSERT_TRUE(proto::decodeFrame(buf, g));
  TEST_ASSERT_EQUAL_MEMORY(&f, &g, sizeof f);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 127.5f, g.bpm());
}

void test_frame_layout_matches_documentation() {
  uint8_t b[proto::kFrameSize];
  proto::encodeFrame(sample(), b);
  TEST_ASSERT_EQUAL_UINT(17, proto::kFrameSize);
  TEST_ASSERT_EQUAL_UINT8(1, b[0]);
  TEST_ASSERT_EQUAL_UINT8(3, b[1]);
  TEST_ASSERT_EQUAL_UINT8(0x80, b[2]); // 127.5 * 256 = 0x7F80, little-endian
  TEST_ASSERT_EQUAL_UINT8(0x7F, b[3]);
  TEST_ASSERT_EQUAL_UINT8(0xCD, b[4]);
  TEST_ASSERT_EQUAL_UINT8(0xAB, b[5]);
  TEST_ASSERT_EQUAL_UINT8(250, b[6]);
  TEST_ASSERT_EQUAL_UINT8(3, b[7]);
  TEST_ASSERT_EQUAL_UINT8(200, b[8]);
  TEST_ASSERT_EQUAL_UINT8(1, b[9]);
  TEST_ASSERT_EQUAL_UINT8(4, b[12]);
  TEST_ASSERT_EQUAL_UINT8(140, b[15]);
  TEST_ASSERT_EQUAL_UINT8(proto::crc8(b, 16), b[16]);
}

void test_crc8_known_value() {
  // CRC-8 (poly 0x07, init 0) of "123456789" is 0xF4
  TEST_ASSERT_EQUAL_HEX8(0xF4, proto::crc8((const uint8_t *)"123456789", 9));
}

void test_crc_catches_every_single_bit_flip() {
  uint8_t buf[proto::kFrameSize];
  proto::encodeFrame(sample(), buf);
  for (size_t byte = 0; byte < proto::kFrameSize; byte++) {
    for (int bit = 0; bit < 8; bit++) {
      uint8_t bad[proto::kFrameSize];
      memcpy(bad, buf, sizeof bad);
      bad[byte] ^= (uint8_t)(1 << bit);
      proto::Frame g;
      TEST_ASSERT_FALSE(proto::decodeFrame(bad, g));
    }
  }
}

void test_absent_board_is_rejected() {
  uint8_t ff[proto::kFrameSize], zero[proto::kFrameSize] = {0};
  memset(ff, 0xFF, sizeof ff); // floating bus with pull-ups
  proto::Frame g;
  TEST_ASSERT_FALSE(proto::decodeFrame(ff, g));
  TEST_ASSERT_FALSE(proto::decodeFrame(zero, g));
}

void test_write_commands() {
  uint8_t w[2];
  proto::WriteCommand c;
  proto::encodeWrite(proto::kRegNoiseGate, 60, w);
  TEST_ASSERT_TRUE(proto::decodeWrite(w, 2, c));
  TEST_ASSERT_EQUAL_UINT8(proto::kRegNoiseGate, c.reg);
  TEST_ASSERT_EQUAL_UINT8(60, c.value);
  const uint8_t tap[] = {proto::kRegTap};
  TEST_ASSERT_TRUE(proto::decodeWrite(tap, 1, c));
  TEST_ASSERT_EQUAL_UINT8(proto::kRegTap, c.reg);
  const uint8_t shortWrite[] = {proto::kRegAgcMode};
  TEST_ASSERT_FALSE(proto::decodeWrite(shortWrite, 1, c));
  const uint8_t unknown[] = {0x77, 1};
  TEST_ASSERT_FALSE(proto::decodeWrite(unknown, 2, c));
  TEST_ASSERT_FALSE(proto::decodeWrite(w, 0, c));
}

void test_features_to_frame() {
  Features f;
  f.bpm = 127.5f;
  f.beatPhase = 1234;
  f.status = proto::kStatusClipping;
  f.levels[2] = 9;
  const proto::Frame fr = toFrame(f);
  TEST_ASSERT_EQUAL_UINT16(0x7F80, fr.bpmQ88);
  TEST_ASSERT_EQUAL_UINT16(1234, fr.beatPhase);
  TEST_ASSERT_EQUAL_UINT8(proto::kStatusClipping, fr.status);
  TEST_ASSERT_EQUAL_UINT8(9, fr.level[2]);
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_frame_round_trip);
  RUN_TEST(test_frame_layout_matches_documentation);
  RUN_TEST(test_crc8_known_value);
  RUN_TEST(test_crc_catches_every_single_bit_flip);
  RUN_TEST(test_absent_board_is_rejected);
  RUN_TEST(test_write_commands);
  RUN_TEST(test_features_to_frame);
  return UNITY_END();
}
