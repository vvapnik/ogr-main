#include <unity.h>
#include "OgrCrc8.h"

using ogr::Crc8;
using ogr::Crc8Builder;

void setUp() {}
void tearDown() {}

// Standard CRC-8/SMBUS check value for ASCII "123456789".
void test_check_vector() {
  const uint8_t input[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  Crc8Builder b;
  b.add(input, sizeof(input));
  TEST_ASSERT_EQUAL_HEX8(0xF4, b.value());
}

void test_builder_matches_master_and_node_pec_convention() {
  // A write to REG 0x06 = 0x01 addressed to 0x2A — spec §5.5's worked example.
  Crc8Builder b;
  b.add(0x54); // (0x2A << 1) | W
  b.add(0x06);
  b.add(0x01);
  // No fixed expected value asserted here (that's the node repo's job); this
  // just pins down that add(byte) and add(buffer) compose the same way the
  // master's writePec()/readPec() rely on.
  Crc8Builder viaBuffer;
  uint8_t bytes[] = {0x54, 0x06, 0x01};
  viaBuffer.add(bytes, sizeof(bytes));
  TEST_ASSERT_EQUAL_HEX8(b.value(), viaBuffer.value());
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_check_vector);
  RUN_TEST(test_builder_matches_master_and_node_pec_convention);
  return UNITY_END();
}
