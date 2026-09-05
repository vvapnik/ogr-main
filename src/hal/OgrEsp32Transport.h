// I2C-master transport for ESP32 / ESP32-S3 / ESP32-C3, on top of Arduino's
// TwoWire. SDA/SCL and which hardware I2C peripheral to use (Wire vs Wire1
// on chips that have two) are all constructor arguments — nothing here is
// pinned to a specific board.
#pragma once

#include <Wire.h>
#include "OgrMasterTransport.h"

namespace ogr_hal_esp32 {

class OgrEsp32Transport : public ogr_main::IOgrMasterTransport {
public:
  // `wire`: Wire or Wire1. Bus speed is fixed at 100 kHz internally (spec
  // §1 — a hard physical constraint, not something a caller should tune).
  //
  // `enPin`: the master's EN output, feeding EN_IN of the bottom module via
  // the board's level converter (spec §3.1 — "The Master asserts HIGH on
  // the EN line feeding the bottom connector of the first module"). Driven
  // HIGH once in begin() and left there; the master never needs to lower
  // its own EN the way a node's EN_OUT cascades. Pass -1 (default) if this
  // board hardwires EN high itself and there's nothing for software to do —
  // but verify that's actually true before relying on it: a floating EN pin
  // can read as an ambiguous, marginal level (found via real hardware
  // testing: an EN pin left as an unconfigured input measured ~3V instead
  // of a clean 5V, which was enough to make node discovery intermittent).
  OgrEsp32Transport(TwoWire &wire, int sdaPin, int sclPin, int enPin = -1);

  void begin() override;
  ogr_main::BusResult write(uint8_t address, const uint8_t *payload, uint8_t len) override;
  ogr_main::BusResult writeRead(uint8_t address, const uint8_t *writePayload, uint8_t writeLen, uint8_t *readBuf,
                                 uint8_t readLen, uint8_t &outLen) override;

private:
  static ogr_main::BusResult mapEndTransmissionResult(uint8_t code);

  TwoWire &wire_;
  int sdaPin_;
  int sclPin_;
  int enPin_;
};

} // namespace ogr_hal_esp32
