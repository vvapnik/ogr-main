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
  OgrEsp32Transport(TwoWire &wire, int sdaPin, int sclPin);

  void begin() override;
  ogr_main::BusResult write(uint8_t address, const uint8_t *payload, uint8_t len) override;
  ogr_main::BusResult writeRead(uint8_t address, const uint8_t *writePayload, uint8_t writeLen, uint8_t *readBuf,
                                 uint8_t readLen, uint8_t &outLen) override;

private:
  static ogr_main::BusResult mapEndTransmissionResult(uint8_t code);

  TwoWire &wire_;
  int sdaPin_;
  int sclPin_;
};

} // namespace ogr_hal_esp32
