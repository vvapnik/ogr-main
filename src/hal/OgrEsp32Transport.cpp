#include "OgrEsp32Transport.h"

namespace ogr_hal_esp32 {

namespace {
constexpr uint32_t kBusFrequencyHz = 100000; // spec §1: Standard Mode, fixed
}

OgrEsp32Transport::OgrEsp32Transport(TwoWire &wire, int sdaPin, int sclPin, int enPin)
    : wire_(wire), sdaPin_(sdaPin), sclPin_(sclPin), enPin_(enPin) {}

void OgrEsp32Transport::begin() {
  if (enPin_ >= 0) {
    pinMode(enPin_, OUTPUT);
    digitalWrite(enPin_, HIGH);
  }
  wire_.begin(sdaPin_, sclPin_, kBusFrequencyHz);
}

ogr_main::BusResult OgrEsp32Transport::mapEndTransmissionResult(uint8_t code) {
  switch (code) {
    case 0:
      return ogr_main::BusResult::Ok;
    case 2:
      return ogr_main::BusResult::AddressNack;
    case 3:
      return ogr_main::BusResult::DataNack;
    default:
      return ogr_main::BusResult::Error;
  }
}

ogr_main::BusResult OgrEsp32Transport::write(uint8_t address, const uint8_t *payload, uint8_t len) {
  wire_.beginTransmission(address);
  wire_.write(payload, len);
  return mapEndTransmissionResult(wire_.endTransmission(true));
}

ogr_main::BusResult OgrEsp32Transport::writeRead(uint8_t address, const uint8_t *writePayload, uint8_t writeLen,
                                                  uint8_t *readBuf, uint8_t readLen, uint8_t &outLen) {
  outLen = 0;

  wire_.beginTransmission(address);
  wire_.write(writePayload, writeLen);
  ogr_main::BusResult writeResult = mapEndTransmissionResult(wire_.endTransmission(false)); // repeated START
  if (writeResult != ogr_main::BusResult::Ok) {
    return writeResult;
  }

  uint8_t got = wire_.requestFrom(static_cast<int>(address), static_cast<int>(readLen));
  if (got == 0) {
    return ogr_main::BusResult::AddressNack; // nothing answered the read-phase address either
  }
  while (wire_.available() && outLen < readLen) {
    readBuf[outLen++] = static_cast<uint8_t>(wire_.read());
  }
  return ogr_main::BusResult::Ok;
}

} // namespace ogr_hal_esp32
