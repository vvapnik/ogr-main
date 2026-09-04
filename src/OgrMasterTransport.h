// I2C-master bus abstraction. The HAL (src/hal/OgrEsp32Transport.*) implements
// this on top of Arduino's Wire; OgrMaster only ever sees this interface, so
// its enumeration/scanning/retry logic is host-testable against a mock.
#pragma once

#include <stdint.h>

namespace ogr_main {

enum class BusResult : uint8_t {
  Ok,           // full transaction completed, every phase ACKed
  AddressNack,  // no device answered the address phase — nothing there
  DataNack,     // device ACKed its address but NACKed a data byte
  Error,        // any other bus-level failure (timeout, arbitration, ...)
};

class IOgrMasterTransport {
public:
  virtual ~IOgrMasterTransport() = default;

  virtual void begin() = 0;

  // [START] [addr+W] [payload_0..payload_len-1] [STOP]
  virtual BusResult write(uint8_t address, const uint8_t *payload, uint8_t len) = 0;

  // [START] [addr+W] [writePayload...] [REPEATED START] [addr+R] -> up to
  // readLen bytes [STOP]. `outLen` receives how many bytes were actually
  // read (may be less than readLen only on a bus-level failure).
  virtual BusResult writeRead(uint8_t address, const uint8_t *writePayload, uint8_t writeLen,
                               uint8_t *readBuf, uint8_t readLen, uint8_t &outLen) = 0;
};

} // namespace ogr_main
