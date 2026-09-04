// OpenGardenRack master (head module) engine — platform-independent.
//
// Owns the module table and drives the three scanning cycles from spec §3.3
// (health-check, scan-for-new, full-range) plus enumeration (§3.1) and
// orphan recovery (§3.5). Talks to the bus only through IOgrMasterTransport
// — a HAL package (ogr-main's src/hal/) supplies the ESP32 Wire-based
// implementation; this class never touches hardware directly.
//
// Usage sketch:
//
//   OgrEsp32Transport transport(/*sda=*/21, /*scl=*/22);
//   OgrMaster master(transport);
//
//   void setup() {
//     master.onModuleDiscovered([](const ModuleInfo &m) { ... });
//     master.begin();
//   }
//
//   void loop() {
//     master.update(millis());
//   }
#pragma once

#include <stdint.h>
#include "OgrMasterTransport.h"
#include "OgrModule.h"
#include "OgrTypes.h"

namespace ogr_main {

using ModuleEventCallback = void (*)(const ModuleInfo &module);

class OgrMaster {
public:
  explicit OgrMaster(IOgrMasterTransport &transport);

  // Tunables — override before begin() if the defaults don't fit. Periods
  // are "no faster than"; update() does a bounded amount of work per call
  // and paces itself against these, it never blocks for a full sweep.
  void setHealthCheckPeriodMs(uint32_t ms) { healthCheckPeriodMs_ = ms; }
  void setScanForNewPeriodMs(uint32_t ms) { scanForNewPeriodMs_ = ms; }
  void setFullRangeScanPeriodMs(uint32_t ms) { fullRangeScanPeriodMs_ = ms; }
  void setRemovedNackThreshold(uint8_t k) { removedNackThreshold_ = k; }   // spec §3.3 "K"
  void setProblemFailThreshold(uint8_t n) { problemFailThreshold_ = n; }  // spec §3.3 "N"

  // All optional. Fire synchronously from within update().
  void onModuleDiscovered(ModuleEventCallback cb) { discoveredCb_ = cb; }
  void onModuleProblem(ModuleEventCallback cb) { problemCb_ = cb; }
  void onModuleRecovered(ModuleEventCallback cb) { recoveredCb_ = cb; } // Problem -> Ok
  void onModuleRemoved(ModuleEventCallback cb) { removedCb_ = cb; }

  // Starts the transport and arms an immediate full-range scan — spec §3.3:
  // "must always run once, unconditionally, on master startup".
  void begin();

  // Call every loop() iteration with the current milliseconds timestamp
  // (Arduino: millis()). Does at most one bus transaction's worth of work.
  void update(uint32_t nowMs);

  uint8_t moduleCount() const { return moduleCount_; }
  const ModuleInfo *moduleAt(uint8_t index) const { return index < moduleCount_ ? &modules_[index] : nullptr; }
  const ModuleInfo *findModule(uint8_t address) const;

  // ---- Register-level API -------------------------------------------------
  // Reads retry up to 3x on a PEC mismatch (spec §5.4) and report whether
  // the address even answered. Writes are fire-and-forget at the protocol
  // level (spec §5.3, revised) — they report only bus-level delivery, not
  // whether the node's PEC check accepted them; read back explicitly (or
  // just check STATUS) to confirm a write that matters.
  bool readSystemByte(uint8_t address, ogr::Reg reg, uint8_t &value);
  bool writeSystemByte(uint8_t address, ogr::Reg reg, uint8_t value);

  bool restart(uint8_t address, ogr::RestartType type);

  bool selectEndpoint(uint8_t address, uint8_t epIndex);
  bool readEndpointDescriptor(uint8_t address, ogr::TypeId &type, ogr::Mode &mode, ogr::Format &format);
  bool readEndpointData(uint8_t address, uint32_t &value);
  bool writeEndpointData(uint8_t address, uint32_t value);

  bool readPlantUid(uint8_t address, uint32_t &uid);
  bool writePlantUid(uint8_t address, uint32_t uid);
  bool readBlueprintUid(uint8_t address, uint32_t &uid);

private:
  enum class ReadOutcome : uint8_t { Success, AddressNack, CommFailure };

  ReadOutcome readRegisterRaw(uint8_t address, uint8_t reg, uint8_t *outData, uint8_t dataLen);
  uint8_t writePec(uint8_t address, uint8_t reg, const uint8_t *data, uint8_t len) const;
  uint8_t readPec(uint8_t address, uint8_t reg, const uint8_t *data, uint8_t len) const;

  void tickHealthCheck(uint32_t now);
  void tickScanForNew(uint32_t now);
  void tickFullRangeScan(uint32_t now);

  void healthCheckOne(uint8_t index);
  void probeForNewModule();
  bool registerModuleAtDefaultAddress(ogr::ClassId classId, uint8_t epCount, uint8_t sysFlags);
  uint8_t allocateFreeAddress() const;
  bool isKnownAddress(uint8_t address) const;

  void recoverOrphan(uint8_t orphanAddress); // spec §3.5, synchronous

  ModuleInfo *addModuleSlot();
  void removeModuleAt(uint8_t index);

  IOgrMasterTransport *transport_;

  ModuleInfo modules_[OGR_MAX_MODULES];
  uint8_t moduleCount_ = 0;

  uint32_t healthCheckPeriodMs_ = 50;
  uint32_t scanForNewPeriodMs_ = 200;
  uint32_t fullRangeScanPeriodMs_ = 30000;
  uint8_t removedNackThreshold_ = 3;
  uint8_t problemFailThreshold_ = 3;

  uint32_t lastHealthCheckMs_ = 0;
  uint8_t healthCheckCursor_ = 0;

  uint32_t lastScanForNewMs_ = 0;

  uint32_t lastFullRangeScanMs_ = 0;
  uint8_t fullRangeScanCursor_ = 0x01;
  bool fullRangeSweepPending_ = true; // forces the mandatory first sweep on begin()

  ModuleEventCallback discoveredCb_ = nullptr;
  ModuleEventCallback problemCb_ = nullptr;
  ModuleEventCallback recoveredCb_ = nullptr;
  ModuleEventCallback removedCb_ = nullptr;
};

} // namespace ogr_main
