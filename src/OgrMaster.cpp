#include "OgrMaster.h"
#include "OgrCrc8.h"

namespace ogr_main {

namespace {
constexpr uint8_t regByte(ogr::Reg r) { return static_cast<uint8_t>(r); }

// Spec §3.4. See the pendingVerification_ member comment in OgrMaster.h.
constexpr uint32_t kSetAddrSettleMs = 50;
} // namespace

OgrMaster::OgrMaster(IOgrMasterTransport &transport) : transport_(&transport) {}

void OgrMaster::begin() {
  transport_->begin();

  moduleCount_ = 0;
  healthCheckCursor_ = 0;
  lastHealthCheckMs_ = 0;
  lastScanForNewMs_ = 0;
  pendingVerification_ = false;

  // Spec §3.3: the full-range scan "must always run once, unconditionally,
  // on master startup" — this is exactly that scenario (table starts empty
  // regardless of what's already addressed on the bus).
  lastFullRangeScanMs_ = 0;
  fullRangeScanCursor_ = 0x01;
  fullRangeSweepPending_ = true;
}

void OgrMaster::update(uint32_t nowMs) {
  tickPendingVerification(nowMs);
  tickFullRangeScan(nowMs);
  tickScanForNew(nowMs);
  tickHealthCheck(nowMs);
}

const ModuleInfo *OgrMaster::findModule(uint8_t address) const {
  for (uint8_t i = 0; i < moduleCount_; ++i) {
    if (modules_[i].address == address) {
      return &modules_[i];
    }
  }
  return nullptr;
}

// ---- PEC -------------------------------------------------------------

uint8_t OgrMaster::writePec(uint8_t address, uint8_t reg, const uint8_t *data, uint8_t len) const {
  ogr::Crc8Builder b;
  b.add(static_cast<uint8_t>(address << 1));
  b.add(reg);
  b.add(data, len);
  return b.value();
}

uint8_t OgrMaster::readPec(uint8_t address, uint8_t reg, const uint8_t *data, uint8_t len) const {
  ogr::Crc8Builder b;
  b.add(static_cast<uint8_t>(address << 1));
  b.add(reg);
  b.add(static_cast<uint8_t>((address << 1) | 1));
  b.add(data, len);
  return b.value();
}

// ---- Register-level primitives ----------------------------------------

OgrMaster::ReadOutcome OgrMaster::readRegisterRaw(uint8_t address, uint8_t reg, uint8_t *outData, uint8_t dataLen) {
  uint8_t expectedTotal = static_cast<uint8_t>(dataLen + 1);

  // Spec §5.4: retries apply to PEC mismatches only, up to 3 attempts total.
  // An address-phase NACK is a different failure mode entirely (§3.2/§3.3 —
  // it feeds the REMOVED/K-consecutive counter in the health-check cycle,
  // not this retry loop) and is never retried here.
  for (uint8_t attempt = 0; attempt < 3; ++attempt) {
    uint8_t raw[ogr::kMaxReadFrame];
    uint8_t got = 0;
    BusResult br = transport_->writeRead(address, &reg, 1, raw, expectedTotal, got);

    if (br == BusResult::AddressNack) {
      return ReadOutcome::AddressNack;
    }
    if (br != BusResult::Ok || got != expectedTotal) {
      continue; // bus hiccup other than address NACK — still counts as an attempt
    }

    uint8_t expectedPecByte = readPec(address, reg, raw, dataLen);
    if (raw[dataLen] == expectedPecByte) {
      for (uint8_t i = 0; i < dataLen; ++i) {
        outData[i] = raw[i];
      }
      return ReadOutcome::Success;
    }
    // PEC mismatch: retry.
  }
  return ReadOutcome::CommFailure;
}

bool OgrMaster::readSystemByte(uint8_t address, ogr::Reg reg, uint8_t &value) {
  return readRegisterRaw(address, regByte(reg), &value, 1) == ReadOutcome::Success;
}

bool OgrMaster::writeSystemByte(uint8_t address, ogr::Reg reg, uint8_t value) {
  uint8_t rb = regByte(reg);
  uint8_t pec = writePec(address, rb, &value, 1);
  uint8_t frame[3] = {rb, value, pec};
  return transport_->write(address, frame, 3) == BusResult::Ok;
}

bool OgrMaster::restart(uint8_t address, ogr::RestartType type) {
  return writeSystemByte(address, ogr::Reg::Restart, static_cast<uint8_t>(type));
}

bool OgrMaster::selectEndpoint(uint8_t address, uint8_t epIndex) {
  return writeSystemByte(address, ogr::Reg::EpSelect, epIndex);
}

bool OgrMaster::readEndpointDescriptor(uint8_t address, ogr::TypeId &type, ogr::Mode &mode, ogr::Format &format) {
  uint8_t data[ogr::kEpDescriptorWidth];
  if (readRegisterRaw(address, regByte(ogr::Reg::EpDescriptor), data, ogr::kEpDescriptorWidth) != ReadOutcome::Success) {
    return false;
  }
  type = static_cast<ogr::TypeId>(data[0]);
  mode = static_cast<ogr::Mode>(data[1]);
  format = static_cast<ogr::Format>(data[2]);
  return true;
}

bool OgrMaster::readEndpointData(uint8_t address, uint32_t &value) {
  uint8_t data[ogr::kEpDataWidth];
  if (readRegisterRaw(address, regByte(ogr::Reg::EpData), data, ogr::kEpDataWidth) != ReadOutcome::Success) {
    return false;
  }
  value = static_cast<uint32_t>(data[0]) | (static_cast<uint32_t>(data[1]) << 8) |
          (static_cast<uint32_t>(data[2]) << 16) | (static_cast<uint32_t>(data[3]) << 24);
  return true;
}

bool OgrMaster::writeEndpointData(uint8_t address, uint32_t value) {
  uint8_t data[4] = {
      static_cast<uint8_t>(value),
      static_cast<uint8_t>(value >> 8),
      static_cast<uint8_t>(value >> 16),
      static_cast<uint8_t>(value >> 24),
  };
  uint8_t rb = regByte(ogr::Reg::EpData);
  uint8_t pec = writePec(address, rb, data, 4);
  uint8_t frame[6] = {rb, data[0], data[1], data[2], data[3], pec};
  return transport_->write(address, frame, 6) == BusResult::Ok;
}

bool OgrMaster::readPlantUid(uint8_t address, uint32_t &uid) {
  uint8_t bytes[4];
  for (uint8_t i = 0; i < 4; ++i) {
    if (readRegisterRaw(address, static_cast<uint8_t>(regByte(ogr::Reg::PlantUid0) + i), &bytes[i], 1) !=
        ReadOutcome::Success) {
      return false;
    }
  }
  uid = static_cast<uint32_t>(bytes[0]) | (static_cast<uint32_t>(bytes[1]) << 8) |
        (static_cast<uint32_t>(bytes[2]) << 16) | (static_cast<uint32_t>(bytes[3]) << 24);
  return true;
}

bool OgrMaster::writePlantUid(uint8_t address, uint32_t uid) {
  bool ok = true;
  for (uint8_t i = 0; i < 4; ++i) {
    ogr::Reg reg = static_cast<ogr::Reg>(regByte(ogr::Reg::PlantUid0) + i);
    ok = writeSystemByte(address, reg, static_cast<uint8_t>(uid >> (i * 8))) && ok;
  }
  return ok;
}

bool OgrMaster::readBlueprintUid(uint8_t address, uint32_t &uid) {
  uint8_t bytes[4];
  for (uint8_t i = 0; i < 4; ++i) {
    if (readRegisterRaw(address, static_cast<uint8_t>(regByte(ogr::Reg::BpUid0) + i), &bytes[i], 1) !=
        ReadOutcome::Success) {
      return false;
    }
  }
  uid = static_cast<uint32_t>(bytes[0]) | (static_cast<uint32_t>(bytes[1]) << 8) |
        (static_cast<uint32_t>(bytes[2]) << 16) | (static_cast<uint32_t>(bytes[3]) << 24);
  return true;
}

// ---- Module table -------------------------------------------------------

ModuleInfo *OgrMaster::addModuleSlot() {
  if (moduleCount_ >= OGR_MAX_MODULES) {
    return nullptr;
  }
  ModuleInfo &slot = modules_[moduleCount_];
  slot = ModuleInfo();
  moduleCount_++;
  return &slot;
}

void OgrMaster::removeModuleAt(uint8_t index) {
  if (index >= moduleCount_) {
    return;
  }
  for (uint8_t i = index; static_cast<uint8_t>(i + 1) < moduleCount_; ++i) {
    modules_[i] = modules_[i + 1];
  }
  moduleCount_--;
}

bool OgrMaster::isKnownAddress(uint8_t address) const { return findModule(address) != nullptr; }

uint8_t OgrMaster::allocateFreeAddress() const {
  if (moduleCount_ >= OGR_MAX_MODULES) {
    return 0;
  }
  // Deliberately starts one above the default address and stays clear of
  // the reserved 0x78-0x7F block (spec §3.3); addresses 0x01-0x07 are left
  // unassigned too, matching common I2C practice for the reserved/special
  // low range, even though the spec's *scan* range doesn't explicitly
  // exclude them (nothing requires the assignment pool to use every
  // scanned address).
  for (uint8_t addr = ogr::kDefaultAddress + 1; addr < ogr::kReservedBlockStart; ++addr) {
    if (!isKnownAddress(addr)) {
      return addr;
    }
  }
  return 0;
}

// ---- Health-check cycle — spec §3.3 --------------------------------------

void OgrMaster::tickHealthCheck(uint32_t now) {
  if (moduleCount_ == 0) {
    return;
  }
  if (now - lastHealthCheckMs_ < healthCheckPeriodMs_) {
    return;
  }
  lastHealthCheckMs_ = now;

  if (healthCheckCursor_ >= moduleCount_) {
    healthCheckCursor_ = 0;
  }
  healthCheckOne(healthCheckCursor_);
}

void OgrMaster::healthCheckOne(uint8_t index) {
  ModuleInfo m = modules_[index]; // survives the array possibly shifting under removeModuleAt()
  uint8_t status = 0;
  ReadOutcome outcome = readRegisterRaw(m.address, regByte(ogr::Reg::Status), &status, 1);

  if (outcome == ReadOutcome::AddressNack) {
    m.addressNackStreak++;
    if (m.addressNackStreak >= removedNackThreshold_) {
      if (removedCb_) {
        removedCb_(m);
      }
      removeModuleAt(index); // cursor stays put: whatever shifted into `index` is checked next time
      return;
    }
    modules_[index] = m;
    healthCheckCursor_ = static_cast<uint8_t>((index + 1) % moduleCount_);
    return;
  }

  if (outcome == ReadOutcome::CommFailure) {
    m.addressNackStreak = 0; // the address itself answered — this isn't a REMOVED-track event
    m.responseFailStreak++;
    if (m.responseFailStreak >= problemFailThreshold_ && m.state != ModuleState::Problem) {
      m.state = ModuleState::Problem;
      if (problemCb_) {
        problemCb_(m);
      }
    }
    modules_[index] = m;
    healthCheckCursor_ = static_cast<uint8_t>((index + 1) % moduleCount_);
    return;
  }

  // Success: address answered and the read completed cleanly.
  m.addressNackStreak = 0;
  m.responseFailStreak = 0;
  ModuleState observed = (status == static_cast<uint8_t>(ogr::Status::Ok)) ? ModuleState::Ok : ModuleState::Problem;
  if (observed != m.state) {
    m.state = observed;
    if (observed == ModuleState::Problem && problemCb_) {
      problemCb_(m);
    } else if (observed == ModuleState::Ok && recoveredCb_) {
      recoveredCb_(m);
    }
  }
  modules_[index] = m;
  healthCheckCursor_ = static_cast<uint8_t>((index + 1) % moduleCount_);
}

// ---- Scan-for-new cycle — spec §3.3 --------------------------------------

void OgrMaster::tickScanForNew(uint32_t now) {
  if (pendingVerification_) {
    return; // one enumeration in flight at a time — see tickPendingVerification()
  }
  if (now - lastScanForNewMs_ < scanForNewPeriodMs_) {
    return;
  }
  lastScanForNewMs_ = now;
  probeForNewModule(now);
}

void OgrMaster::probeForNewModule(uint32_t now) {
  uint8_t classByte;
  bool gotClass = readRegisterRaw(ogr::kDefaultAddress, regByte(ogr::Reg::Class), &classByte, 1) == ReadOutcome::Success;
  trace("read-class-0x08", ogr::kDefaultAddress, gotClass);
  if (!gotClass) {
    return; // nothing at 0x08 (or a transient hiccup) — tried again next period, nothing else to do
  }

  uint8_t epCount = 0;
  readRegisterRaw(ogr::kDefaultAddress, regByte(ogr::Reg::EpCount), &epCount, 1);
  uint8_t sysFlags = 0;
  readRegisterRaw(ogr::kDefaultAddress, regByte(ogr::Reg::SysFlags), &sysFlags, 1);

  beginRegisterModuleAtDefaultAddress(static_cast<ogr::ClassId>(classByte), epCount, sysFlags, now);
}

void OgrMaster::beginRegisterModuleAtDefaultAddress(ogr::ClassId classId, uint8_t epCount, uint8_t sysFlags,
                                                     uint32_t now) {
  uint8_t newAddr = allocateFreeAddress();
  if (newAddr == 0) {
    return; // table full / address pool exhausted
  }

  bool wroteAddr = writeSystemByte(ogr::kDefaultAddress, ogr::Reg::SetAddr, newAddr);
  trace("write-set-addr", newAddr, wroteAddr);
  if (!wroteAddr) {
    return; // bus-level failure; module is presumably still at 0x08, retried next pass
  }

  // Spec §3.4: a node may need a moment to actually start listening on its
  // new address — reprogramming an I2C peripheral is not always the
  // sub-microsecond register write it is on a bare-metal AVR/USI target.
  // Measured on an ESP32-S3 (ESP-IDF's driver/i2c_slave.h, which tears down
  // and recreates the whole slave device on an address change): ~0.9ms
  // worst case. kSetAddrSettleMs gives comfortable margin above that.
  // Non-blocking — tickPendingVerification() picks this back up once
  // kSetAddrSettleMs has actually elapsed, rather than stalling update().
  pendingVerification_ = true;
  pendingAddress_ = newAddr;
  pendingClassId_ = classId;
  pendingEpCount_ = epCount;
  pendingSysFlags_ = sysFlags;
  pendingSetAddrMs_ = now;
}

void OgrMaster::tickPendingVerification(uint32_t now) {
  if (!pendingVerification_) {
    return;
  }
  if (now - pendingSetAddrMs_ < kSetAddrSettleMs) {
    return; // kSetAddrSettleMs hasn't elapsed yet
  }
  pendingVerification_ = false;

  // Spec §5.3 (revised): a write is never confirmed in-band, so a SET_ADDR
  // this critical is worth an explicit read-back before trusting it.
  uint8_t protVer = 0;
  bool verified = readRegisterRaw(pendingAddress_, regByte(ogr::Reg::ProtVer), &protVer, 1) == ReadOutcome::Success;
  trace("read-new-addr", pendingAddress_, verified);
  if (!verified) {
    return; // SET_ADDR didn't take; module stays UNASSIGNED, picked up again next scan-for-new pass
  }

  finishRegisterModule();
}

void OgrMaster::finishRegisterModule() {
  ModuleInfo *slot = addModuleSlot();
  if (!slot) {
    return;
  }
  slot->address = pendingAddress_;
  slot->classId = pendingClassId_;
  slot->epCount = pendingEpCount_;
  slot->sysFlags = pendingSysFlags_;
  slot->state = ModuleState::Ok;

  if (discoveredCb_) {
    discoveredCb_(*slot);
  }
}

// ---- Full-range scan + orphan recovery — spec §3.3/§3.5 ------------------

void OgrMaster::tickFullRangeScan(uint32_t now) {
  if (!fullRangeSweepPending_) {
    if (now - lastFullRangeScanMs_ < fullRangeScanPeriodMs_) {
      return;
    }
    fullRangeSweepPending_ = true;
    fullRangeScanCursor_ = 0x01;
  }

  while (fullRangeScanCursor_ < ogr::kReservedBlockStart &&
         (fullRangeScanCursor_ == ogr::kDefaultAddress || isKnownAddress(fullRangeScanCursor_))) {
    fullRangeScanCursor_++;
  }

  if (fullRangeScanCursor_ >= ogr::kReservedBlockStart) {
    fullRangeSweepPending_ = false; // swept the whole range, nothing orphaned
    lastFullRangeScanMs_ = now;
    return;
  }

  uint8_t addr = fullRangeScanCursor_++;
  uint8_t protVer;
  ReadOutcome outcome = readRegisterRaw(addr, regByte(ogr::Reg::ProtVer), &protVer, 1);
  if (outcome != ReadOutcome::AddressNack) {
    // Something answered an address we don't know — an orphan (§3.3).
    // End this sweep here; recovery below can itself reshape the table
    // significantly, and the next scheduled sweep will catch anything left.
    fullRangeSweepPending_ = false;
    lastFullRangeScanMs_ = now;
    trace("orphan-found", addr, true);
    recoverOrphan(addr);
  }
}

void OgrMaster::recoverOrphan(uint8_t orphanAddress) {
  // Spec §3.5: RESTART known modules from the topmost (= the end of our
  // table, since registration order is physical order) downward, rechecking
  // after each whether the orphan is still there. Stop as soon as it falls
  // silent — that's the signal it cascaded back to UNASSIGNED, and the
  // ordinary scan-for-new cycle re-registers it (and anything else that
  // cascaded) at its correct position from there.
  for (int i = static_cast<int>(moduleCount_) - 1; i >= 0; --i) {
    restart(modules_[static_cast<uint8_t>(i)].address, ogr::RestartType::Soft);

    uint8_t protVer;
    if (readRegisterRaw(orphanAddress, regByte(ogr::Reg::ProtVer), &protVer, 1) == ReadOutcome::AddressNack) {
      return;
    }
  }

  // Reached the bottom of what we know (or the table was empty to begin
  // with — the master-reboot-with-a-stale-table scenario from §3.3) and the
  // orphan is still answering: physically, it must sit at or below
  // everything we have on record. There's no known module left whose
  // restart would cascade to it, so — per §3.4, RESTART is defined as the
  // software equivalent of unplugging/replugging a module — RESTART the
  // orphan directly. This never "adopts" its address (§3.5's actual
  // constraint): it just forces the same well-defined UNASSIGNED path, and
  // the module (and anything stacked on it) rejoins through ordinary
  // registration like any other reset.
  restart(orphanAddress, ogr::RestartType::Soft);
}

} // namespace ogr_main
