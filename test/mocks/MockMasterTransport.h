// A tiny bus simulator: enough real node behavior (SET_ADDR, RESTART,
// PROT_VER/STATUS/CLASS/EP_COUNT/SYS_FLAGS reads, correct PEC) to exercise
// OgrMaster's enumeration/scanning/retry logic without any hardware.
#pragma once

#include <vector>
#include "OgrCrc8.h"
#include "OgrMasterTransport.h"
#include "OgrTypes.h"

namespace ogr_test {

namespace {
constexpr uint8_t regByte(ogr::Reg r) { return static_cast<uint8_t>(r); }

uint8_t computeWritePec(uint8_t address, uint8_t reg, const uint8_t *data, uint8_t len) {
  ogr::Crc8Builder b;
  b.add(static_cast<uint8_t>(address << 1));
  b.add(reg);
  b.add(data, len);
  return b.value();
}

uint8_t computeReadPec(uint8_t address, uint8_t reg, const uint8_t *data, uint8_t len) {
  ogr::Crc8Builder b;
  b.add(static_cast<uint8_t>(address << 1));
  b.add(reg);
  b.add(static_cast<uint8_t>((address << 1) | 1));
  b.add(data, len);
  return b.value();
}
} // namespace

struct VirtualNode {
  bool atDefaultAddress = false; // sitting UNASSIGNED, answering 0x08
  bool registered = false;
  uint8_t address = 0;
  ogr::ClassId classId = ogr::ClassId::Custom;
  uint8_t epCount = 0;
  uint8_t sysFlags = 0;
  uint8_t status = 0;
  bool alive = true; // false: never answers anything (fully absent/hung)

  uint8_t epSelect = 0;
  uint32_t epDataValue = 0;

  // Test-only: this mock has no notion of physical stacking order, so it
  // can't derive "restarting the module below X cascades X's EN_IN low" on
  // its own (spec §3.4). A test that needs that effect sets this to the
  // address whose RESTART should silence this node, simulating exactly the
  // physical relationship it wants to exercise.
  int silenceOnRestartOfAddress = -1;

  // Test knobs to force failure modes the health-check/retry logic reacts to.
  uint8_t injectCommFailures = 0; // next N writeRead()s return BusResult::Error instead of a real reply
};

class MockMasterTransport : public ogr_main::IOgrMasterTransport {
public:
  void begin() override { beginCalls++; }

  ogr_main::BusResult write(uint8_t address, const uint8_t *payload, uint8_t len) override {
    VirtualNode *n = findAnswering(address);
    if (!n) {
      return ogr_main::BusResult::AddressNack;
    }
    if (len < 2) {
      return ogr_main::BusResult::Error;
    }
    uint8_t reg = payload[0];
    uint8_t dataLen = static_cast<uint8_t>(len - 2);
    uint8_t pec = payload[len - 1];
    if (pec != computeWritePec(address, reg, payload + 1, dataLen)) {
      // Spec §5.3 (revised): a node ACKs regardless and just drops a
      // bad-PEC write. From the bus's point of view this still looks like
      // BusResult::Ok — the master never gets in-band notice.
      return ogr_main::BusResult::Ok;
    }
    applyWrite(*n, reg, payload + 1, dataLen);

    if (reg == regByte(ogr::Reg::Restart)) {
      restartedAddresses.push_back(address);
      for (auto &other : nodes) {
        if (other.silenceOnRestartOfAddress == static_cast<int>(address)) {
          other.registered = false;
          other.atDefaultAddress = false;
        }
      }
    }
    return ogr_main::BusResult::Ok;
  }

  ogr_main::BusResult writeRead(uint8_t address, const uint8_t *writePayload, uint8_t /*writeLen*/, uint8_t *readBuf,
                                 uint8_t readLen, uint8_t &outLen) override {
    outLen = 0;
    VirtualNode *n = findAnswering(address);
    if (!n) {
      return ogr_main::BusResult::AddressNack;
    }
    if (n->injectCommFailures > 0) {
      n->injectCommFailures--;
      return ogr_main::BusResult::Error;
    }

    uint8_t reg = writePayload[0];
    uint8_t data[4] = {0, 0, 0, 0};
    uint8_t dataLen = registerReadWidth(reg);
    fillReadData(*n, reg, data);
    uint8_t pec = computeReadPec(address, reg, data, dataLen);

    for (uint8_t i = 0; i < dataLen && outLen < readLen; ++i) {
      readBuf[outLen++] = data[i];
    }
    if (outLen < readLen) {
      readBuf[outLen++] = pec;
    }
    return ogr_main::BusResult::Ok;
  }

  std::vector<VirtualNode> nodes;
  std::vector<uint8_t> restartedAddresses;
  int beginCalls = 0;

private:
  VirtualNode *findAnswering(uint8_t address) {
    for (auto &n : nodes) {
      if (!n.alive) {
        continue;
      }
      if (n.registered && n.address == address) {
        return &n;
      }
      if (!n.registered && n.atDefaultAddress && address == ogr::kDefaultAddress) {
        return &n;
      }
    }
    return nullptr;
  }

  static void applyWrite(VirtualNode &n, uint8_t reg, const uint8_t *data, uint8_t len) {
    if (reg == regByte(ogr::Reg::SetAddr) && len == 1 && !n.registered) {
      n.registered = true;
      n.address = data[0];
      n.atDefaultAddress = false;
    } else if (reg == regByte(ogr::Reg::Restart) && len == 1) {
      n.registered = false;
      n.address = 0;
      // A real module's EN_OUT drop cascades neighbors; tests drive that
      // explicitly by setting atDefaultAddress on the right node instead of
      // simulating EN wiring here.
      n.atDefaultAddress = false;
    } else if (reg == regByte(ogr::Reg::EpSelect) && len == 1) {
      n.epSelect = data[0];
    } else if (reg == regByte(ogr::Reg::EpData) && len == ogr::kEpDataWidth) {
      n.epDataValue = static_cast<uint32_t>(data[0]) | (static_cast<uint32_t>(data[1]) << 8) |
                       (static_cast<uint32_t>(data[2]) << 16) | (static_cast<uint32_t>(data[3]) << 24);
    }
  }

  static uint8_t registerReadWidth(uint8_t reg) {
    if (reg == regByte(ogr::Reg::EpDescriptor)) {
      return ogr::kEpDescriptorWidth;
    }
    if (reg == regByte(ogr::Reg::EpData)) {
      return ogr::kEpDataWidth;
    }
    return 1;
  }

  static void fillReadData(const VirtualNode &n, uint8_t reg, uint8_t *out) {
    if (reg == regByte(ogr::Reg::ProtVer)) {
      out[0] = ogr::kProtocolVersion;
    } else if (reg == regByte(ogr::Reg::Status)) {
      out[0] = n.status;
    } else if (reg == regByte(ogr::Reg::Class)) {
      out[0] = static_cast<uint8_t>(n.classId);
    } else if (reg == regByte(ogr::Reg::EpCount)) {
      out[0] = n.epCount;
    } else if (reg == regByte(ogr::Reg::SysFlags)) {
      out[0] = n.sysFlags;
    } else if (reg == regByte(ogr::Reg::EpSelect)) {
      out[0] = n.epSelect;
    } else if (reg == regByte(ogr::Reg::EpData)) {
      out[0] = static_cast<uint8_t>(n.epDataValue);
      out[1] = static_cast<uint8_t>(n.epDataValue >> 8);
      out[2] = static_cast<uint8_t>(n.epDataValue >> 16);
      out[3] = static_cast<uint8_t>(n.epDataValue >> 24);
    } else {
      out[0] = 0;
    }
  }
};

} // namespace ogr_test
