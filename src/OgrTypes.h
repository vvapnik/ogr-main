// OpenGardenRack wire-protocol constants — mirrors docs/04-06-07 of the spec.
// Deliberately duplicated from ogr-node-core rather than shared via a
// dependency: this is a description of the wire format both sides must
// agree on, not shared logic, and ogr-main has no other reason to depend on
// a package that exists to run on the *node* side of the same bus.
// https://github.com/vvapnik/open-garden-rack
#pragma once

#include <stdint.h>

namespace ogr {

constexpr uint8_t kDefaultAddress = 0x08;
constexpr uint8_t kProtocolVersion = 0x10; // v1.0

// Address-space bounds a master must respect when scanning/assigning —
// spec §3.3: "the whole 7-bit address space, minus 0x08 and the reserved
// 0x78-0x7F block".
constexpr uint8_t kReservedBlockStart = 0x78;
constexpr uint8_t kReservedBlockEnd = 0x7F;

enum class Reg : uint8_t {
  ProtVer = 0x00,
  Status = 0x01,
  Class = 0x02,
  EpCount = 0x03,
  SysFlags = 0x04,
  SetAddr = 0x05,
  Restart = 0x06,
  PlantUid0 = 0x07,
  PlantUid1 = 0x08,
  PlantUid2 = 0x09,
  PlantUid3 = 0x0A,
  BpUid0 = 0x0B,
  BpUid1 = 0x0C,
  BpUid2 = 0x0D,
  BpUid3 = 0x0E,
  EpSelect = 0x0F,
  // 0x10-0x1F reserved

  // Capability descriptor window: select 0x20, read exactly 3 bytes (Type,
  // Mode, Format) in one transaction.
  EpDescriptor = 0x20,

  // Data window: select 0x40, read/write exactly 4 bytes in one transaction.
  EpData = 0x40,
};

enum class Status : uint8_t {
  Ok = 0,
  Error = 1,
  Busy = 2,
};

enum class RestartType : uint8_t {
  Soft = 0x01,
  Hard = 0x02,
};

constexpr uint8_t kFlagIsAnchor = 1u << 0;
constexpr uint8_t kFlagHasBlueprint = 1u << 1;

constexpr uint32_t kNoPlantUid = 0xFFFFFFFFu;
constexpr uint32_t kNoBlueprintUid = 0x00000000u;

enum class ClassId : uint8_t {
  PlantContainer = 0x01,
  Light = 0x02,
  WaterSupply = 0x03,
  Climate = 0x04,
  Custom = 0xFF,
};

enum class TypeId : uint8_t {
  Moisture = 0x40,
  Temp = 0x41,
  Humidity = 0x42,
  Light = 0x43,
  Weight = 0x44,
  Distance = 0x45,
  WaterLevel = 0x46,

  Relay = 0x80,
  Dimmer = 0x81,
  WaterValve = 0x82,
  LightSwitch = 0x83,
  WaterPump = 0x84,
  Vibro = 0x85,

  Camera = 0xC0,
};

enum class Mode : uint8_t {
  Read = 0x01,
  Write = 0x02,
  ReadWrite = 0x03,
};

enum class Format : uint8_t {
  Binary = 0x01,
  Percent = 0x05,
  Raw16 = 0x0A,
};

constexpr uint8_t kEpDescriptorWidth = 3;
constexpr uint8_t kEpDataWidth = 4;

// Largest frame either direction ever needs to carry — spec §5.2.
constexpr uint8_t kMaxWriteFrame = 1 + 4 + 1;
constexpr uint8_t kMaxReadFrame = 4 + 1;

} // namespace ogr
