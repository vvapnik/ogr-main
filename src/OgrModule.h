// A tracked module in the master's table — spec §3.2. Only OK/PROBLEM
// entries live here; UNASSIGNED modules aren't "in the table" yet (they're
// still being discovered by the scan-for-new cycle), and REMOVED means the
// entry is deleted outright, not a tag.
#pragma once

#include <stdint.h>
#include "OgrTypes.h"

#ifndef OGR_MAX_MODULES
#define OGR_MAX_MODULES 12 // spec §1: hard per-chain limit
#endif

namespace ogr_main {

enum class ModuleState : uint8_t { Ok, Problem };

struct ModuleInfo {
  uint8_t address = 0;
  ogr::ClassId classId = ogr::ClassId::Custom;
  uint8_t epCount = 0;
  uint8_t sysFlags = 0;
  ModuleState state = ModuleState::Ok;

  // Two independent failure counters — spec §3.3's PROBLEM/REMOVED
  // distinction is exactly "ACKed but something's wrong" (responseFailStreak,
  // STATUS!=0, or exhausted PEC retries) vs "NACK on the address itself"
  // (addressNackStreak). Each resets whenever its own condition doesn't
  // reproduce; they don't share state.
  uint8_t addressNackStreak = 0;
  uint8_t responseFailStreak = 0;
};

} // namespace ogr_main
