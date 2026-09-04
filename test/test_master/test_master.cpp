#include <unity.h>
#include "OgrMaster.h"
#include "../mocks/MockMasterTransport.h"

using namespace ogr;
using namespace ogr_main;
using namespace ogr_test;

namespace {

MockMasterTransport transport;

int discoveredCalls = 0;
ModuleInfo lastDiscovered;
int problemCalls = 0;
ModuleInfo lastProblem;
int recoveredCalls = 0;
ModuleInfo lastRecovered;
int removedCalls = 0;
ModuleInfo lastRemoved;

void onDiscovered(const ModuleInfo &m) {
  discoveredCalls++;
  lastDiscovered = m;
}
void onProblem(const ModuleInfo &m) {
  problemCalls++;
  lastProblem = m;
}
void onRecovered(const ModuleInfo &m) {
  recoveredCalls++;
  lastRecovered = m;
}
void onRemoved(const ModuleInfo &m) {
  removedCalls++;
  lastRemoved = m;
}

// Small fixture: a master wired to the mock bus, with short scan periods and
// low thresholds so tests don't need to simulate hundreds of ticks.
OgrMaster makeMaster() {
  OgrMaster m(transport);
  m.setHealthCheckPeriodMs(10);
  m.setScanForNewPeriodMs(10);
  m.setFullRangeScanPeriodMs(100000); // tests trigger the mandatory first sweep via begin(), not the period
  m.setRemovedNackThreshold(3);
  m.setProblemFailThreshold(3);
  m.onModuleDiscovered(onDiscovered);
  m.onModuleProblem(onProblem);
  m.onModuleRecovered(onRecovered);
  m.onModuleRemoved(onRemoved);
  return m;
}

// Runs update() enough times, `stepMs` apart, to guarantee every cycle's
// period has elapsed at least once (used to drain the mandatory first
// full-range sweep, which steps one address at a time).
void pump(OgrMaster &m, uint32_t &now, int ticks, uint32_t stepMs = 15) {
  for (int i = 0; i < ticks; ++i) {
    now += stepMs;
    m.update(now);
  }
}

} // namespace

void setUp() {
  transport.nodes.clear();
  transport.restartedAddresses.clear();
  discoveredCalls = 0;
  problemCalls = 0;
  recoveredCalls = 0;
  removedCalls = 0;
}
void tearDown() {}

// A module sitting at the default address gets discovered, assigned a free
// address, and appended to the table — spec §3.1.
void test_enumeration_assigns_address() {
  VirtualNode n;
  n.atDefaultAddress = true;
  n.classId = ClassId::PlantContainer;
  n.epCount = 1;
  n.sysFlags = ogr::kFlagIsAnchor;
  transport.nodes.push_back(n);

  OgrMaster master = makeMaster();
  uint32_t now = 0;
  master.begin();
  pump(master, now, 200); // drains the mandatory full-range sweep + scan-for-new

  TEST_ASSERT_EQUAL_INT(1, master.moduleCount());
  TEST_ASSERT_EQUAL_INT(1, discoveredCalls);
  const ModuleInfo *m = master.moduleAt(0);
  TEST_ASSERT_NOT_NULL(m);
  TEST_ASSERT_TRUE(m->address != ogr::kDefaultAddress);
  TEST_ASSERT_EQUAL_INT(static_cast<int>(ClassId::PlantContainer), static_cast<int>(m->classId));
  TEST_ASSERT_EQUAL_UINT8(1, m->epCount);
  TEST_ASSERT_EQUAL_UINT8(ogr::kFlagIsAnchor, m->sysFlags);
}

// Two modules registered in sequence (mirroring the bottom-up EN cascade)
// end up in the table in that same order — the table's order *is* physical
// position, spec §3 intro.
void test_enumeration_preserves_registration_order() {
  VirtualNode bottom;
  bottom.atDefaultAddress = true;
  bottom.classId = ClassId::PlantContainer;
  transport.nodes.push_back(bottom);

  OgrMaster master = makeMaster();
  uint32_t now = 0;
  master.begin();
  pump(master, now, 200);
  TEST_ASSERT_EQUAL_INT(1, master.moduleCount());
  uint8_t bottomAddr = master.moduleAt(0)->address;

  // The bottom module's EN_OUT going high (a consequence of its own
  // enumeration) is what lets the next module reach 0x08 — simulated here
  // by putting the second node at the default address only now.
  VirtualNode top;
  top.atDefaultAddress = true;
  top.classId = ClassId::Light;
  transport.nodes.push_back(top);
  pump(master, now, 200);

  TEST_ASSERT_EQUAL_INT(2, master.moduleCount());
  TEST_ASSERT_EQUAL_UINT8(bottomAddr, master.moduleAt(0)->address);
  TEST_ASSERT_EQUAL_INT(static_cast<int>(ClassId::Light), static_cast<int>(master.moduleAt(1)->classId));
}

// STATUS == 0 keeps a healthy module marked Ok across repeated health-checks.
void test_health_check_keeps_ok_module_ok() {
  VirtualNode n;
  n.atDefaultAddress = true;
  n.status = 0;
  transport.nodes.push_back(n);

  OgrMaster master = makeMaster();
  uint32_t now = 0;
  master.begin();
  pump(master, now, 300);

  TEST_ASSERT_EQUAL_INT(1, master.moduleCount());
  TEST_ASSERT_EQUAL_INT(static_cast<int>(ModuleState::Ok), static_cast<int>(master.moduleAt(0)->state));
  TEST_ASSERT_EQUAL_INT(0, problemCalls);
}

// A module reporting STATUS != 0 is marked Problem on the very next
// health-check read — spec §3.3, "ACK but STATUS = Error ... mark PROBLEM".
void test_health_check_marks_problem_on_status_error() {
  VirtualNode n;
  n.atDefaultAddress = true;
  n.status = 0;
  transport.nodes.push_back(n);

  OgrMaster master = makeMaster();
  uint32_t now = 0;
  master.begin();
  pump(master, now, 200); // let it enumerate and settle as Ok

  uint8_t addr = master.moduleAt(0)->address;
  transport.nodes[0].status = static_cast<uint8_t>(ogr::Status::Error);

  pump(master, now, 5); // one more health-check pass is enough

  const ModuleInfo *m = master.findModule(addr);
  TEST_ASSERT_NOT_NULL(m);
  TEST_ASSERT_EQUAL_INT(static_cast<int>(ModuleState::Problem), static_cast<int>(m->state));
  TEST_ASSERT_EQUAL_INT(1, problemCalls);

  // ... and it recovers once STATUS goes back to Ok.
  transport.nodes[0].status = 0;
  pump(master, now, 5);
  m = master.findModule(addr);
  TEST_ASSERT_EQUAL_INT(static_cast<int>(ModuleState::Ok), static_cast<int>(m->state));
  TEST_ASSERT_EQUAL_INT(1, recoveredCalls);
}

// A module that ACKs its address but whose transactions keep failing
// (timeout / exhausted PEC retries) only flips to Problem after N
// consecutive such failures, not on the first one — spec §3.3.
void test_health_check_marks_problem_after_comm_failure_threshold() {
  VirtualNode n;
  n.atDefaultAddress = true;
  transport.nodes.push_back(n);

  OgrMaster master = makeMaster();
  master.setProblemFailThreshold(3);
  uint32_t now = 0;
  master.begin();
  pump(master, now, 200);
  uint8_t addr = master.moduleAt(0)->address;
  size_t idx = 0;
  for (; idx < transport.nodes.size(); ++idx) {
    if (transport.nodes[idx].address == addr) break;
  }

  // Each health-check attempt internally retries up to 3x on failure (§5.4),
  // so N=3 consecutive PROBLEM-worthy attempts consume 3*3=9 raw failures.
  transport.nodes[idx].injectCommFailures = 255;

  pump(master, now, 2); // 2 failed health-check attempts (6 raw failures) — not yet Problem
  TEST_ASSERT_EQUAL_INT(static_cast<int>(ModuleState::Ok), static_cast<int>(master.findModule(addr)->state));

  pump(master, now, 1); // the 3rd attempt (9th raw failure) tips it over
  TEST_ASSERT_EQUAL_INT(static_cast<int>(ModuleState::Problem), static_cast<int>(master.findModule(addr)->state));
  TEST_ASSERT_EQUAL_INT(1, problemCalls);
}

// A module that stops answering its address entirely (K consecutive address
// NACKs) is removed from the table and its address is freed — spec §3.2/§3.3.
void test_health_check_removes_module_after_address_nack_threshold() {
  VirtualNode n;
  n.atDefaultAddress = true;
  transport.nodes.push_back(n);

  OgrMaster master = makeMaster();
  master.setRemovedNackThreshold(3);
  uint32_t now = 0;
  master.begin();
  pump(master, now, 200);
  TEST_ASSERT_EQUAL_INT(1, master.moduleCount());

  transport.nodes[0].alive = false; // physically gone: every phase NACKs

  pump(master, now, 2); // 2 consecutive NACKs — not removed yet
  TEST_ASSERT_EQUAL_INT(1, master.moduleCount());

  pump(master, now, 1); // 3rd consecutive NACK
  TEST_ASSERT_EQUAL_INT(0, master.moduleCount());
  TEST_ASSERT_EQUAL_INT(1, removedCalls);
}

// Full-range scan finds an address nobody registered (an orphan) and
// recovers it by restarting known modules top-down, stopping as soon as the
// orphan falls silent — spec §3.5. Table has two known modules, A (bottom,
// registered first) and C (top, registered second); the orphan is wired
// (via the mock's test-only hook — see MockMasterTransport.h) to fall
// silent specifically when C is restarted, simulating a module physically
// sitting just above C. Recovery must try C before A, and must never touch
// A once C's restart already made the orphan disappear.
void test_orphan_recovery_tries_topmost_known_module_first() {
  VirtualNode a;
  a.atDefaultAddress = true;
  transport.nodes.push_back(a);

  OgrMaster master = makeMaster();
  uint32_t now = 0;
  master.begin();
  pump(master, now, 50);
  TEST_ASSERT_EQUAL_INT(1, master.moduleCount());
  uint8_t addrA = master.moduleAt(0)->address;

  transport.nodes[0].atDefaultAddress = false; // simulate A already stable; next node reaches 0x08 in its place
  VirtualNode c;
  c.atDefaultAddress = true;
  transport.nodes.push_back(c);
  pump(master, now, 50);
  TEST_ASSERT_EQUAL_INT(2, master.moduleCount());
  uint8_t addrC = master.moduleAt(1)->address; // topmost = most recently registered = last in the table

  VirtualNode orphan;
  orphan.registered = true;
  orphan.address = 0x50;
  orphan.silenceOnRestartOfAddress = addrC;
  transport.nodes.push_back(orphan);

  master.setFullRangeScanPeriodMs(0); // force a fresh sweep that will find 0x50
  pump(master, now, 300);

  TEST_ASSERT_FALSE(transport.nodes[2].registered); // orphan fell silent
  TEST_ASSERT_EQUAL_UINT32(1, transport.restartedAddresses.size());
  TEST_ASSERT_EQUAL_UINT8(addrC, transport.restartedAddresses[0]); // C tried first, A never touched
  TEST_ASSERT_EQUAL_UINT8(addrA, master.moduleAt(0)->address);     // A's table entry is untouched
}

// When the master's table is completely empty (the post-reboot scenario
// spec §3.3 calls out) and a full-range scan finds an already-addressed
// module, there's nothing known to cascade-restart — recoverOrphan falls
// back to restarting the orphan directly (§3.4: RESTART == unplug/replug).
void test_orphan_recovery_direct_restart_when_table_empty() {
  VirtualNode orphan;
  orphan.registered = true;
  orphan.address = 0x42;
  transport.nodes.push_back(orphan);

  OgrMaster master = makeMaster();
  uint32_t now = 0;
  master.begin(); // table is empty; mandatory first sweep runs immediately
  pump(master, now, 200);

  TEST_ASSERT_FALSE(transport.nodes[0].registered); // fell back to restarting the orphan itself
  TEST_ASSERT_EQUAL_INT(0, master.moduleCount());   // never "adopted" at its old address
}

// Endpoint data window read/write round-trips a little-endian uint32,
// exercising the same PEC machinery as the system-register primitives.
void test_endpoint_data_round_trip() {
  VirtualNode n;
  n.atDefaultAddress = true;
  n.epCount = 1;
  transport.nodes.push_back(n);

  OgrMaster master = makeMaster();
  uint32_t now = 0;
  master.begin();
  pump(master, now, 200);
  uint8_t addr = master.moduleAt(0)->address;

  TEST_ASSERT_TRUE(master.selectEndpoint(addr, 0));
  TEST_ASSERT_TRUE(master.writeEndpointData(addr, 0x1234ABCD));

  uint32_t value = 0;
  TEST_ASSERT_TRUE(master.readEndpointData(addr, value));
  TEST_ASSERT_EQUAL_HEX32(0x1234ABCD, value);
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_enumeration_assigns_address);
  RUN_TEST(test_enumeration_preserves_registration_order);
  RUN_TEST(test_health_check_keeps_ok_module_ok);
  RUN_TEST(test_health_check_marks_problem_on_status_error);
  RUN_TEST(test_health_check_marks_problem_after_comm_failure_threshold);
  RUN_TEST(test_health_check_removes_module_after_address_nack_threshold);
  RUN_TEST(test_orphan_recovery_tries_topmost_known_module_first);
  RUN_TEST(test_orphan_recovery_direct_restart_when_table_empty);
  RUN_TEST(test_endpoint_data_round_trip);
  return UNITY_END();
}
