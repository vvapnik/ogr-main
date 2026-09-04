# ogr-main

Head-module (I2C master) library for [OpenGardenRack](https://github.com/vvapnik/open-garden-rack):
module table, enumeration, the three scanning cycles (health-check,
scan-for-new, full-range), orphan recovery, and register/endpoint access.
Targets ESP32 / ESP32-S3 / ESP32-C3 (Arduino framework) via a Wire-based
transport; the engine itself only depends on a small `IOgrMasterTransport`
interface, so its logic is host-testable without any hardware.

## Usage

```cpp
#include <OgrMainEsp32.h>

using namespace ogr;
using namespace ogr_main;
using namespace ogr_hal_esp32;

OgrEsp32Transport transport(Wire, /*sda=*/21, /*scl=*/22);
OgrMaster master(transport);

void onDiscovered(const ModuleInfo &m) {
  Serial.printf("discovered 0x%02X class=0x%02X\n", m.address, (uint8_t)m.classId);
}

void setup() {
  master.onModuleDiscovered(onDiscovered);
  master.begin();
}

void loop() {
  master.update(millis());
}
```

See [examples/BasicScan](examples/BasicScan) for a complete sketch, including
reading an endpoint's data window.

SDA/SCL and which hardware I2C peripheral to bind to (`Wire` or `Wire1`) are
constructor arguments to `OgrEsp32Transport` — nothing about the pin layout
is baked into the library. Bus speed is fixed at 100 kHz internally (spec
§1 — a hard physical constraint, not a free parameter).

## Design notes

- **No dynamic allocation.** The module table is a fixed-size array
  (`OGR_MAX_MODULES`, default 12 — the spec's own hard per-chain limit).
- **Each scanning cycle does a bounded amount of work per `update()` call**
  (one health-check probe, or one scan-for-new probe, or one full-range-scan
  probe / one orphan-recovery step) — `update()` never blocks for a full
  sweep, so a busy loop() stays responsive even with a full 12-module chain.
- **Reads retry, writes don't.** Spec §5.4 retries apply only to a read's
  PEC mismatch (up to 3 attempts); per the (revised) spec §5.3, a write is
  never confirmed in-band, so `writeSystemByte`/`writeEndpointData` report
  only bus-level delivery. `registerModuleAtDefaultAddress()` explicitly
  reads back `PROT_VER` after `SET_ADDR` before trusting it, exactly per the
  spec's own guidance for a write that matters.
- **Address-phase NACKs are never retried** by the read-retry loop — that's
  a different failure mode entirely (spec §3.2/§3.3: it feeds the
  `addressNackStreak` → `REMOVED` counter in the health-check cycle, not the
  PEC-mismatch retry loop).

### A spec interpretation worth flagging

Spec §3.5 describes orphan recovery as "restart known modules top-down,
starting from the topmost." It doesn't say what to do when there are no
known modules to restart — which is exactly the scenario §3.3 calls out as
the *reason* the full-range scan must run unconditionally on master
startup: a freshly-rebooted master has an **empty** table while the bus
already has fully-addressed modules on it.

`OgrMaster::recoverOrphan()` handles this by falling back to restarting the
orphan directly once it runs out of known modules to cascade through (see
the comment in `src/OgrMaster.cpp`). This never "adopts" the orphan's
address in place — §3.5's actual constraint — it just forces the same
well-defined `RESTART` → `UNASSIGNED` → re-registration path any other
restart takes. Worth confirming this matches intent, or writing up as a
spec clarification/ADR.

### Address allocation range

The spec's full-range *scan* covers 0x01-0x77 (excluding 0x08). This
library's `allocateFreeAddress()` only *assigns* new addresses from
0x09-0x77, leaving 0x01-0x07 unused — the conventional reserved/special
range in general I2C practice. The spec doesn't mandate this narrower
assignment pool; it was a deliberate choice, not a spec requirement.

## Testing

```sh
pio test -e native
```

Host-based tests (`test/test_master`) run the engine against a small bus
simulator (`test/mocks/MockMasterTransport.h`) covering enumeration order,
health-check Ok/Problem/Removed transitions (both the STATUS-content and
comm-failure-threshold triggers), orphan recovery (both the documented
cascade and the empty-table fallback), and endpoint data round-tripping.

The example also builds against the real ESP32 Arduino core for all three
targets:

```sh
pio ci examples/BasicScan/BasicScan.ino --lib="." --board=esp32dev
pio ci examples/BasicScan/BasicScan.ino --lib="." --board=esp32-s3-devkitc-1
pio ci examples/BasicScan/BasicScan.ino --lib="." --board=esp32-c3-devkitm-1
```
