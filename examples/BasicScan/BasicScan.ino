// Minimal head-module sketch: bring up the bus, print every module the
// master discovers/loses, and periodically print the reading of endpoint 0
// on any module whose CLASS is Plant Container.
//
// SDA/SCL are plain constructor arguments — wire them to whatever GPIOs suit
// the board (ESP32 lets I2C route to nearly any pin via its GPIO matrix).
#include <OgrMainEsp32.h>

using namespace ogr;
using namespace ogr_main;
using namespace ogr_hal_esp32;

constexpr int kSdaPin = 21;
constexpr int kSclPin = 22;

OgrEsp32Transport transport(Wire, kSdaPin, kSclPin);
OgrMaster master(transport);

void onDiscovered(const ModuleInfo &m) {
  Serial.printf("[OGR] discovered addr=0x%02X class=0x%02X epCount=%u\n", m.address,
                static_cast<uint8_t>(m.classId), m.epCount);
}

void onProblem(const ModuleInfo &m) { Serial.printf("[OGR] PROBLEM addr=0x%02X\n", m.address); }

void onRecovered(const ModuleInfo &m) { Serial.printf("[OGR] recovered addr=0x%02X\n", m.address); }

void onRemoved(const ModuleInfo &m) { Serial.printf("[OGR] removed addr=0x%02X\n", m.address); }

void setup() {
  Serial.begin(115200);

  master.onModuleDiscovered(onDiscovered);
  master.onModuleProblem(onProblem);
  master.onModuleRecovered(onRecovered);
  master.onModuleRemoved(onRemoved);

  master.begin();
}

void loop() {
  master.update(millis());

  static uint32_t lastPrint = 0;
  if (millis() - lastPrint > 2000) {
    lastPrint = millis();
    for (uint8_t i = 0; i < master.moduleCount(); ++i) {
      const ModuleInfo *m = master.moduleAt(i);
      if (m->classId != ClassId::PlantContainer) continue;

      master.selectEndpoint(m->address, 0);
      uint32_t value = 0;
      if (master.readEndpointData(m->address, value)) {
        Serial.printf("[OGR] addr=0x%02X ep0=%lu\n", m->address, static_cast<unsigned long>(value));
      }
    }
  }
}
