# AGENTS.md

Guidance for AI agents working in this repo.

Project map: `ARCHITECTURE.takesix.json` — read it first. It shows how the
master engine (`src/OgrMaster.*`), the module table, the bus transport
interface and its ESP32 implementation, the protocol constants and the
host tests fit together, with file paths for each part.

## What this is

Head-module (I2C master) library for OpenGardenRack. The protocol itself
is specified in
[open-garden-rack](https://github.com/vvapnik/open-garden-rack); this
library follows it. The engine talks to the bus only through
`IOgrMasterTransport`; `src/hal/OgrEsp32Transport.*` is the ESP32
implementation on Arduino `Wire`.

## Rules

- No dynamic allocation; the module table is a fixed array.
- `update()` must stay non-blocking: at most one bus transaction's worth
  of work per call, waits are paced against `nowMs`.
- Cover engine changes with host tests: `pio test -e native`.
- When files or responsibilities move, update `ARCHITECTURE.takesix.json`
  so its paths still match the code.

## AI assistance

This project is developed with the help of Claude Code (Anthropic).
