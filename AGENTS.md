# AGENTS.md

Repo-specific guidance for AI agents. Read this before editing firmware or the web GUI.

## What this repo is

ESP32 robot control: single-file web GUI (`pca9685_servo_controller.html` — HTML + CSS + inline JS) that talks to PCA9685 servo driver boards over WebSerial (or ESP-NOW via a controller board), plus generated Arduino firmware copies (`robot/`, `controller/`, root `pca_webcontrol.ino`). Web app instructions: see `AGENT.md`.

## Verify before assuming

No build system, package manager, tests, lint, or CI exist here. Nothing to compile or run automatically.

When editing the web GUI, note it is a single HTML file with an inline `<script>` — validate JS edits by extracting the script block and syntax-checking it (e.g. `node --check`).

## Architecture — easy to get wrong

- The web app source of truth is the single file `pca9685_servo_controller.html` (it contains the firmware generator: `robotFirmware()` / `controllerFirmware()` emitting the Arduino sources). The sketches under `robot/` and `controller/` are generated copies — edit the generator and re-sync the copies together, or the app's "Copy Code" overwrites hand edits.
- Board protocol (Serial 115200): robot handles `?`/`hello`, `s <pin> <angle>`, `g`, `m 1|0`, `o x y z`; streams `HELLO ...`, `Q <r|c> w x y z`, `M <r|c> x y z`.
- Two-board mode: WebSerial -> controller ESP32 (mode 1) -> ESP-NOW -> robot ESP32 (mode 0). Firmware config shape lives in `configs/*.json` (`servomin`, `servomax`, `servos[{pin,label,min,max}]`, `esp{...}`, `fw{...}`); web detects firmware via baked FNV-1a config id.

## Hardware gotchas

- Servos need an external PSU on the PCA9685 V+ terminal; do not power servos from the MCU 5V rail.
- I2C: PCA9685 0x40, MPU6050 0x68, compass 0x2C; ESP32 SDA/SCL 21/22.

## Conventions

- Leg servo mapping: leg N -> yaw pin N-1, lift pin N+3; commanded 90 = limb horizontal; lift up/down mirrored between leg pairs (legs 1&4 vs 2&3).
- Gait v1 is a static-stable crawl (one leg in swing, three planted); PID/tilt correction planned later via IMU.
