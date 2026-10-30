# AGENT.md — PCA9685 Servo Control Studio

Single-file web app (HTML + CSS + JS all inline in one `.html`) that talks to PCA9685 servo driver boards over WebSerial (or ESP-NOW via a controller board), plus the generated Arduino firmware.

## Run

```
cd /home/kaamaru/Arduino/pca_webcontrol
python3 -m http.server 8000
```

Then open <http://localhost:8000/pca9685_servo_controller.html>.

WebSerial requires a secure context, so use localhost (or https). Frontend-only project: no build step, no npm, no backend.

## File map (repo root)

- `pca9685_servo_controller.html` — the whole app: markup, `<style>`, and one inline `<script>` (helpers, state, UI render/actions, WebSerial protocol handling, sensor fusion, calibration, pose/sequence storage, drag & drop, `robotFirmware()` / `controllerFirmware()` code generators, copy-to-clipboard).
- `robot/robot.ino`, `controller/controller.ino`, `pca_webcontrol.ino` — generated Arduino firmware copies from the in-file generators; re-flash after changing the generator or the config.
- `configs/*.json` — firmware config (`servomin`, `servomax`, `servos[{pin,label,min,max}]`, `esp{...}`, `fw{...}`).

## Notes

- Single file, plain inline `<script>` (no ES modules, no bundler, no npm). Inline `onclick`/`onchange` handlers are used and must stay valid global function names.
- Keep firmware source of truth in the generator functions inside the HTML file; the Arduino sketches are generated copies — edit the generator and re-sync, or the "Copy Code" button overwrites hand edits.

## Hardware

- Servos need an external PSU on the PCA9685 V+ terminal; do not power servos from the MCU 5V rail.
- I2C: PCA9685 0x40, MPU6050 0x68, compass 0x2C; ESP32 SDA/SCL 21/22.
- Leg servo mapping: leg N -> yaw pin N-1, lift pin N+3; commanded 90 = limb horizontal; lift up/down mirrored between leg pairs (legs 1&4 vs 2&3).
- Gait v1 is a static-stable crawl (one leg in swing, three planted); PID/tilt correction planned later via IMU.
