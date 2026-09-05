# Arduino / ESP32 Projects

This repository contains the Arduino and embedded projects from my personal
Arduino folder, separated from general C++ language learning.

## Included project areas

- Robot cars, 2WD/4WD movement, motor drivers, and servo control
- Bluetooth-controlled robot experiments
- LAFVIN robot-arm smart-car lessons
- Line tracking, ultrasonic sensing, infrared remote control, and obstacle
  avoidance
- Smart-farm and aquaponics prototypes
- Medicine dispenser and sensor experiments
- SAR robot controller versions using serial communication and I2C
- PlatformIO LED and ultrasonic experiments

The source inventory includes 55 Arduino sketches plus the small local headers,
implementation files, and handoff notes needed to understand selected projects.
Installed Arduino libraries, build folders, virtual environments, APKs,
archives, videos, and other generated/vendor files are intentionally excluded.

## Folder layout

All imported sketches are under [`projects/`](projects/). The original folder
names are preserved so the history of the experiments remains visible.

## What this documents

- Arduino `setup()` / `loop()` structure
- Digital I/O, PWM, analog input, and serial communication
- Motor drivers, servos, Bluetooth, RFID, and ultrasonic sensors
- Robot navigation, line tracking, obstacle avoidance, and following behavior
- Embedded state machines and multi-board robot communication
- Sensor-driven agriculture and aquaponics control
- PlatformIO project organization and embedded C++ configuration

## Security note

The separate `Documents\FingerprintEnrollmentSystem.ino` file was not copied:
it contains Wi-Fi/Firebase credentials and personal/device data. It must be
sanitized and have credentials rotated before any public upload. Authentication
experiments included here contain code structures only; no network credentials
are included.
