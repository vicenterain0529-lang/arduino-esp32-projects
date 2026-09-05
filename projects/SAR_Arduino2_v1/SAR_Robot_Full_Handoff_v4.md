# SAR Disaster Response Robot Full Handoff

Updated: 2026-04-23
Workspace: `C:\Users\Windows\Downloads\SAR_Arduino2_v1`

## Read This First

This project is a two-Arduino search-and-response robot with an Android operator app and an optional PC vision assistant.

If someone only reads the first few pages, these are the most important things to understand:

1. The robot has two separate Arduino Uno boards.
   - Arduino 2 is the master brain.
   - Arduino 1 is the motor and servo controller.

2. The Android app is the main operator console.
   - It connects over Bluetooth.
   - It shows telemetry, alerts, radar-style distance feedback, logs, and investigation prompts.
   - It can switch between manual and autonomous operation.

3. The PC vision system is optional.
   - It does not directly drive the robot motors.
   - It sends person-detection events to the Android app.
   - The app can then ask the operator whether the robot should investigate.

4. The current active robot code is the `v3` pair:
   - `SAR_Arduino2_v3\SAR_Arduino2_v3.ino`
   - `SAR_Arduino1_v3\SAR_Arduino1_v3.ino`

5. The current master firmware now supports live tuning from commands.
   - `THR ...` changes robot thresholds.
   - `TIME ...` changes movement and scan timings.
   - These settings are now stored in EEPROM and survive reboot.

6. The robot is built around layered behavior priority.
   - Manual operator override wins first.
   - Hazard response is next.
   - Obstacle avoidance is next.
   - Victim investigation comes after that.
   - Normal patrol is the lowest priority autonomous behavior.

7. The most common system failure points are:
   - bad I2C wiring between the two Arduinos
   - missing common ground
   - wrong Bluetooth pairing or stale telemetry
   - MQ2 not calibrated
   - PC vision network mismatch or camera source mismatch

## One-Sentence Purpose

This robot is designed to be a small disaster-response search platform that can be manually driven, patrol autonomously, avoid obstacles, react to hazards, detect possible victims, and optionally use PC-based vision to help identify people.

## What The Full System Includes

The full project currently includes these major parts:

### 1. Robot hardware control

- Arduino 2 master brain firmware
- Arduino 1 motor controller firmware
- HC-05 Bluetooth link
- I2C communication between the two Arduinos
- motors, servo sweep, sensors, and status lighting

### 2. Operator control software

- Android app for robot control and monitoring
- telemetry parser
- live logs
- drive HUD and monitoring dashboard
- threshold configuration UI
- victim and vision confirmation prompts

### 3. Optional PC vision support

- Python-based person-detection bridge
- GUI launcher for PC vision
- event bridge from PC to Android app
- lightweight update packaging for sending new PC vision code to another PC

### 4. Documentation and support files

- summaries
- test checklist
- setup guides
- Google Drive deployment/update package for PC vision
- older handoff documents for reference

## Current Active Files

These are the main files a new maintainer should treat as the current live project:

### Robot firmware

- Master brain: `SAR_Arduino2_v3\SAR_Arduino2_v3.ino`
- Motor controller: `SAR_Arduino1_v3\SAR_Arduino1_v3.ino`

### Android app

- Main activity: `android-app\app\src\main\java\com\sarrobot\controller\MainActivity.java`
- Bluetooth layer: `android-app\app\src\main\java\com\sarrobot\controller\BluetoothController.java`
- Telemetry parser: `android-app\app\src\main\java\com\sarrobot\controller\TelemetryParser.java`
- Telemetry model: `android-app\app\src\main\java\com\sarrobot\controller\RobotTelemetry.java`
- Vision bridge server: `android-app\app\src\main\java\com\sarrobot\controller\VisionEventServer.java`

### PC vision

- Vision backend: `pc-vision\phase1_person_event_bridge.py`
- Vision shared helpers: `pc-vision\vision_common.py`
- Vision GUI: `pc-vision\vision_ui.py`
- Vision launcher scripts: `pc-vision\run_vision.ps1`, `pc-vision\run_vision_ui.ps1`

## Workspace Contents Explained

This section explains the important top-level folders and files in plain language.

### `SAR_Arduino2_v3`

This folder contains the active master brain sketch.

Purpose:
- reads sensors
- decides what the robot should do
- talks to the Android app through Bluetooth
- talks to Arduino 1 through I2C

### `SAR_Arduino1_v3`

This folder contains the active motor controller sketch.

Purpose:
- receives motion and servo commands from Arduino 2
- runs the motors through the L298N driver
- manages speed ramps and drive mode changes
- positions the front and rear sweep servos

### `android-app`

This is the Android operator console project.

Purpose:
- connects to the robot over Bluetooth
- shows live telemetry
- sends commands
- displays alerts and investigation prompts
- hosts the local HTTP endpoint used by PC vision events

### `pc-vision`

This is the optional Windows Python vision module.

Purpose:
- opens an IP camera stream or webcam
- runs person detection
- sends `vision` events to the Android app on port `8765`

### `GOOGLE_DRIVE_PC_SETUP`

This is the deployment/update package for moving the PC vision module to another Windows PC.

Purpose:
- full first-time PC setup handoff
- lightweight update package for existing PCs
- updater script
- deployable zip file

### `00_UPLOAD_TO_NEW_PC`

This is another simplified handoff folder for copying `pc-vision` to a new PC.

### `arduinosarv2`

This is a sandbox for the next-generation architecture, not the main production version.

Purpose:
- experimental v2 architecture work
- future direction ideas

### Existing summary and handoff docs

Important reference files:
- `SAR_Robot_Function_Summary_v3.md`
- `SAR_Robot_Test_Checklist.md`
- `SAR_Robot_Project_Handoff.txt`
- `SAR_Robot_Project_Handoff.docx`

These are useful references, but this document is meant to be the clearer current-state handoff.

## System Architecture

At a high level, the robot works like this:

1. The operator uses the Android app or a Bluetooth terminal.
2. Arduino 2 receives commands and reads all main sensors.
3. Arduino 2 decides whether the robot should:
   - obey manual control
   - patrol
   - avoid an obstacle
   - investigate a possible victim
   - retreat from hazard
4. Arduino 2 sends motor and servo commands to Arduino 1 over I2C.
5. Arduino 1 physically drives the motors and sweep servos.
6. Arduino 2 sends telemetry back to the Android app.
7. Optional PC vision can send a person-detection event to the app.
8. The app can ask the operator whether to investigate and then send a high-level event command back to the robot.

### Plain-Language Control Chain

Operator or app command -> Arduino 2 brain -> I2C -> Arduino 1 motor control -> robot movement

Sensor data -> Arduino 2 brain -> telemetry -> Android app display

PC vision event -> Android app -> operator decision -> Arduino 2 investigate command

## Major Responsibilities By Subsystem

### Arduino 2: Master Brain

Arduino 2 is responsible for:

- Bluetooth command input
- line-command parsing
- sensor reading
- risk scoring
- obstacle logic
- autonomous patrol logic
- victim tracking logic
- hazard handling
- telemetry formatting
- NeoPixel status behavior
- sending commands to Arduino 1 over I2C

### Arduino 1: Motor Controller

Arduino 1 is responsible for:

- left and right motor drive
- speed modes
- acceleration smoothing
- turn control
- front sweep servo
- rear sweep servo
- hazard stop/resume handling
- auto/manual motor mode switching

### Android App

The Android app is responsible for:

- Bluetooth connection management
- displaying telemetry and health status
- drive-mode and monitor-mode UI
- radar display and trend graphs
- hazard banner and alerts
- threshold tuning UI
- auto PWM tuning UI
- handling victim prompts
- handling vision prompts
- hosting the vision event server on port `8765`

### PC Vision

The PC vision module is responsible for:

- opening the camera source
- detecting people
- estimating direction and closeness
- sending JSON events to the Android app
- giving an operator-friendly Windows launcher UI

## Hardware Inventory And Purpose

### Core robot electronics

- Arduino Uno R3 for master brain
- Arduino Uno R3 for motor control
- HC-05 Bluetooth module
- L298N motor driver
- front sweep servo
- rear sweep servo
- NeoPixel ring or strip on the master side

### Sensors on Arduino 2

- front ultrasonic sensor
- rear ultrasonic sensor
- two PIR motion sensors
- two IR proximity sensors on `D12` and `D13`
- three analog flame sensors
- MQ2 gas sensor
- DHT11 temperature/humidity sensor

## Master Brain Pin Map

Current active master pin mapping from `SAR_Arduino2_v3.ino`:

- `D2` = HC-05 RX
- `D3` = HC-05 TX
- `D4` = front ultrasonic trigger
- `D5` = DHT11
- `D6` = NeoPixel
- `D7` = PIR 1 right side
- `D8` = PIR 2 left side
- `D9` = rear ultrasonic trigger
- `D10` = rear ultrasonic echo
- `D11` = front ultrasonic echo
- `D12` = IR proximity left
- `D13` = IR proximity right
- `A0` = flame left
- `A1` = flame center
- `A2` = flame right
- `A3` = MQ2 gas
- `A4` = I2C SDA
- `A5` = I2C SCL

## Motor Controller Pin Map

Current active motor controller mapping from `SAR_Arduino1_v3.ino`:

- `D6` = L298N ENA
- `D7` = L298N IN1
- `D8` = L298N IN2
- `D5` = L298N ENB
- `D9` = L298N IN3
- `D10` = L298N IN4
- `D3` = front ultrasonic servo
- `D11` = rear ultrasonic servo
- `D2` = optional e-stop input
- I2C slave address = `8`

## Sensors And What Each One Does

### Front ultrasonic

Purpose:
- front obstacle distance
- patrol slow-down
- stop decision
- scan/turn logic
- victim approach distance check

### Rear ultrasonic

Purpose:
- rear clearance while backing up
- hazard retreat support

### IR proximity left and right

Purpose:
- immediate short-range obstacle detection on the front low area
- faster than waiting for a full scan
- supports quick stop and turn bias

Important note:
- these are digital module outputs
- their sensitivity is mostly adjusted by the module potentiometer, not by a numeric code threshold

### PIR 1 and PIR 2

Purpose:
- left and right side motion detection
- victim confidence building
- victim bearing hint

### Flame sensors

Purpose:
- detect fire signal strength
- estimate fire direction: left, center, or right
- contribute to hazard risk scoring

Important note:
- this build treats the flame module as reversed logic
- lower reading at idle, higher reading when flame is detected

### MQ2 gas sensor

Purpose:
- smoke/gas rise detection
- compares against saved baseline
- contributes to hazard scoring

Important note:
- it needs warmup and calibration
- the baseline is saved in EEPROM

### DHT11

Purpose:
- temperature and humidity reporting
- temperature contributes to hazard logic

Important note:
- DHT11 is intentionally polled slowly because it is not a fast sensor

## Main Robot Functions

### Manual drive

The robot can be driven directly by the operator.

Functions included:
- forward
- backward
- turn left
- turn right
- stop
- speed mode selection
- servo scan positioning

### Autonomous patrol

The robot can patrol on its own.

Functions included:
- drive forward
- slow when nearing an obstacle
- stop and back away when blocked
- scan left, right, and center
- choose a turn direction
- resume patrol after clearing

### Fast obstacle reaction

The current firmware now includes a faster front obstacle path.

Important behavior:
- front ultrasonic and IR proximity are checked on a fast path
- front obstacle confirmation delay in patrol was removed
- threshold crossing can trigger an immediate obstacle response

### Victim tracking

The robot can build a probable victim state using motion and context.

Functions included:
- watch PIR activity
- maintain victim confidence score
- estimate probable direction
- create pending investigate target
- confirm victim after approach

### Investigate behavior

When investigate is triggered, the robot can:

- pause patrol
- orient toward the target
- move closer
- hold and verify
- confirm or clear the target

### Hazard response

The robot continuously evaluates hazard state using:

- gas
- temperature
- flame

When a hazard becomes critical, the robot can:

- stop current objective
- retreat
- turn away
- observe
- report through telemetry and app UI

### Telemetry and status output

The robot produces telemetry for:

- operator monitoring
- app parsing
- debugging
- health-state detection

### Visual status output

The NeoPixel hardware is used for:

- status indication
- activity indication
- caution/hazard feedback

## Behavior Priority Ladder

This is the safest way to understand the robot's decision system:

1. Manual override
2. Hazard handling
3. Obstacle avoidance
4. Victim investigation
5. Search pattern
6. Normal patrol

## Android App Features

The Android app includes:

- classic Bluetooth support for HC-05
- BLE scanning support
- device picker
- connection state display
- radar view
- gauge views
- trend graphs
- front/rear/temperature/gas/humidity display
- PIR badges
- alert severity display
- hazard banner
- raw console log
- recent commands
- drive-mode and monitor-mode UI
- auto PWM editor
- threshold configuration dialog
- telemetry on/off control
- manual vision assist toggle
- sync-on-connect support

### Android app vision support

The Android app includes an internal event server:

- local server port: `8765`
- path used by PC vision: `POST /vision`
- health check: `GET /health`

Purpose:
- receive person-detection events from the PC
- display a banner
- show an operator decision dialog
- send investigate events to the robot if approved

### Android app victim support

The app can also respond to robot-generated victim events:

- show prompts for pending victim states
- allow investigate or abandon flow
- show victim lock and recovery states in the hazard banner

## PC Vision Features

The current PC vision package includes:

- local per-PC config
- support for IP webcam or built-in/USB webcam
- auto controller link discovery
- wired USB option
- GUI launcher
- updater package for sending changes to another PC

### Files in `pc-vision`

- `phase1_person_event_bridge.py`
  - main vision event bridge
- `vision_common.py`
  - shared helpers
- `vision_ui.py`
  - GUI launcher
- `run_vision.ps1`
  - CLI launcher
- `run_vision_ui.ps1`
  - GUI launcher script
- `build_vision_ui.ps1`
  - builds standalone GUI executable
- `installer\SARVisionInstaller.iss`
  - Inno Setup installer script
- `vision_config.json`
  - per-PC settings
- `vision_ui_state.json`
  - UI runtime state
- `yolov8n.pt`
  - model file

## Command Reference

### Basic single-character commands

Movement:
- `F` forward
- `B` backward
- `L` left
- `R` right
- `S` stop

Speed:
- `1` slow
- `2` medium
- `3` fast

Servo:
- `<` left
- `>` right
- `C` center

Mode and safety:
- `A` toggle autonomous
- `E` toggle hazard
- `Z` toggle auto-safety

Debug and maintenance:
- `?` full status
- `V` telemetry snapshot
- `P` toggle live telemetry
- `I` I2C scan
- `T` one-time sensor test
- `K` MQ2 baseline calibration
- `~` reset runtime state and stored config

### Line commands

These are text commands sent over Bluetooth or serial, usually prefixed with `:`.

Examples:
- `:CFG?`
- `:THR?`
- `:TIME?`
- `:MODE AUTO`
- `:EVENT INVESTIGATE LEFT`

### Threshold tuning commands

Current threshold commands:

- `THR?`
- `THR STOP 15`
- `THR WARN 30`
- `THR CLEAR 40`
- `THR GASWARN 400`
- `THR GASDANGER 600`
- `THR TEMPWARN 38`
- `THR TEMPDANGER 50`
- `THR PWM 80`
- `THR SAVE`
- `THR LOAD`
- `THR RESET`

Important note:
- `THR IR` is intentionally handled as hardware-side guidance because the IR modules on `D12/D13` are digital threshold modules with onboard sensitivity adjustment.

### Timing tuning commands

Current timing commands:

- `TIME?`
- `TIME BACKUP 750`
- `TIME TURN 950`
- `TIME BACKMIN 320`
- `TIME TURNMIN 420`
- `TIME BACKSTEP 180`
- `TIME TURNSTEP 180`
- `TIME SCANLEFT 220`
- `TIME SCANRIGHT 300`
- `TIME SCANCENTER 120`
- `TIME MOTIONPAUSE 1200`
- `TIME INVHOLD 1000`
- `TIME INVALIGN 450`
- `TIME INVAPPROACH 1200`
- `TIME SAVE`
- `TIME LOAD`
- `TIME RESET`

### Mode commands

- `:MODE AUTO`
- `:MODE MANUAL`

### Event commands

- `:EVENT INVESTIGATE LEFT`
- `:EVENT INVESTIGATE CENTER`
- `:EVENT INVESTIGATE RIGHT`
- `:EVENT INVESTIGATE B=45 SRC=VISION C=0.82`
- `:EVENT CLEAR`
- `:EVENT HOLD`

## Telemetry Summary

The telemetry currently includes:

- current mode
- front and rear distance
- IR left and right state
- temperature and humidity
- gas raw, filtered, and delta
- flame sensor values
- flame direction
- servo angle
- PIR state
- alert tokens
- autonomous state
- hazard state
- auto-state label
- victim confidence
- victim confirmed flag
- victim bearing
- victim pending flag
- hazard scores
- auto PWM
- offline flags

## Typical Real-World Workflows

### Workflow 1: Manual driving

1. Power both Arduinos.
2. Pair phone with `HC-05`.
3. Open Android app.
4. Connect to the robot.
5. Use the drive controls.
6. Use `S` or hazard control to stop immediately when needed.

### Workflow 2: Autonomous patrol

1. Confirm sensors are reading correctly.
2. Enable autonomous mode from app or command.
3. Robot moves forward.
4. If blocked, it stops, backs up, scans, turns, and resumes.

### Workflow 3: Motion or victim event

1. PIR activity raises victim confidence.
2. App can show confirmation prompt.
3. Operator chooses investigate or abandon.
4. Robot investigates if approved.
5. Robot may confirm victim and hold.

### Workflow 4: Vision-assisted detection

1. Run PC vision.
2. PC sends event to Android app.
3. App displays vision banner or dialog.
4. Operator approves investigate.
5. Robot receives high-level event command and starts investigate flow.

### Workflow 5: Hazard encounter

1. Gas, flame, or heat score rises.
2. Hazard state becomes active.
3. Robot stops current flow.
4. Robot retreats and reorients.
5. App shows hazard alert state.

## Startup And Operating Procedure

### Before power-up

- verify wiring
- verify common ground between both Arduinos
- confirm motor controller is loaded on Arduino 1
- confirm master brain is loaded on Arduino 2
- confirm I2C wiring
- confirm sensors are connected

### Robot startup

1. Power the motor controller side.
2. Power the master brain side.
3. Wait for MQ2 warmup.
4. Let the app connect.
5. Check telemetry freshness.
6. Run a sensor test if needed.

### Recommended first checks

- `T`
- `V`
- `I`
- `THR?`
- `TIME?`

## PC Vision Setup And Update Procedure

### First-time setup on a Windows PC

1. Install Python 3.11 or newer.
2. Copy the `pc-vision` folder or the full setup package.
3. Open PowerShell in the `pc-vision` folder.
4. Run `.\run_vision_ui.ps1` or `.\run_vision.ps1 -Setup`.
5. Configure controller link and camera source.

### Lightweight update for another PC

Use:
- `GOOGLE_DRIVE_PC_SETUP\SAR_PC_Vision_Lightweight_Update.zip`

Purpose:
- send only updated Python/UI code
- avoid copying `.venv`
- avoid copying the large model file
- preserve that PC's config files

Update command on target PC:

```powershell
.\update_existing_pc_vision.ps1 -TargetPath "C:\path\to\existing\pc-vision"
```

## Test And Verification

The main checklist already exists in:

- `SAR_Robot_Test_Checklist.md`

The core validation groups are:

- manual drive
- servo test
- sensor test
- telemetry and app test
- autonomous patrol
- motion confirmation
- victim tracking
- hazard logic
- recovery and search pattern

## Important Maintenance Notes

### I2C reliability

Most mysterious freezes between the two Arduinos are usually I2C-related.

Always verify:
- common ground
- `A4` to `A4`
- `A5` to `A5`
- short wires
- correct slave address `8`

### MQ2 calibration

MQ2 must be calibrated in a stable clean-air condition.

Use:
- `K`

### IR proximity range

The IR proximity sensors on `D12` and `D13` are short-range digital modules.

Important:
- their physical range is mostly hardware-limited
- onboard potentiometer adjustment matters
- code sees them as triggered or not triggered

### Threshold and timing persistence

Threshold and timing values are now stored in EEPROM.

That means:
- changes survive reboot
- `RESET` clears the stored tuning back to defaults

## Known Practical Limits

- PIR motion is not identity recognition
- DHT11 is slow and not highly precise
- MQ2 depends heavily on proper baseline and environment
- ultrasonic navigation works best in simpler indoor spaces
- IR proximity modules are short-range
- PC vision depends on camera quality, lighting, and network stability
- the Android app is the main human decision layer for investigate flow

## Recommended Future Improvements

- add an official wiring diagram image
- add battery, power, and charging documentation
- add a part-number BOM
- add a single operator quick-start sheet
- add saved named tuning profiles like `FAST`, `SAFE`, and `DEMO`
- add formal ACK/heartbeat between both Arduinos
- add compiled APK handoff notes and version tagging
- add a final production enclosure/service guide

## Best Short Summary For A New Teammate

This robot is a two-Arduino rescue platform where Arduino 2 does the thinking, Arduino 1 does the driving, the Android app is the main operator dashboard, and the optional PC vision module helps the app detect people. The robot can be manually controlled, patrol autonomously, avoid obstacles, react to hazards, track possible victims, and now supports live threshold and timing tuning without reflashing.
