# SAR Robot Function Summary v3

## System Overview

This robot is a two-Arduino search-and-response platform with optional Android and PC vision support.

- Arduino 2 is the master brain.
- Arduino 1 is the motor controller and servo controller.
- The Android app is the main operator console over HC-05 Bluetooth.
- The optional PC vision module can send person-detection events to the Android app.

## Main Hardware Functions

### Arduino 2: Master Brain

Arduino 2 handles:

- Bluetooth command input through HC-05
- Sensor reading and sensor fusion
- autonomous navigation logic
- victim tracking logic
- hazard detection and hazard scoring
- telemetry generation
- status light behavior through NeoPixels
- I2C command output to Arduino 1

### Arduino 1: Motor Controller

Arduino 1 handles:

- left and right drive motors through the L298N driver
- movement directions: forward, backward, left, right, stop
- speed modes and smooth acceleration
- autonomous/manual motor mode switching
- front and rear ultrasonic sweep servo positioning
- hazard stop / resume behavior

## Sensors And What They Do

The robot currently uses:

- Front ultrasonic sensor
  - measures forward obstacle distance
  - used for slowing, stopping, scanning, and autonomous path decisions

- Rear ultrasonic sensor
  - measures rear clearance during backing and retreat

- PIR 1 and PIR 2 motion sensors
  - detect motion on left/right sides
  - contribute to victim confidence and bearing
  - can now trigger a confirmation-first popup flow

- MQ2 gas sensor
  - detects gas/smoke rise against a saved baseline
  - used for warning and hazard scoring

- DHT11 temperature and humidity sensor
  - provides environmental temperature and humidity
  - temperature contributes to hazard scoring

- 3 analog flame sensors
  - left, center, right flame direction estimate
  - used for fire risk scoring and fire bearing

## Operator Control Methods

The robot can be controlled in several ways:

### 1. Manual Bluetooth Control

Available commands include:

- `F`, `B`, `L`, `R`, `S` for movement
- `1`, `2`, `3` for speed modes
- `<`, `>`, `C` for servo scan direction
- `A` to toggle autonomous mode
- `E` to toggle hazard / emergency state
- `P` to toggle live telemetry
- `V` for one telemetry snapshot
- `I` to scan the I2C bus
- `T` to run a sensor test
- `K` to calibrate MQ2 baseline
- `Z` to toggle auto-safety
- `~` to reset runtime state

### 2. Android App Control

The Android app provides:

- HC-05 Bluetooth connection and reconnect support
- movement buttons / game-style drive HUD
- monitor mode with full telemetry
- radar-style distance display
- hazard banner and alert summaries
- editable auto-speed PWM setting
- live console log
- telemetry freshness / stale-data detection
- popup confirmation dialogs for vision and victim/motion events

### 3. PC Vision Bridge

The optional PC vision system can:

- use IP camera or webcam input
- detect people on the PC side
- send `person_detected` or `person_close` events to the Android app
- let the app ask the operator whether to investigate

Important note:

- The PC vision module does not directly change robot firmware behavior.
- It sends events to the Android app, and the app can then send investigate commands to the robot.

## Telemetry And Monitoring

The robot reports a compact telemetry line that includes:

- mode
- front and rear distance
- temperature and humidity
- gas raw value, filtered value, and delta
- flame sensor values and flame direction
- servo angle
- PIR status
- alert tokens
- autonomous state
- hazard state
- auto-state label
- victim confidence
- victim confirmed flag
- victim bearing
- victim approval pending flag
- hazard scores
- configured auto PWM
- sensor offline flags

This means the operator can see both raw readings and interpreted robot state.

## Core Robot Behaviors

### Manual Driving

In manual mode the operator can:

- drive the robot directly
- stop at any time
- change speed modes
- center or move the scanning servo
- manually trigger or clear hazard mode

Manual control overrides autonomous behavior.

### Autonomous Patrol

When autonomous mode is enabled, the robot can:

- patrol forward
- reduce speed when something is near
- stop when an obstacle is too close
- back away when blocked
- scan left, center, and right
- choose a safer turn direction
- continue patrolling after the turn

### Search Pattern Mode

If the front sensor becomes unreliable or no clear path is available, the robot can:

- enter an automatic search pattern
- alternate movement legs
- keep searching until a target or new condition interrupts it

### Victim Tracking

The robot uses PIR motion plus environmental context to build victim confidence.

It can:

- track possible victim direction
- estimate left, center, or right bearing
- raise a pending victim state
- wait for operator approval
- start investigate mode when approved
- confirm a victim when close enough
- hold position on a confirmed victim

### Motion Confirmation Flow

Current behavior in v3:

- PIR motion can trigger a pending target
- the Android app shows a confirmation popup first
- if the operator taps Investigate, the robot starts the investigate mission
- if the operator taps Abandon, the mission is cleared and patrol resumes
- if the app is not connected, the robot waits briefly and then continues the investigate flow automatically

This gives both:

- operator approval when the app is active
- autonomous continuity when no app is available

### Investigate Mode

Once investigate is approved, the robot can:

- align toward the target bearing
- move forward toward the target
- stop and verify at close distance
- confirm or clear the target

### Victim Hold Mode

If a victim is confirmed, the robot can:

- stop movement
- hold on the victim location
- report victim lock in telemetry and app UI
- wait for the operator to resume patrol

### Hazard Detection

The robot continuously evaluates:

- flame intensity and direction
- gas rise over baseline
- temperature rise

It computes hazard risk and can enter hazard response when conditions are serious enough.

### Hazard Response

When a critical hazard is detected, the robot can:

- enter hazard mode
- stop current mission flow
- retreat backward
- turn away based on fire bearing when available
- observe until conditions improve
- return to patrol when safe again

The operator can also trigger hazard mode manually.

## Scenario Summary

### Scenario 1: Normal Remote Driving

What happens:

- operator connects using the Android app or Bluetooth terminal
- robot receives manual drive commands
- sensors still keep updating telemetry
- hazard alerts can still appear

Best use:

- direct navigation
- testing
- controlled demos

### Scenario 2: Autonomous Corridor Patrol

What happens:

- robot patrols forward on its own
- slows near obstacles
- stops, backs up, scans, and turns when blocked
- resumes forward patrol after clearing the obstacle

Best use:

- basic autonomous movement in indoor spaces

### Scenario 3: Motion Seen On Left Or Right

What happens:

- PIR detects motion
- robot marks a possible target
- app shows a motion confirmation popup
- operator may investigate or abandon
- without the app, the robot waits briefly then investigates automatically

Best use:

- human-presence detection with operator confirmation

### Scenario 4: Vision Sees A Person

What happens:

- PC vision sends a person event to the app
- app shows a vision confirmation popup
- operator can investigate, abort, or ignore
- investigate sends a high-level `:EVENT INVESTIGATE LEFT|CENTER|RIGHT`

Best use:

- camera-assisted victim discovery

### Scenario 5: Possible Victim Becomes Confirmed

What happens:

- robot approaches the suspected target
- verify stage checks confidence and approach distance
- robot confirms victim and enters hold mode
- app shows victim hold state

Best use:

- locating and marking a person for rescue response

### Scenario 6: Gas Or Fire Danger

What happens:

- hazard score rises from gas, flame, or heat
- robot can switch into hazard state
- robot retreats and reorients
- operator sees hazard warnings in telemetry and app banner

Best use:

- dangerous environment awareness
- preventing the robot from driving deeper into unsafe zones

### Scenario 7: Sensor Trouble

What happens:

- telemetry reports stale/offline sensor state
- app shows warning badges and stale telemetry indicators
- robot may switch behavior, including search logic, depending on which sensor is unavailable

Best use:

- diagnostics
- troubleshooting before and during operation

## Visual Feedback

The NeoPixel ring is used for status indication. It can visually reflect:

- motion caution
- scanning / turning activity
- hazard states
- general robot status

The Android app also shows visual status through:

- hazard banners
- PIR badges
- alert severity badges
- radar display
- trend graphs
- victim and vision confirmation popups

## Runtime Configuration

The system supports runtime tuning for some values, including:

- auto PWM speed
- distance thresholds
- gas warning and danger thresholds
- temperature thresholds

This means the robot can be retuned for different environments without rewriting the sketch.

## Current Strengths

- works with or without the app in autonomous mode
- combines multiple sensor types instead of relying on only one
- supports manual, semi-autonomous, and autonomous operation
- supports operator confirmation for motion and vision targets
- provides rich telemetry for debugging and field use
- separates brain logic and motor control across two Arduinos

## Current Practical Limits

- PIR-based victim detection is still an inference, not identity recognition
- DHT11 is slow and limited for precision environmental sensing
- MQ2 requires proper baseline calibration for useful readings
- ultrasonic navigation works best in simple indoor environments
- PC vision depends on camera quality, lighting, and network connection to the phone
- the Android app is the main approval interface for interactive investigate prompts

## Best Short Description

This robot can manually drive, patrol autonomously, avoid obstacles, detect motion, estimate possible victims, confirm targets with operator approval, monitor fire/gas/heat hazards, retreat from dangerous conditions, stream live telemetry to an Android app, and optionally use PC-based vision to help find people.
