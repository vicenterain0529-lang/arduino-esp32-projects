# Emergency Response Architecture

## Core Behavior Ladder

### 1. Manual Override

- Any operator movement or stop command immediately drops the robot out of autonomous decision making.
- `STOP` and emergency commands must bypass normal command throttling.

### 2. Hazard Response

Trigger examples:

- Confirmed flame direction from left / middle / right flame sensors
- Dangerous gas rise over baseline
- Extreme temperature
- Future dangerous values from additional sensors

Expected response:

1. Stop current pathing.
2. Move away from the likely hazard direction.
3. Reposition to an observe distance instead of charging closer.
4. Re-scan the front sector using the angular ultrasonic servo.
5. Hold a stable observation posture and continue reporting telemetry.

### 3. Collision Avoidance

Inputs:

- Front ultrasonic
- Rear ultrasonic
- Servo-based angular front scan
- Memory of previously blocked headings

Behavior:

- Immediate obstacles force stop/back/turn decisions.
- Scan history should reduce repeated bad turns.
- The robot should prefer headings with both current clearance and better recent history.

## Obstacle Memory Concept

The servo radar should maintain lightweight entries like:

- angle
- measured distance
- timestamp
- confidence / repeat count

Use it for:

- avoiding immediate re-entry into the same blocked angle
- improving turn choice after backing up
- building a short-term local memory even without full SLAM

## Flame Vector Concept

Treat the three flame sensors as a directional hazard vector:

- left dominant: danger likely on left sector
- center dominant: danger ahead
- right dominant: danger likely on right sector

This should not blindly steer toward flame.

For an emergency-response robot, flame direction combines with gas and temperature to decide:

- retreat direction
- observe orientation
- whether to hold position and report

## Investigation Layer

Investigation should activate when:

- motion is detected
- gas is rising but not yet hazardous
- temperature is elevated but not critical
- flame is weak or intermittent

Behavior:

- pause patrol
- slow movement
- scan left / center / right
- classify whether the event is likely hazard, obstacle, or false alarm

## Patrol Layer

- Lowest priority
- Cautious cruise
- In V2, autonomous motion is capped separately from manual speeds
- Default cap currently prepared as `80` in the motor v2 sketch

## Current V2 Seeding Status

Already prepared in code:

- new isolated `arduinosarv2` workspace
- master v2 sketch seeded from current stable brain
- motor v2 sketch seeded from current motor controller
- manual speed presets remain `100 / 120 / 140`
- autonomous speed cap separated and set to `80`

Planned next coding pass:

- implement hazard retreat state machine
- add obstacle-memory data structures and scoring
- fuse flame/gas/temp/motion into a unified priority resolver
- expose editable autonomy settings through commands or app controls
