# SAR Robot Test Checklist

## Active Files

- Master Arduino: [SAR_Arduino2_v3.ino](/C:/Users/Windows/Downloads/SAR_Arduino2_v1/SAR_Arduino2_v3/SAR_Arduino2_v3.ino)
- Motor Arduino: [SAR_Arduino1_v3.ino](/C:/Users/Windows/Downloads/SAR_Arduino2_v1/SAR_Arduino1_v3/SAR_Arduino1_v3.ino)
- Android app: [MainActivity.java](/C:/Users/Windows/Downloads/SAR_Arduino2_v1/android-app/app/src/main/java/com/sarrobot/controller/MainActivity.java)

## Pre-Test Setup

- [ ] Upload master code to Arduino 2
- [ ] Upload motor code to Arduino 1
- [ ] Confirm both boards power on
- [ ] Confirm common ground between both Arduinos
- [ ] Confirm I2C wiring is correct
- [ ] Pair phone with `HC-05`
- [ ] Open Android app
- [ ] Keep robot lifted for first motor test
- [ ] Keep stop command / power cutoff ready

## Command Reference

- `F` forward
- `B` backward
- `L` left
- `R` right
- `S` stop
- `1` slow speed
- `2` medium speed
- `3` fast speed
- `<` servo left
- `>` servo right
- `C` servo center
- `A` toggle autonomous
- `E` toggle hazard
- `V` one telemetry line
- `P` live telemetry on/off
- `T` sensor test
- `I` I2C scan
- `K` MQ2 calibration
- `~` reset runtime state

## 1. Manual Drive Test

- [ ] `F` moves forward
- [ ] `B` moves backward
- [ ] `L` turns left
- [ ] `R` turns right
- [ ] `S` stops immediately
- [ ] `1`, `2`, `3` change speed correctly
- [ ] `E` hazard stops movement

Pass notes:

-

## 2. Servo Test

- [ ] `<` moves scan servo left
- [ ] `>` moves scan servo right
- [ ] `C` returns servo to center
- [ ] Servo direction matches expected left/right view

Pass notes:

-

## 3. Sensor Test

- [ ] `T` prints readable sensor values
- [ ] Front ultrasonic changes with obstacle distance
- [ ] Rear ultrasonic changes with obstacle distance
- [ ] PIR1 changes to motion when triggered
- [ ] PIR2 changes to motion when triggered
- [ ] MQ2 gives stable baseline after warmup
- [ ] Flame sensors react when tested
- [ ] Temperature and humidity values appear valid

Pass notes:

-

## 4. Telemetry And App Test

- [ ] App connects to `HC-05`
- [ ] Telemetry updates in app
- [ ] Front and rear distance match real scene
- [ ] PIR badges react correctly
- [ ] Hazard banner appears when expected
- [ ] No stale telemetry warning during live updates
- [ ] Console log shows incoming state lines

Pass notes:

-

## 5. Autonomous Patrol Test

- [ ] Enable auto with `A` or app
- [ ] Robot patrols forward
- [ ] Robot slows/stops near obstacle
- [ ] Robot backs up
- [ ] Robot scans
- [ ] Robot turns
- [ ] Robot resumes patrol

Pass notes:

-

## 6. Motion Confirmation Test

- [ ] Enable auto mode
- [ ] Trigger PIR motion
- [ ] App shows `Motion Detected`
- [ ] `Investigate` starts target approach
- [ ] `Abandon` clears target and resumes patrol
- [ ] Without app, robot waits briefly then continues automatically

Pass notes:

-

## 7. Victim Tracking Test

- [ ] Sustained motion increases victim confidence
- [ ] Low confidence only shows tracking behavior
- [ ] Higher confidence enters pending approval
- [ ] Investigate aligns toward target bearing
- [ ] Robot approaches target
- [ ] Verify stage runs
- [ ] Confirmed victim enters hold mode

Pass notes:

-

## 8. Hazard Test

- [ ] Manual hazard works with `E`
- [ ] Gas rise affects alert state
- [ ] Flame input affects alert state
- [ ] High hazard causes robot retreat/observe behavior in auto
- [ ] Hazard clear returns robot to normal state

Pass notes:

-

## 9. Recovery Test

- [ ] Put robot in clutter / corner
- [ ] Auto mode does not loop forever in same simple pattern
- [ ] Repeated obstacle failures escalate recovery
- [ ] App can show `RECOVERY MODE`
- [ ] Robot eventually escapes or enters search mode

Pass notes:

-

## 10. Search Pattern Test

- [ ] Search mode activates when path is unclear
- [ ] Search does not blindly alternate forever
- [ ] Search direction follows remembered obstacle bias
- [ ] Search forward move only happens when front path is clear enough

Pass notes:

-

## Fail / Debug Notes

- Issue:

- What happened:

- Expected:

- Command/state at time of failure:

- Sensor values:

- App message or telemetry clue:

## Final Result

- [ ] Manual control passed
- [ ] Sensors passed
- [ ] Telemetry passed
- [ ] Autonomous patrol passed
- [ ] Motion/victim logic passed
- [ ] Hazard logic passed
- [ ] Recovery/search passed

Overall notes:

-
