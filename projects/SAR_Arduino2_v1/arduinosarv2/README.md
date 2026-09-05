# Arduino SAR V2

This folder is the isolated workspace for the next-generation SAR robot control stack.

It is seeded from the current stable codebase, but intended for the integrated emergency-response system rather than day-to-day stable use.

## Included

- `master/SAR_Arduino2_v2.ino`
  - Master brain sandbox
  - Keeps the latest telemetry, radar-angle, and Bluetooth fixes as the starting point
- `motor/SAR_Arduino1_v2.ino`
  - Motor controller sandbox
  - Uses manual speed presets `100 / 120 / 140`
  - Adds a separate autonomous cautious cap of `80`

## Priority Model

1. Manual override always wins.
2. Hazard response retreats from danger and observes from a safer distance.
3. Collision avoidance uses front/rear ultrasonic plus servo-angle scan memory.
4. Investigation pauses patrol when motion or suspicious sensor combinations appear.
5. Patrol/navigation is the default lowest-priority behavior.

## Important V2 Direction

- Manual speed presets stay tuned for operator control.
- Autonomous drive is intentionally slower and capped separately.
- The angular ultrasonic scan should become a small obstacle memory, not just a live snapshot.
- The left / middle / right flame sensors should act like a directional fire vector.
- Hazard mode should eventually become an active response mode, not only a stop state.

## Next Build Targets

- Convert hazard detection into `RETREAT -> OBSERVE -> REPORT`.
- Add scan-memory slots that remember previously blocked angles.
- Fuse flame direction, gas rise, temperature, and motion into one decision layer.
- Add configuration commands later for editing autonomous caution values without reflashing.
