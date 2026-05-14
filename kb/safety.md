# Safety Framework

Single source of truth: `~/so101/shared/safety.json`

Both `ui/app.py` and `mcp_server.py` import thresholds from this file.

## Seven Principles

| ID | Principle | Implementation |
|----|-----------|---------------|
| P1 | No teleport | Ramped moves: speed proportional to error, max 60 counts/cycle |
| P2 | Know before move | Pre-flight checks on teleop state, temps, arm assignment. Read actual position each cycle (not assumed). |
| P3 | Margin of safety | Stay 5% inside calibrated range on both leader and follower |
| P4 | Torque is temporary | Auto-disable after ramp completion (2.5s timeout) |
| P5 | Detect and halt | Continuous load/temp/encoder-wrap monitoring |
| P6 | Equalize before track | Teleop startup: follower aligns to leader pose before tracking begins |
| P7 | Human in the loop | Warnings on anomalies and large moves (>500 counts) |

## Thresholds

```json
{
  "thresholds": {
    "temp_cutoff_c": 65,
    "stall_load_threshold": 800,
    "stall_count_limit": 6,
    "encoder_wrap_threshold": 1500,
    "load_emergency_pct": 75,
    "calibration_margin": 0.05,
    "large_move_warning": 500
  },
  "motion": {
    "max_delta_per_cycle": 60,
    "speed_min": 80,
    "speed_max": 500,
    "speed_k_proportional": 1.2,
    "speed_k_feedforward": 0.6
  }
}
```

## Temperature Limits

| Range | Status |
|-------|--------|
| <50C | Normal continuous operation |
| 50-65C | Elevated, reduce load |
| >65C | Back off immediately |
| 70C | Hardware cutoff (reg 13 default), torque auto-disables |

## Load Thresholds

| Present_Load | Interpretation |
|---|---|
| 0-400 | Light, normal |
| 400-700 | Moderate, acceptable |
| 700-800 | Heavy, approaching limit |
| 800+ | Overload timer started |
| 1000 | Full stall |

## Safety Layers (in teleop, applied in order)

1. **Calibrated range margins** — 5% inside measured min/max
2. **Startup equalization** — follower moves slowly (speed=150) to match leader pose, waits until all joints within 25 counts (up to 6s)
3. **Proportional speed** — speed scales naturally with error, no violent jumps
4. **Encoder wrap detection** — position jump >1500 counts = halt that joint (torque off)
5. **Load monitoring** — any motor >75% load = emergency stop (all torque off)

## Safe Move Procedure (always)

1. Stop motor server and wait 2s for port release
2. Read current position before writing any goal
3. Sanity-check read — if returns 0 with comm=0, do NOT write (corrupt read, port not released)
4. Clamp target to small delta from current (+-100 steps for nudge)
5. Keep torque enabled only during move, disable immediately after

## Stall Recovery Grace

After a commanded move, stall detection is suppressed for 2s (`move_grace` dict) so motors can escape stall zones without tripping overload protection.

## Incident Log

### 2026-05-14: elbow_flex encoder wrap

- **Cause**: Calibration was 0-4095 (never swept). Target computed as 3890 (near max).
- **Sequence**: Motor traveled 2852 -> 4068 -> wrapped to 5 -> continued to ~160 (hard stop). Stalled 35s with torque on. Grinding sound.
- **Damage**: None (temp 29C after).
- **Fix**: Follower elbow_flex recalibrated to 24-4051. With 5% margin, max target is ~3845. Later raised range_min to 300 for desk collision prevention.

### 2026-05-14: Acceleration default 0

- **Cause**: STS3215 ships with Acceleration=0, meaning instant velocity step = mechanical jerk.
- **Fix**: Set all 6 follower motors to Acceleration=100.

### 2026-05-14: Port read corruption

- **Cause**: Motor server killed, port not released, first read returned 0, script wrote goal=50 (hard stop territory).
- **Fix**: Always wait 2s after killing motor server. Always sanity-check reads.

## Calibration Is Safety

Uncalibrated joints (0-4095) give the position mapper permission to command the full encoder range, including past physical hard stops. **Before teleop**: confirm no joint has 0-4095 range in calibration files.
