# Teleoperation & Control

## Architecture

```
Leader arm -> read position (reg 56) + velocity (reg 58)
           |
     map_val(leader_pos, l_lo, l_hi, f_lo, f_hi)  <- calibrated safe ranges
           |
     error = target - follower_pos
     speed = K * error + FF * leader_vel            <- cybernetic loop
           |
Follower arm <- write goal_speed (reg 46) + goal_position (reg 42)
           |
     read follower_pos -> compare to target -> next cycle
```

Loop rate: ~10-20Hz. Full round-trip per joint: 2x read2ByteTxRx + 2x write2ByteTxRx ~ 8ms.

## The Problem with Open-Loop Speed

1. **Teleporting** — follower starts far from leader, fixed speed = violent rush to catch up
2. **Blind commanding** — script sends commands but never checks if follower is tracking; stalls and wraps go undetected

## Proportional Speed Control (current implementation)

```
error  = desired_position - follower_actual_position
speed  = clamp(K_SPEED * |error| + FF_GAIN * |leader_velocity|, SPEED_MIN, SPEED_MAX)
write goal_speed = speed
write goal_position = desired
```

| Parameter | Value | Effect |
|-----------|-------|--------|
| K_SPEED | 1.2 | Speed counts per count of error |
| FF_GAIN | 0.6 | Leader velocity feedforward gain |
| SPEED_MIN | 80 | Minimum speed when nearly at target |
| SPEED_MAX | 500 | Maximum speed when far behind |

- **Small error** -> slow speed -> smooth fine tracking
- **Large error** -> fast speed -> closes gap quickly
- **Leader moving fast** -> velocity feedforward anticipates instead of reacting

## Velocity Feedforward

Leader velocity read from register 58 (Present_Velocity) each cycle. STS3215 format: bit 10 = direction, bits 0-9 = magnitude. Mask with 0x3FF for unsigned speed.

Adds ~3ms per cycle (6 joints x 0.5ms) within 50ms budget at 20Hz.

Skipped during equalization phase (leader stationary, vel=0 expected).

## Startup Equalization

Before tracking begins:
1. Read leader pose
2. Follower moves slowly (speed=150) toward leader's current position
3. Wait until all joints within 25 counts of target (up to 6s timeout)
4. Then begin live tracking

Prevents violent initial jump when poses are far apart.

## Ramped Moves (non-teleop)

MCP and API moves use `ramped=true`:
- Steps toward target at max 60 counts/cycle
- Reads actual position each cycle (not assumed) — self-corrects if motor stalls
- Stall detection suppressed for 2s grace period after commanded move

## LeRobot Native Teleop Loop (~50 Hz)

```
1. robot.get_observation()    -> sync_read Present_Position from follower
2. teleop.get_action()        -> sync_read Present_Position from leader
3. Process through pipeline   -> normalize, scale
4. robot.send_action()        -> sync_write Goal_Position to follower
```

Sync read/write for 6 motors: ~5-10ms each. Total loop: ~15-20ms.

## Data Recording (lerobot-record)

| Property | Value |
|---|---|
| Frame rate | 50 FPS (default) |
| Observations/frame | 6 joint positions |
| Actions/frame | 6 joint commands |
| Format | Parquet + MP4 video |
| Additional | Camera images per frame |

## Key Files

| File | Purpose |
|---|---|
| `~/so101/ui/app.py` | UI server with teleop loop (primary) |
| `~/so101/scripts/teleop_6joint.py` | Standalone teleop with safety layers |
| `~/so101/scripts/wrist_follow.py` | Single-joint teleop demo |
