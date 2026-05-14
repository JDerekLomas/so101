# Hardware & Motor Control

## Motors

| Property | Value |
|---|---|
| Model | Feetech STS3215 |
| Count | 6 per arm (12 total) |
| Resolution | 12-bit (4096 steps, range 0-4095) |
| Protocol | Feetech STS/SMS half-duplex TTL serial, Protocol 0 |
| Baud Rate | 1 Mbps |

## Serial Ports (This Machine)

| Arm | Port |
|---|---|
| Follower | `/dev/tty.usbmodem5B141123331` |
| Leader | `/dev/tty.usbmodem5B141116761` |

**WARNING**: Port assignment is non-deterministic. These swap on replug. Always verify via PID-gain check (see [troubleshooting.md](troubleshooting.md#port-identification)).

## Joint Layout (Both Arms)

| Motor ID | Joint Name |
|---|---|
| 1 | shoulder_pan |
| 2 | shoulder_lift |
| 3 | elbow_flex |
| 4 | wrist_flex |
| 5 | wrist_roll |
| 6 | gripper |

## STS3215 Register Map

### EEPROM (persistent across power cycles)

| Register | Address | Size | Notes |
|---|---|---|---|
| ID | 5 | 1B | Motor ID |
| Baud_Rate | 6 | 1B | 0=1Mbps, 1=500k, 2=250k, 3=128k, 4=115200, 5=57600 |
| Return_Delay_Time | 7 | 1B | |
| Min_Position_Limit | 9 | 2B | |
| Max_Position_Limit | 11 | 2B | |
| Max_Temp_Limit | 13 | 1B | Default 70C |
| P_Coefficient | 21 | 1B | Follower=16, Leader=0 (backdriveable) |
| D_Coefficient | 22 | 1B | Follower=32, Leader=0 |
| I_Coefficient | 23 | 1B | 0 on both |
| Homing_Offset | 31 | 2B | Protocol 0 only |
| Overload_Torque | 36 | 1B | Default 80 (80%) |
| Protection_Torque | 34 | 1B | Default 20 (20% during protection) |
| Protection_Time | 35 | 1B | Default 200 (x40ms = 8s) |

### SRAM (runtime, lost on power cycle)

| Register | Address | Size | Notes |
|---|---|---|---|
| Torque_Enable | 40 | 1B | 1=enabled, 0=disabled |
| Acceleration | 41 | 1B | Motion ramp. Set to 100 on follower (was 0 = instant jerk) |
| Goal_Position | 42 | 2B | Write target position |
| Goal_Speed | 46 | 2B | Speed limit. Bit 10=direction, bits 0-9=magnitude |
| Present_Position | 56 | 2B | Read current position |
| Present_Velocity | 58 | 2B | Bit 10=direction, bits 0-9=magnitude |
| Present_Load | 60 | 2B | Bits 0-9=magnitude (0-1000), bit 10=direction |
| Present_Voltage | 62 | 1B | Value x 0.1V |
| Present_Temperature | 63 | 1B | Degrees C |
| Servo_Status | 65 | 1B | Error bitmask: bit0=voltage, bit1=sensor, bit2=temp, bit3=current, bit5=overload |
| Moving | 66 | 1B | 1=in motion |
| Present_Current | 69 | 2B | Value x 6.5mA |

### PID Gains (confirmed 2026-05-14)

| Register | Follower | Leader | Notes |
|---|---|---|---|
| P_Coefficient (21) | 16 | 0 | Leader P=0 for backdriveable compliance |
| D_Coefficient (22) | 32 | 0 | |
| I_Coefficient (23) | 0 | 0 | |
| Acceleration (41) | 100 | — | Was 0 (instant jerk), set to 100 |

## Position Normalization

| Mode | Raw Range | Normalized | Used For |
|---|---|---|---|
| DEGREES | 0-4095 | -180 to +180 | 5 arm joints |
| RANGE_0_100 | 0-4095 | 0 to 100% | Gripper |

## Calibration

Each motor stores:
- `homing_offset` — EEPROM (Protocol 0). Centers the range.
- `range_min` / `range_max` — define motion range for normalization
- `drive_mode` — direction of travel

### Calibration Files

| Arm | File | Robot ID |
|---|---|---|
| Follower | `~/.cache/huggingface/lerobot/calibration/robots/so_follower/my_follower.json` | `my_follower` |
| Leader | `~/.cache/huggingface/lerobot/calibration/teleoperators/so_leader/my_leader.json` | `my_leader` |

> Robot ID is `my_follower` / `my_leader` — NOT `my_follower_arm`.

### Current Ranges (2026-05-14)

**Follower:**
- shoulder_pan: 917-3660
- shoulder_lift: 659-3110
- elbow_flex: 300-4051 (range_min raised from 24 to 300 for desk collision prevention)
- wrist_flex: 945-3339
- wrist_roll: 27-3892
- gripper: 1323-2862

**Leader:**
- shoulder_pan: 781-3267
- shoulder_lift: 930-3369
- elbow_flex: 1858-4064
- wrist_flex: 882-3227
- wrist_roll: 2-4065
- gripper: 1846-3018

## Voltage Variants

| Arm | Motor Variant | Supply | Expected Present_Voltage |
|---|---|---|---|
| Follower (SO-101) | 12V (C018) | 12V, >=5A | 120-140 (x0.1V) |
| Leader (SO-101) | 7.4V (C001/C044/C046) | 5-8.4V | 60-84 (x0.1V) |

**Plugging 12V into leader = immediate voltage alarm.**

## Python Environment

| Property | Value |
|---|---|
| Python | 3.12 |
| Virtual env | `~/lerobot-env-312/` |
| LeRobot | v0.5.2 |
| Key packages | scservo_sdk, lerobot, anthropic, flask, opencv-python |

## LeRobot Source Files

| File | Purpose |
|---|---|
| `~/so101/lerobot/src/lerobot/motors/motors_bus.py` | Motor bus abstraction |
| `~/so101/lerobot/src/lerobot/motors/feetech/feetech.py` | STS3215 driver |
| `~/so101/lerobot/src/lerobot/motors/feetech/tables.py` | Register map |
| `~/so101/lerobot/src/lerobot/robots/so_follower/so_follower.py` | Follower robot class |
| `~/so101/lerobot/src/lerobot/teleoperators/so_leader/so_leader.py` | Leader teleoperator |

## Teleoperate Command

```bash
lerobot-teleoperate \
  --robot.type=so101_follower \
  --robot.port=/dev/tty.usbmodem5B141123331 \
  --robot.id=my_follower \
  --teleop.type=so101_leader \
  --teleop.port=/dev/tty.usbmodem5B141116761 \
  --teleop.id=my_leader
```

## Patched Code & Workarounds

Root cause: half-duplex TTL bus signal integrity. Motors 2, 3, 4 intermittent in full chain.

1. **`motors_bus.py` — `_assert_motors_exist`**: Retries 5x with 0.5s delays, bypasses missing motor error
2. **`feetech.py` — firmware check**: `raise_on_error=False`, comparison `> 1` (more lenient)
3. **`feetech.py` — `read_calibration`**: Added `num_retry=20` for intermittent bus
