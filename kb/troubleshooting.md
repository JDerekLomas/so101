# Troubleshooting & Diagnostics

## Port Identification

Both controller boards share the same USB serial number. Assignment swaps on every replug.

### PID-Gain Method (preferred, no movement needed)

Leader always has P=0, D=0 (backdriveable). Follower has P=16, D=32.

```python
from scservo_sdk import PortHandler, PacketHandler
import sys, os
sys.stderr = open(os.devnull, "w")
for port_name in ["/dev/tty.usbmodem5B141123331", "/dev/tty.usbmodem5B141116761"]:
    p = PortHandler(port_name); h = PacketHandler(0)
    p.openPort(); p.setBaudRate(1_000_000)
    P, _, _ = h.read1ByteTxRx(p, 1, 21)
    D, _, _ = h.read1ByteTxRx(p, 1, 22)
    role = "LEADER" if P == 0 else "FOLLOWER"
    print(f"{port_name} -> {role} (P={P}, D={D})")
    p.closePort()
```

### Wiggle Test (alternative)

Read positions from both ports in loop for 15s. Physically move leader arm — port showing changes is leader.

## Quick Health Check

```python
from scservo_sdk import PortHandler, PacketHandler
import os, sys
sys.stderr = open(os.devnull, 'w')

PORT = "/dev/tty.usbmodem5B141123331"  # confirm via PID check first
NAMES = {1:"shoulder_pan",2:"shoulder_lift",3:"elbow_flex",4:"wrist_flex",5:"wrist_roll",6:"gripper"}

port = PortHandler(PORT); h = PacketHandler(0)
port.openPort(); port.setBaudRate(1_000_000)

print(f"{'motor':<16} {'pos':>5} {'load%':>6} {'volt V':>7} {'temp C':>7} {'status':>8} {'current mA':>10}")
for mid in range(1, 7):
    pos,  c0, _ = h.read2ByteTxRx(port, mid, 56)
    load, c1, _ = h.read2ByteTxRx(port, mid, 60)
    volt, c2, _ = h.read1ByteTxRx(port, mid, 62)
    temp, c3, _ = h.read1ByteTxRx(port, mid, 63)
    stat, c4, _ = h.read1ByteTxRx(port, mid, 65)
    curr, c5, _ = h.read2ByteTxRx(port, mid, 69)
    if c0 == 0:
        load_pct = (load & 0x3FF) / 10.0
        errors = []
        if stat & 0x01: errors.append("VOLTAGE")
        if stat & 0x04: errors.append("TEMP")
        if stat & 0x20: errors.append("OVERLOAD")
        print(f"{NAMES[mid]:<16} {pos:>5} {load_pct:>6.1f} {volt*0.1:>7.1f} {temp:>7} {'OK' if stat==0 else ','.join(errors):>8} {curr*6.5:>10.0f}")
port.closePort()
```

## Common Problems

### "Chunky" / Stiff / Jerky Movement

**Causes (in order of likelihood):**
1. **Overload protection** — servo hits ~80% torque, drops to 20%, recovers. Check Servo_Status bit 5, Present_Load >800.
2. **PID too aggressive** — factory D=32 causes overshoot. Lower D to 4-8, P from 32 to 20.
3. **No acceleration ramp** — Acceleration=0 = instant velocity step. Write 50-150 to reg 41.
4. **Gear wear** — ~0.87 degree backlash from new (spec <=0.5). Gripper and shoulder_lift wear fastest.
5. **Gravity load** — shoulder_lift and elbow_flex fight gravity. Use rest pose + torque-off when idle.

### Overheating

Causes: continuous high-torque holding, overload oscillation, fighting mechanical stops.
Fix: disable torque when idle, rest pose between episodes, lower Max_Temp_Limit (reg 13) to 60C for early warning.

### Voltage Errors (`[RxPacketError] Input voltage error!`)

Most common community failure. Wrong supply voltage for motor variant. See [hardware.md](hardware.md#voltage-variants).

### Motor Not Found / Comm Errors (comm=-6, RX timeout)

1. Reseat JST connectors on daisy chain
2. Verify unique motor IDs (new servos ship as ID=1)
3. Confirm correct voltage
4. Confirm same firmware (regs 0-1)
5. Stop any other process holding serial port
6. Connect one servo at a time to check for duplicate IDs
7. Firmware update: FT SCServo Debug on Windows (required)

macOS debug tool: [FT_SCServo_Debug_Qt](https://github.com/CarolinePascal/FT_SCServo_Debug_Qt/tree/fix/port-search-timer)

### Overload Protection Triggered (motor goes limp)

Motor drops to 20% torque after sustained >80% for ~8s.

```python
# Recovery (no power cycle needed):
h.write2ByteTxRx(port, motor_id, 42, current_position)
# OR:
h.write1ByteTxRx(port, motor_id, 40, 0); time.sleep(0.5)
h.write1ByteTxRx(port, motor_id, 40, 1)
```

Tune sensitivity: reg 36 (Overload_Torque, default 80%), reg 34 (Protection_Torque, default 20%), reg 35 (Protection_Time, default 200x40ms=8s).

### Calibration Lost After Restart

LeRobot bug (issue #1342): negative homing offsets overflow uint16. Re-run calibration if lost. Files stored in `~/.cache/huggingface/lerobot/calibration/` — not in EEPROM.

### wrist_roll Calibration Crash

LeRobot crashes with `ValueError: Magnitude exceeds 2047` if wrist_roll not near raw position 2048. Rotate manually to mid-range before calibrating.

### Gripper Servo Failure

Most common single-servo failure. Caused by stall overload or cold damage. Prevention: never command gripper minimum when holding an object. Add torque/current limit for gripper.

## Community Failure Modes

| Symptom | Root Cause | Fix |
|---------|-----------|-----|
| All motors missing | Wrong voltage supply | Match 12V vs 7.4V to motor variant |
| All motors missing | Loose JST connector | Reseat every connector |
| Incorrect status packets | Duplicate motor IDs | Assign unique IDs one-at-a-time |
| Incorrect status packets | Firmware mismatch | Update all via FT SCServo Debug |
| Elbow range limited ~90 degrees | Calibration rollover bug | Use range_monitor.py instead of LeRobot cal |
| Wrist_roll cal crash | Joint not near pos 2048 | Rotate manually first |
| Calibration lost on restart | Negative offset uint16 overflow | Re-run calibration |
| Gripper stiff then dead | Stall overload / cold damage | Replace servo |
| Encoder wrap, stall at hard stop | Uncalibrated 0-4095 range | Re-sweep calibration |

## Maintenance Checklist

- [ ] Before each session: confirm port assignments via PID check
- [ ] Every session: check Servo_Status and temperature at startup
- [ ] Every 20h operation: inspect daisy-chain cables at rotating joints
- [ ] When arm feels different: read Present_Load, compare to baseline
- [ ] When replacing servo: check firmware version matches, assign correct ID before bus connect
- [ ] When idle >5 min: disable torque, move to rest pose
