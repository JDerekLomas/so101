# Diagnostic Scripts & Raw Control

All scripts in `~/so101/scripts/`.

## Script Inventory

| Script | Purpose |
|---|---|
| `scan_motors.py` | Scans all `/dev/tty.usbmodem*` ports, pings IDs 1-19 at 1 Mbps |
| `test_motors.py` | Reads Present_Position from all 6 motors on follower |
| `arm_setup.py` | Checks both arms at all baud rates, prints teleoperate command |
| `range_of_motion.py` | Background daemon tracking min/max ranges |
| `range_monitor.py` | Position range monitoring, auto-ID via wiggle test, saves to range_results.json |
| `calibrate_ranges.py` | Interactive range calibration |
| `detect_ports.py` | CLI port identifier (legacy) |
| `teleop_6joint.py` | Full 6-joint teleop with proportional speed + safety layers |
| `wrist_follow.py` | Single-joint teleop demo |
| `nudge_gripper.py` | Gripper open/close test |
| `recorder.py` | Dataset recording helper |
| `record.sh` | Full dataset recording pipeline with multi-camera |
| `index_conversations.py` | Session indexing utility |
| `setup_so101.py` | Environment setup |
| `mailbox_write.sh` | Write a message to mailbox.json |

## Raw Motor Control Snippets

### Ping All Motors

```python
from scservo_sdk import PortHandler, PacketHandler

port = PortHandler("/dev/tty.usbmodem5B141123331")
port.openPort()
port.setBaudRate(1_000_000)
handler = PacketHandler(0)

for mid in range(1, 7):
    model, comm, err = handler.ping(port, mid)
    print(f"ID {mid}: {'OK' if comm == 0 else 'NO RESPONSE'}")

port.closePort()
```

### Move a Motor

```python
from scservo_sdk import PortHandler, PacketHandler

port = PortHandler("/dev/tty.usbmodem5B141123331")
port.openPort()
port.setBaudRate(1_000_000)
handler = PacketHandler(0)

# Enable torque
handler.write1ByteTxRx(port, 1, 40, 1)

# Set goal position (register 42, 2 bytes)
handler.write2ByteTxRx(port, 1, 42, 2258)

# Disable torque
handler.write1ByteTxRx(port, 1, 40, 0)

port.closePort()
```

### Quick Register Reference

| Action | Register | Size | Value |
|---|---|---|---|
| Enable torque | 40 | 1B | 1 |
| Disable torque | 40 | 1B | 0 |
| Set goal position | 42 | 2B | 0-4095 |
| Set goal speed | 46 | 2B | 0-1023 (bit 10=dir) |
| Read position | 56 | 2B | 0-4095 |
| Read velocity | 58 | 2B | bit 10=dir, 0-9=magnitude |
| Read load | 60 | 2B | bit 10=dir, 0-9=magnitude (0-1000) |
| Read temperature | 63 | 1B | degrees C |
| Read status | 65 | 1B | error bitmask |

## Mailbox Write Helper

```bash
# Usage:
~/so101/scripts/mailbox_write.sh "claude-session-N" "What you did" "tag1,tag2"

# Example:
~/so101/scripts/mailbox_write.sh "claude-session-11" "KB restructured into ~/so101/kb/" "kb,complete"
```
