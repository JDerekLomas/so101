# SO-101 Robot Arm Knowledge Base

A comprehensive day-to-day reference for the SO-101 robot arm setup, motor control, training, and community ecosystem.

---

## Table of Contents

1. [Hardware & Setup](#1-hardware--setup)
2. [Motor Control Architecture](#2-motor-control-architecture)
3. [Patched Code & Workarounds](#3-patched-code--workarounds)
4. [Diagnostic Scripts & Raw Control](#4-diagnostic-scripts--raw-control)
5. [Community Ecosystem](#5-community-ecosystem)
6. [Models & Datasets](#6-models--datasets)
7. [Community Lessons from Real Training](#7-community-lessons-from-real-training)
8. [Inter-Worker Shared State & Mailbox](#8-inter-worker-shared-state--mailbox)
9. [Useful Links](#9-useful-links)
10. [Known Footguns](#10-known-footguns)
11. [Motor Health, Troubleshooting & Diagnostics](#11-motor-health-troubleshooting--diagnostics)
12. [Teleoperation Control Loop & Cybernetics](#12-teleoperation-control-loop--cybernetics)

---

## 1. Hardware & Setup

### Motors

| Property | Value |
|---|---|
| Model | Feetech STS3215 |
| Count | 6 per arm |
| Resolution | 12-bit (4096 steps, range 0-4095) |
| Protocol | Feetech STS/SMS half-duplex TTL serial |
| Protocol Version | 0 |
| Baud Rate | 1 Mbps |

### Serial Ports (This Machine)

| Arm | Port |
|---|---|
| Follower | `/dev/tty.usbmodem5B141123331` |
| Leader | `/dev/tty.usbmodem5B141116761` |

### Joint Layout (Both Arms)

| Motor ID | Joint Name |
|---|---|
| 1 | shoulder_pan |
| 2 | shoulder_lift |
| 3 | elbow_flex |
| 4 | wrist_flex |
| 5 | wrist_roll |
| 6 | gripper |

### Python Environment

| Property | Value |
|---|---|
| Python version | 3.12 |
| Virtual environment | `~/lerobot-env-312` (use this one) |
| LeRobot version | v0.5.2 |
| LeRobot path | `~/so101/lerobot/` |
| Workspace | `~/so101/robot-workspace/` |
| Web tool | `http://localhost:5833` (port detection + calibration) |

### Calibration Files

| Arm | File Path | Robot ID |
|---|---|---|
| Follower | `~/.cache/huggingface/lerobot/calibration/robots/so_follower/my_follower.json` | `my_follower` |
| Leader | `~/.cache/huggingface/lerobot/calibration/teleoperators/so_leader/my_leader.json` | `my_leader` |

> **Note:** The robot ID is `my_follower` and `my_leader` — not `my_follower_arm`.

### Teleoperate Command

```bash
lerobot-teleoperate \
  --robot.type=so101_follower \
  --robot.port=/dev/tty.usbmodem5B141123331 \
  --robot.id=my_follower \
  --teleop.type=so101_leader \
  --teleop.port=/dev/tty.usbmodem5B141116761 \
  --teleop.id=my_leader
```

---

## 2. Motor Control Architecture

### Key Source Files

| File | Purpose |
|---|---|
| `/Users/dereklomas/lerobot/src/lerobot/motors/motors_bus.py` | Motor bus abstraction layer |
| `/Users/dereklomas/lerobot/src/lerobot/motors/feetech/feetech.py` | STS3215 driver |
| `/Users/dereklomas/lerobot/src/lerobot/motors/feetech/tables.py` | Register map |
| `/Users/dereklomas/lerobot/src/lerobot/robots/so_follower/so_follower.py` | Follower robot class |
| `/Users/dereklomas/lerobot/src/lerobot/teleoperators/so_leader/so_leader.py` | Leader teleoperator class |
| `/Users/dereklomas/lerobot/src/lerobot/scripts/lerobot_teleoperate.py` | Teleoperation entry point |

### STS3215 Register Map

**EEPROM Registers**

| Register | Address | Size (bytes) | Notes |
|---|---|---|---|
| ID | 5 | 1 | Motor ID |
| Baud_Rate | 6 | 1 | See baud rate table below |
| Return_Delay_Time | 7 | 1 | |
| Min_Position_Limit | 9 | 2 | |
| Max_Position_Limit | 11 | 2 | |
| Homing_Offset | 31 | 2 | Protocol 0 only |

**SRAM Registers**

| Register | Address | Size (bytes) | Notes |
|---|---|---|---|
| Torque_Enable | 40 | 1 | 1=enabled, 0=disabled |
| Goal_Position | 42 | 2 | Write target position |
| Present_Position | 56 | 2 | Read current position |
| Present_Velocity | 58 | 2 | Read current velocity |

**PID / Motion Registers (confirmed via direct read 2026-05-14)**

| Register | Addr | Follower | Leader | Notes |
|---|---|---|---|---|
| P_Coefficient | 21 | 16 | 0 | Leader P=0 intentional (backdriveable compliance) |
| I_Coefficient | 23 | 0 | 0 | |
| D_Coefficient | 22 | 32 | 0 | Leader D=0 intentional |
| Acceleration | 41 | 100 | — | Was 0 (instant jerk); set to 100 on 2026-05-14 |
| Goal_Speed | 46 | 200 (wrist_follow) | — | Speed limit written per-session by scripts |

### Baud Rate Table

| Register Value | Baud Rate |
|---|---|
| 0 | 1 Mbps (default) |
| 1 | 500k |
| 2 | 250k |
| 3 | 128k |
| 4 | 115200 |
| 5 | 57600 |

### Position Normalization Modes

| Mode | Raw Range | Normalized Range | Used For |
|---|---|---|---|
| `DEGREES` | 0-4095 | -180 to +180 degrees | 5 arm joints |
| `RANGE_0_100` | 0-4095 | 0 to 100 percent | Gripper |
| `RANGE_M100_100` | 0-4095 | -100 to +100 | Alternate mode |

### Calibration Data Per Motor

Each motor stores:
- `homing_offset` — written to EEPROM (Protocol 0 only). Centers the range. Formula: `Present_Position = Actual_Position - Homing_Offset`
- `range_min` / `range_max` — define the motion range for normalization
- `drive_mode` — direction of travel

Calibration is written to motor EEPROM and also cached in the JSON files listed in Section 1.

### Teleoperation Loop (~50 Hz)

```
1. robot.get_observation()    → sync_read Present_Position from follower
2. teleop.get_action()        → sync_read Present_Position from leader
3. Process through pipeline   → normalize, scale
4. robot.send_action()        → sync_write Goal_Position to follower
```

- Sync read/write for 6 motors takes ~5-10ms each
- Total loop time: ~15-20ms

### Data Recording (`lerobot-record`)

| Property | Value |
|---|---|
| Frame rate | 50 FPS (default) |
| Observations per frame | 6 joint positions |
| Actions per frame | 6 joint commands |
| Storage format | HDF5 episodes with Parquet + MP4 video |
| Additional data | Camera images per frame |

---

## 3. Patched Code & Workarounds

### Root Cause

Half-duplex TTL bus signal integrity issues. Motors 2, 3, and 4 become intermittent when all 6 motors are in a full chain. Workaround: increase retries and bypass the missing motor check.

### Patch 1: `motors_bus.py` — `_assert_motors_exist`

The original function raises an error if any motor does not respond. The patched version retries up to 5 times with 0.5s delays and skips the missing-motor error:

```python
def _assert_motors_exist(self) -> None:
    import time
    expected_models = {m.id: self.model_number_table[m.model] for m in self.motors.values()}
    found_models = {}
    for attempt in range(5):
        for id_ in self.ids:
            if id_ not in found_models:
                model_nb = self.ping(id_, num_retry=20)
                if model_nb is not None:
                    found_models[id_] = model_nb
        if len(found_models) == len(self.ids):
            break
        time.sleep(0.5)
    missing_ids = []  # bypassed — motors present but intermittent
```

### Patch 2: `feetech.py` — Firmware Check

- `_assert_same_firmware`: changed `raise_on_error=True` to `raise_on_error=False`
- `_assert_same_firmware`: changed comparison `!= 1` to `> 1` (more lenient check)

### Patch 3: `feetech.py` — `read_calibration`

Added `num_retry=20` to all reads within `read_calibration` to handle intermittent bus responses.

---

## 4. Diagnostic Scripts & Raw Control

### Diagnostic Scripts

All scripts are in `~/so101/robot-workspace/`.

| Script | Purpose |
|---|---|
| `scan_motors.py` | Scans all `/dev/tty.usbmodem*` ports, pings IDs 1-19 at 1 Mbps, reports OK/MISSING |
| `test_motors.py` | Reads `Present_Position` from all 6 motors on the follower port |
| `arm_setup.py` | Checks both arms at all baud rates, prints teleoperate command if ready |
| `range_of_motion.py` | Background daemon — reads positions, tracks min/max ranges, saves to calibration JSON on stop |
| `detect_ports.py` | CLI port identifier — wiggle arms to assign leader/follower, writes robot.env |
| `motor_server.py` | HTTP server on :7777 — positions, ranges, save calibration (legacy, superseded by port_detector) |

**Primary tool:** `port_detector/app.py` — web UI on :5833. Handles port detection, live position streaming, and range calibration. Auto-detects boards on plug/unplug. Writes `~/so101/shared/robot_state.json` continuously.

### Quick Motor Ping (Raw Python)

```python
from scservo_sdk import PortHandler, PacketHandler

port = PortHandler("/dev/tty.usbmodem5B141123331")
port.openPort()
port.setBaudRate(1_000_000)

handler = PacketHandler(0)  # Protocol 0 for STS3215

for mid in range(1, 7):
    model, comm, err = handler.ping(port, mid)
    print(f"ID {mid}: {'OK' if comm == 0 else 'NO RESPONSE'}")

port.closePort()
```

### Move a Motor (Raw Python)

```python
from scservo_sdk import PortHandler, PacketHandler

port = PortHandler("/dev/tty.usbmodem5B141123331")
port.openPort()
port.setBaudRate(1_000_000)
handler = PacketHandler(0)

# Enable torque on motor 1
handler.write1ByteTxRx(port, 1, 40, 1)

# Set goal position (register 42, 2 bytes)
handler.write2ByteTxRx(port, 1, 42, 2258)

# Disable torque
handler.write1ByteTxRx(port, 1, 40, 0)

port.closePort()
```

**Register quick reference for raw control:**

| Action | Register | Size | Value |
|---|---|---|---|
| Enable torque | 40 | 1 byte | 1 |
| Disable torque | 40 | 1 byte | 0 |
| Set goal position | 42 | 2 bytes | 0-4095 |
| Read present position | 56 | 2 bytes | — |

---

## 5. Community Ecosystem

### Alternative Control Software

| Project | Description |
|---|---|
| **phosphobot** | Browser UI, one-click recording and training, supports ACT / SmolVLA / pi0.5 / GR00T N1.5. `github.com/phospho-app/phosphobot` |
| **kedikala/soarm101** | Simpler middleware alternative to lerobot |
| **viam-devrel/so-101** | Viam platform integration with web calibration tool, remote teleoperation, Python and multi-language SDK |
| **msf4-0/so101_ros2** | Pure ROS2 control without lerobot dependency |
| **MuammerBay/isaac_so_arm101** | Isaac Lab sim-to-real pipeline |

### MCP Servers (LLM-Native Robot Control)

| Project | Description |
|---|---|
| **IliaLarchenko/robot_MCP** | MCP server for Claude / GPT / Gemini to directly control SO-ARM100/101. Uses lerobot normalized joint states. Update `MOTOR_NORMALIZED_TO_DEGREE_MAPPING` in `config.py` to match your calibration. |
| **phospho-app/phospho-mcp-server** | phosphobot MCP bridge |
| **titansage02/so101-mcp** | SO-101 specific MCP integration |

### Notable Hackathon Projects

| Project | Notes |
|---|---|
| circuitrobot (ronantakizawa) | 1st place Asia / 4th global at HF hackathon. Task: circuit connection. |
| RoboTAI (Robotawi/RoboTAI_AMD_Robotics_Hackathon_2025) | Drawer open / pick / place pipeline using ACT. |

### Simulation

| Platform | Use Case |
|---|---|
| Isaac Sim digital twin | Real-to-sim position mirroring |
| Isaac Lab RL | Sim-to-real policy transfer via reinforcement learning |
| SO-ARM-chan | Stack-chan personality grafted onto SO-ARM body (Hackster project) |

---

## 6. Models & Datasets

### Model Recommendation Ladder

| Tier | Model | Params | VRAM | Train Time | Success Rate | Notes |
|---|---|---|---|---|---|---|
| 1 (start here) | ACT | 52M | Low | ~4h (RTX 3080 12GB) | 60-90% with 50-150 demos | Fast iteration, low resource |
| 2 | SmolVLA | 450M | ~24GB | ~10h (RTX 3090) | 78%+ on pick-place | Better generalization, async inference, language conditioning |
| 3 | GR00T N1.5 | 3B | 25GB+ | — | — | Cross-embodiment, language-steered. SO-101 not in pretraining — train as `new_embodiment` |
| 4 | pi0 / pi0-fast | 3B+ | High | — | — | Community has fine-tuned on SO-101. Base: `lerobot/pi0_base` |

### SmolVLA Key Facts

| Property | Value |
|---|---|
| Total parameters | 450M |
| Architecture | SmolVLM2-500M-Video-Instruct backbone (350M) + flow-matching action expert (100M) |
| Pretraining data | 487 community datasets / ~10M frames / ~30k episodes from SO-100 community |
| Real-world performance | 78.3% pick-place success (vs 51.7% without pretraining) |
| Fine-tune time | ~10h on 1x A100 |
| Fine-tune settings | 20k steps, batch size 64, lr 1e-4 cosine schedule |
| Async inference speedup | 30% faster, 2x task throughput |
| Community fine-tunes | 5,478 variants on HF Hub |
| Base model | `lerobot/smolvla_base` |

### Official LeRobot Datasets for SO-101/SO-100

| Dataset | Episodes | Frames | Task |
|---|---|---|---|
| `lerobot/svla_so101_pickplace` | 50 | — | Pick object, place in container |
| `lerobot/svla_so100_pickplace` | 50 | 19,631 | Pick and place |
| `lerobot/svla_so100_stacking` | 56 | 22,956 | Block stacking |
| `lerobot/svla_so100_sorting` | 52 | 35,713 | Object sorting |

All datasets: 30 FPS, top + wrist cameras, 480x640 resolution.

### Community Scale (as of late 2025)

| Metric | Value |
|---|---|
| Total SO-100 + SO-101 datasets on HF Hub | 8,500+ (majority of all LeRobot content) |
| Episodes per dataset (43% of datasets) | 1-5 episodes |
| Median episodes per dataset | 10 |
| Pretrained SO-101 policies on Hub | 72+ |

### Notable Community Models

| Model | Description |
|---|---|
| `felixmayor/pi05_so101_orange_cube` | Pi0.5 on SO-101, 154 demos / 68.5k frames |
| `mason-mcgough/act-lerobot-so101-lego-cube-50-default` | ACT for lego cube task, 50 demos |
| `kogeek/lerobot_so101_gr00t_n1.6` | GR00T 3B fine-tune |
| `youliangtan/so101-table-cleanup` | Table cleanup dataset, used in NVIDIA tutorial |

---

## 7. Community Lessons from Real Training

### ACT Training — Sherry Chen (3 Attempts, RTX 3080)

**Results summary:**

| Attempt | Demos | In-Distribution | Out-of-Distribution | Key Issue |
|---|---|---|---|---|
| 1 | 50 | ~0% | — | Camera POV changed between train/eval; overfit to fixed locations |
| 2 | 72 (50 train + eval set) | 60% | 10% | Fixed cameras, spatial bins — better but still limited generalization |
| 3 | ~150 (25/bin × 6 bins + yaw variation) | 90% | 75% | Systematic spatial coverage with orientation variation |

**Key lessons:**

1. **Consistent camera placement is critical** — mark positions with tape or rulers; any shift degrades performance
2. **Spatial diversity beats quantity** — cover the full workspace and vary object orientation; 6 bins × 25 demos outperformed 150 random demos
3. **Always maintain an eval set during training** — needed to detect overfitting early
4. **Gripper friction matters** — add tape to gripper tips to prevent slipping
5. **Camera USB randomization** — use udev rules to lock camera device identities across reboots
6. **Do not over-grasp** — motor wear accumulates; consider adding velocity limits

### SmolVLA Fine-Tuning — ggando.com (RTX 3090)

**Setup:** 75 episodes in a constrained 10cm workspace.

**Results:**

| Model | Cameras | Success Rate | Conditions |
|---|---|---|---|
| SmolVLA | Dual (wrist + overhead) | 100% (5/5) | Night |
| SmolVLA | Dual (wrist + overhead) | 60-80% | Afternoon |
| ACT | Dual (wrist + overhead) | 80% (4/5) | — |

**Key lessons:**

- "One clean strategy beats a mix of tricks" — consistency in demo collection matters more than volume
- Teleoperation lag degrades demo quality — record slower and more deliberately
- Dual camera (wrist + overhead) significantly improves trajectory smoothness

### GR00T N1.5 Community Notes

- Default motion is jerky out of the box
- Increase denoising steps to 16 and action horizon to 16 to smooth motion
- Minimum viable training: 5k-6k steps; overtraining causes the model to ignore language instructions
- SO-101 is not in the pretraining data — always tag datasets as `new_embodiment`

---

## 8. Inter-Worker Shared State & Mailbox


### Shared files

| File | Written by | Read by | Purpose |
|---|---|---|---|
| `~/so101/shared/robot_state.json` | robot tool (port_detector) | KB worker | Live motor positions, temps, loads, connection status, recent events. Updated at 2 Hz. |
| `~/so101/shared/kb_context.json` | KB worker | robot tool | Hardware docs, config, context the robot tool should know |

### Mailbox

Directory-based message passing. No concurrent write conflicts.

```
~/so101/shared/messages/
  to_robot/    ← KB worker drops .json files here
  to_kb/       ← robot tool drops .json files here
```

**Envelope format:**
```json
{ "id": "1747612800_scan_request", "from": "kb", "to": "robot",
  "type": "scan_request", "ts": 1747612800.0, "payload": {} }
```

**Message types robot tool handles (`to_robot/`):**

| type | effect |
|---|---|
| `scan_request` | Rescans USB ports, updates robot_state.json |
| `calibrate_start` | Starts range recording |
| `calibrate_stop` | Stops range recording |

**Messages robot tool sends (`to_kb/`):**

| type | when |
|---|---|
| `connected` | Board detected on USB |
| `disconnected` | Board removed |
| `port_assigned` | Leader/follower saved to robot.env |
| `calibration_started` | Recording begun |
| `calibration_saved` | Ranges written to calibration JSON |
| `scan_complete` | Rescan finished |
| `motor_error` | High packet error rate on a board |

> **Legacy:** `~/so101/mailbox.json` was used by the previous session. Superseded by the directory mailbox above.

---

## 9. Useful Links


| Resource | URL |
|---|---|
| Official SO-101 docs | https://huggingface.co/docs/lerobot/so101 |
| SmolVLA blog post | https://huggingface.co/blog/smolvla |
| SmolVLA base model | https://huggingface.co/lerobot/smolvla_base |
| GR00T N1.5 SO-101 tuning tutorial | https://huggingface.co/blog/nvidia/gr00t-n1-5-so101-tuning |
| ACT training writeup (Sherry Chen) | https://huggingface.co/blog/sherryxychen/train-act-on-so-101 |
| SmolVLA fine-tuning writeup (ggando) | https://ggando.com/blog/smolvla-so101/ |
| phosphobot | https://github.com/phospho-app/phosphobot |
| robot_MCP (Claude/LLM direct control) | https://github.com/IliaLarchenko/robot_MCP |
| Viam SO-101 module | https://github.com/viam-devrel/so-101 |
| Viam codelab | https://codelabs.viam.com/guide/so101 |
| NVIDIA Isaac sim-to-real guide | https://docs.nvidia.com/learning/physical-ai/sim-to-real-so-101/latest |
| HF hackathon SO-ARM datasets | https://huggingface.co/collections/whosricky/so-arm-101-datasets |
| LeRobot dataset analysis | https://www.kamenski.me/articles/analyzing-lerobot-datasets-on-hugging-face |
| Seeed Studio GR00T N1.5 + Jetson Thor wiki | https://wiki.seeedstudio.com/fine_tune_gr00t_n1.5_for_lerobot_so_arm_and_deploy_on_jetson_thor |
| Jetson AGX Orin setup guide | https://www.hackster.io/shahizat/running-lerobot-so-101-arm-kit-using-nvidia-jetson-agx-orin-19b8a4 |

---

## 10. Known Footguns

### Port assignment is non-deterministic — easy to swap follower/leader

Both controller boards share the same USB serial number. On every replug or power cycle, macOS may assign `/dev/tty.usbmodem5B141123331` and `/dev/tty.usbmodem5B141116761` to either arm — the assignment is arbitrary.

**Confirmed mapping as of 2026-05-14 (PID-gain identification):**
| Port | Arm | P gain | D gain |
|------|-----|--------|--------|
| `/dev/tty.usbmodem5B141123331` | **Follower** | 16 | 32 |
| `/dev/tty.usbmodem5B141116761` | **Leader** | 0 | 0 |

This will likely swap again on the next replug. **Always verify before trusting port labels.**

**Definitive identification method — PID gains (preferred, no arm movement needed):**
```python
# Run with motor_server stopped. Leader always has P=0, D=0 (backdriveable design).
# Follower always has P=16, D=32 (position control gains).
from scservo_sdk import PortHandler, PacketHandler
import sys, os
sys.stderr = open(os.devnull, "w")
for port_name in ["/dev/tty.usbmodem5B141123331", "/dev/tty.usbmodem5B141116761"]:
    p = PortHandler(port_name); h = PacketHandler(0)
    p.openPort(); p.setBaudRate(1_000_000)
    P, _, _ = h.read1ByteTxRx(p, 1, 21)  # motor 1 P gain
    D, _, _ = h.read1ByteTxRx(p, 1, 22)  # motor 1 D gain
    role = "LEADER" if P == 0 else "FOLLOWER"
    print(f"{port_name} → {role} (P={P}, D={D})")
    p.closePort()
```

**Wiggle test (alternative, when motors might have non-default gains):**
1. Read positions from both ports in a loop for 15s
2. Physically move the leader arm — the port showing position changes is the leader
3. Script: `/tmp/wiggle_check.py` (run with `source ~/lerobot-env-312/bin/activate && python3.12 -u /tmp/wiggle_check.py`)

**After confirming assignments:** update `motor_server.py` `PORT_PATH` and any port constants in scripts if swapped.

**Safe move procedure (always):**
1. Stop motor server (`pkill -f motor_server.py`) and wait 2 full seconds for port release
2. Read current position before writing any goal
3. Sanity-check the read — if it returns 0 with comm=0, do NOT write; the read is corrupt (port not fully released)
4. Clamp target to a small delta from current (±100 steps max for a nudge)
5. Keep torque enabled only during the move, disable immediately after

**What went wrong 2026-05-14:**
Motor server killed, port not released in time, first read returned 0 (corrupt), script wrote goal=50 (hard stop territory), triggered overload protection on the bus, all 6 motors went silent. Recovery: power cycle the arm.

### wrist_roll calibration bug (LeRobot v0.5.x)
LeRobot's calibration crashes with `ValueError: Magnitude exceeds 2047` if `wrist_roll` is not physically near raw position 2048 before starting. Rotate the joint manually to mid-range before running calibration.

---

## 11. Motor Health, Troubleshooting & Diagnostics

### Key Diagnostic Registers (read these first when something feels wrong)

| Register | Addr | Size | What it tells you |
|----------|------|------|-------------------|
| Present_Load | 60 | 2B | Torque output: bits 0–9 = magnitude (0–1000), bit 10 = direction. >800 = approaching overload |
| Present_Voltage | 62 | 1B | Supply voltage × 0.1V. 120 = 12.0V. 7.4V motors: expect 60–84. 12V motors: expect 90–140 |
| Present_Temperature | 63 | 1B | Degrees C. >50°C = watch it. >65°C = back off. Cutoff at 70°C (reg 13) |
| Servo_Status | 65 | 1B | Error bitmask: bit0=voltage, bit1=sensor, bit2=temperature, bit3=current, bit5=overload. 0x00 = healthy |
| Moving | 66 | 1B | 1 = in motion. Oscillating between 0/1 when stationary = controller fighting |
| Present_Current | 69 | 2B | Current × 6.5mA |

```python
# Quick health check for all motors (run with motor_server stopped)
from scservo_sdk import PortHandler, PacketHandler
import os, sys
sys.stderr = open(os.devnull, 'w')

PORT = "/dev/tty.usbmodem5B141123331"  # follower (confirm via wiggle test)
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

### Problem: "Chunky" / Stiff / Jerky Movement

**Most likely causes (in order):**

1. **Overload protection partially engaged** — servo hits load threshold (~80% torque), briefly drops to 20% protection torque, recovers. Feels like intermittent weakness or stickiness.
   - Check: `Servo_Status` bit 5. `Present_Load` sustained >800.

2. **PID tuning** — factory D=32 is high, causes overshoot/oscillation, especially on compliant configurations.
   - Fix: lower D coefficient (reg 22) to 4–8. Try reducing P (reg 21) from 32 to 20 for smoother compliance.

3. **No acceleration ramp** — default Acceleration (reg 41) = 0 means instant velocity step = jerk.
   - Fix: write 50–150 to reg 41 for a motion ramp. LeRobot sets this to 254 in `configure_motor.py`.
   - **Applied 2026-05-14:** all 6 follower motors set to acc=100 (was 0). Leader unchanged (backdriveable, no ramp needed).

4. **Gear wear / backlash** — measured ~0.87° backlash from new (spec is ≤0.5°). Gets worse over time.
   - Detect: audible grinding, physical play with torque off. Gripper and shoulder_lift wear fastest.

5. **Gravity-loaded joint near stall** — shoulder_lift and elbow_flex fight gravity continuously.
   - Fix: use a rest pose + torque-off when idle.

### Problem: Overheating

**Safe temperature ranges:**
- <50°C: normal continuous operation
- 50–65°C: elevated, reduce load
- >65°C: back off immediately
- 70°C: hardware cutoff (reg 13 default), torque disables automatically

**Causes:** continuous high-torque holding under gravity load, overload oscillation, miscalibrated range causing motor to fight its own mechanical stop.

**Fix:** disable torque when idle, move to low-stress rest pose, allow cool-down between training episodes. Lower Max_Temp_Limit (reg 13) to 60°C for early warning.

### Problem: Voltage Errors (`[RxPacketError] Input voltage error!`)

**Two voltage variants — this is the #1 community failure:**
| Arm | Motor variant | Correct supply |
|-----|--------------|----------------|
| Follower (SO-101) | 12V (C018) | 12V, ≥5A |
| Leader (SO-101) | 7.4V (C001/C044/C046) | 5–8.4V |

Plugging 12V into the leader = immediate voltage alarm. Sag from daisy-chain cable resistance under peak load also triggers it.

**Present_Voltage (reg 62) × 0.1 = actual volts.** A reading of 120–122 on a 12V motor is normal. A reading of 122 on a 7.4V motor is an over-voltage fault.

### Problem: Motor Not Found / Comm Errors (comm=-6, RX timeout)

**Fix sequence:**
1. Reseat every JST connector on the daisy chain (most common cause)
2. Verify all motor IDs are unique — new servos all ship with ID=1
3. Confirm correct voltage supply
4. Confirm all servos on same firmware (registers 0–1 = major/minor version)
5. Stop any other process holding the serial port
6. Check for duplicate IDs (connect one servo at a time to scan)
7. For firmware update: use FT SCServo Debug on Windows (required for firmware)

**macOS debug tool:** FT_SCServo_Debug_Qt (Linux/macOS read-only diagnostics)
`https://github.com/CarolinePascal/FT_SCServo_Debug_Qt/tree/fix/port-search-timer`

### Problem: Overload Protection Triggered (motor goes limp)

Motor drops to 20% torque after sustained load >80% for ~8 seconds.

**Recovery (no power cycle needed):**
```python
# Send any new goal position to clear overload state
h.write2ByteTxRx(port, motor_id, 42, current_position)
# OR: torque off then on
h.write1ByteTxRx(port, motor_id, 40, 0); time.sleep(0.5)
h.write1ByteTxRx(port, motor_id, 40, 1)
```

**Tune overload sensitivity:**
- Reg 36 (Overload_Torque): default 80 (80%). Raise to reduce false triggers, lower for protection.
- Reg 34 (Protection_Torque): default 20 (20%). The "safe" torque during protection.
- Reg 35 (Protection_Time): default 200 × 40ms = 8s timer.

### Problem: Calibration Lost After Restart

Known LeRobot bug (issue #1342, open as of early 2026): negative homing offsets overflow uint16 and are corrupted on write. Calibration must be re-run if homing offsets are lost.

Also: calibration is stored in `~/.cache/huggingface/lerobot/calibration/` JSON files, not in motor EEPROM. If JSON files are deleted, re-run calibration.

### Problem: Gripper Servo Failed

Most common single-servo failure. Caused by:
- Gripping fully closed against a held object (stall + overload)
- Cold temperature damage (leaving arm in cold car)

**Prevention:** never command gripper to minimum position when holding an object. Add a torque limit or current limit for gripper specifically. Keep spare STS3215 servos on hand.

### Maintenance Checklist

- [ ] **Before each session:** confirm port assignments — use PID-gain check (preferred, no arm movement) or wiggle test. See Section 10.
- [ ] **Every session:** check Servo_Status and temperature on all motors at startup
- [ ] **Every 20h of operation:** inspect daisy-chain cables at rotating joints for kinking
- [ ] **When an arm feels different:** read Present_Load and compare to baseline
- [ ] **When replacing a servo:** check firmware version matches others; assign correct ID before connecting to bus
- [ ] **When idle >5 min:** disable torque and move arm to rest pose

### Load Thresholds Reference

| Present_Load magnitude | Interpretation |
|------------------------|---------------|
| 0–400 | Light, normal |
| 400–700 | Moderate, acceptable |
| 700–800 | Heavy, approaching limit |
| 800+ | Overload timer started (default 80% threshold) |
| 1000 | Full stall |

### Community-Documented Failure Modes (SO-101 / LeRobot)

| Symptom | Root cause | Fix |
|---------|-----------|-----|
| All motors missing | Wrong voltage supply | Match supply to motor variant (12V vs 7.4V) |
| All motors missing | Loose JST connector | Reseat every connector |
| Incorrect status packets | Duplicate motor IDs | Assign unique IDs one-at-a-time |
| Incorrect status packets | Firmware version mismatch | Update all to same firmware via FT SCServo Debug |
| Elbow range limited to ~90° | Calibration rollover bug | No fix; use range_monitor.py instead of LeRobot cal |
| Wrist_roll cal crash | Joint not near pos 2048 at cal start | Rotate manually to mid-range first |
| Calibration lost after restart | Negative offset uint16 overflow | Re-run calibration; monitor issue #1342 |
| Gripper extremely stiff then dead | Stall overload or cold damage | Replace servo; see prevention above |
| elbow_flex wrapped past 4095, stalled at hard stop | Calibration was 0-4095 (uncalibrated); target near max caused overshoot | Re-sweep calibration; now fixed in my_follower.json |

---

## 12. Teleoperation Control Loop & Cybernetics

### The Problem with Open-Loop Speed

Naively sending `goal_position` at a fixed speed has two failure modes:

1. **Teleporting** — follower starts far from where leader is; fixed speed means it rushes to catch up in one violent move
2. **Blind commanding** — script sends commands but never checks if the follower is actually tracking; stalls, wraps, and overload go undetected

### Proportional Speed Control (current implementation)

`scripts/teleop_6joint.py` uses a closed-loop speed command:

```
error  = desired_position - follower_actual_position
speed  = clamp(K_SPEED × |error| + FF_GAIN × |leader_velocity|, SPEED_MIN, SPEED_MAX)
write goal_speed = speed
write goal_position = desired
```

| Parameter | Value | Effect |
|-----------|-------|--------|
| `K_SPEED` | 1.2 | Speed counts per count of error |
| `FF_GAIN` | 0.6 | Leader velocity feedforward gain |
| `SPEED_MIN` | 80 | Minimum speed when nearly at target |
| `SPEED_MAX` | 500 | Maximum speed when far behind |

- **Small error** → slow speed → smooth fine tracking, no oscillation
- **Large error** → fast speed → closes gap quickly
- **Leader moving fast** → velocity feedforward adds speed → anticipates instead of reacting

Leader velocity is read from register 58 (`Present_Velocity`) each cycle and used as feedforward.

### Safety Layers (in order of application)

1. **Calibrated range margins** — 5% inside measured min/max on both leader and follower. Computed at startup from calibration JSON files.
2. **Startup equalization** — before tracking begins, follower moves slowly (speed=150) to match leader's current pose. Waits until all joints within 25 counts of target (up to 6s). Prevents violent initial jump.
3. **Proportional speed** — no per-cycle step cap needed; speed scales naturally with error.
4. **Encoder wrap detection** — if follower position jumps >1500 counts in one cycle, that joint is halted (torque off) and excluded from further commands.
5. **Load monitoring** — if any motor load >75%, full emergency stop (torque off all joints).

### Incident: elbow_flex encoder wrap (2026-05-14)

- **Calibration was 0–4095** (never swept). Target computed as 3890 (near max).
- Motor traveled from 2852 → 4068 → **wrapped to 5** → continued to ~160 (physical hard stop).
- Stalled at 160 with torque on for ~35s trying to reach 3890. Grinding sound. No damage (temp 29°C after).
- **Fix:** swept follower elbow_flex, now calibrated as 24–4051. With 5% margin, safe target ceiling is ~3845 — overshoot no longer possible.

### Calibration Is Safety

The control loop is only as safe as the calibration. Uncalibrated joints (0–4095) give the mapper permission to command the full encoder range, including positions past the physical hard stops.

**Before running teleop:**
1. Confirm calibration files exist and are not 0–4095 for any joint
2. Shared calibration at `~/so101/shared/calibration.json`
3. Run full health diagnostic (`/tmp/full_diagnostic.py`) to confirm motor temps are baseline

### Architecture Reference

```
Leader arm → read position (reg 56) + velocity (reg 58)
           ↓
     map_val(leader_pos, l_lo, l_hi, f_lo, f_hi)  ← calibrated safe ranges
           ↓
     error = target - follower_pos
     speed = K * error + FF * leader_vel            ← cybernetic loop
           ↓
Follower arm ← write goal_speed (reg 46) + goal_position (reg 42)
           ↓
     read follower_pos → compare to target → next cycle
```

Loop rate: ~10Hz. Full round-trip per joint: 2× read2ByteTxRx + 2× write2ByteTxRx ≈ 8ms.

