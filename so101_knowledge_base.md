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
8. [Useful Links](#8-useful-links)

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

**PID Registers**

| Register | Default Value | Notes |
|---|---|---|
| P_Coefficient | 16 | Lowered from factory 32 to reduce shakiness |
| I_Coefficient | 0 | |
| D_Coefficient | 32 | |

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
