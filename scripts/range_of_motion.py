"""
Range-of-motion recorder — runs in background, writes state to file.
Stop it by creating the STOP_FILE path, or kill the process.
"""
import time
import json
import os
import sys
from scservo_sdk import PortHandler, PacketHandler

PORT = "/dev/tty.usbmodem5B141123331"
ADDR_PRESENT_POSITION = 56
NAMES = {1: "shoulder_pan", 2: "shoulder_lift", 3: "elbow_flex",
         4: "wrist_flex", 5: "wrist_roll", 6: "gripper"}
STATE_FILE = "/private/tmp/claude-501/rom_state.json"
STOP_FILE  = "/private/tmp/claude-501/rom_stop"

port = PortHandler(PORT)
if not port.openPort():
    sys.exit(1)
port.setBaudRate(1_000_000)
handler = PacketHandler(0)

mins = {i: 4095 for i in range(1, 7)}
maxs = {i: 0    for i in range(1, 7)}

if os.path.exists(STOP_FILE):
    os.remove(STOP_FILE)

while not os.path.exists(STOP_FILE):
    pos = {}
    for mid in range(1, 7):
        val, comm, _ = handler.read2ByteTxRx(port, mid, ADDR_PRESENT_POSITION)
        if comm == 0:
            pos[mid] = val
            if val < mins[mid]: mins[mid] = val
            if val > maxs[mid]: maxs[mid] = val

    state = {
        NAMES[i]: {"pos": pos.get(i), "min": mins[i], "max": maxs[i]}
        for i in range(1, 7)
    }
    with open(STATE_FILE, "w") as f:
        json.dump(state, f)

    time.sleep(0.05)

# Save final calibration
cal_path = os.path.expanduser(
    "~/.cache/huggingface/lerobot/calibration/robots/so_follower/my_follower.json"
)
try:
    with open(cal_path) as f:
        cal = json.load(f)
    for mid, name in NAMES.items():
        if name in cal:
            cal[name]["range_min"] = mins[mid]
            cal[name]["range_max"] = maxs[mid]
    with open(cal_path, "w") as f:
        json.dump(cal, f, indent=4)
except Exception:
    pass

port.closePort()
