"""
Live range calibration — move all joints through their full range on both arms.
Press Ctrl+C when done. Calibration files will be updated automatically.
"""
import json
import time
import signal
import sys
from pathlib import Path
from scservo_sdk import PortHandler, PacketHandler

ARMS = {
    "follower": {
        "port": "/dev/tty.usbmodem5B141123331",
        "cal_path": Path.home() / ".cache/huggingface/lerobot/calibration/robots/so_follower/my_follower.json",
    },
    "leader": {
        "port": "/dev/tty.usbmodem5B141116761",
        "cal_path": Path.home() / ".cache/huggingface/lerobot/calibration/teleoperators/so_leader/my_leader.json",
    },
}

ID_NAMES = {1: "shoulder_pan", 2: "shoulder_lift", 3: "elbow_flex",
            4: "wrist_flex", 5: "wrist_roll", 6: "gripper"}

def read_positions(port, handler):
    positions = {}
    for motor_id in range(1, 7):
        try:
            val, result, _ = handler.read2ByteTxRx(port, motor_id, 56)
            if result == 0:
                positions[motor_id] = val & 0x0FFF  # 12-bit position
        except Exception:
            pass
    return positions

def open_port(port_path):
    try:
        p = PortHandler(port_path)
        if p.openPort():
            p.setBaudRate(1_000_000)
            return p
    except Exception:
        pass
    return None

# Open ports
ports = {}
handlers = {}
for arm, cfg in ARMS.items():
    p = open_port(cfg["port"])
    if p:
        ports[arm] = p
        handlers[arm] = PacketHandler(0)
        print(f"Connected: {arm} ({cfg['port']})")
    else:
        print(f"Not connected: {arm}")

if not ports:
    print("No arms connected.")
    sys.exit(1)

# Track min/max per arm per motor
ranges = {arm: {mid: {"min": 9999, "max": -9999} for mid in range(1, 7)} for arm in ports}

print("\nMove ALL joints through their FULL range on both arms.")
print("Press Ctrl+C when done.\n")

def save_and_exit(sig=None, frame=None):
    print("\n\nSaving calibration...")
    for arm, cfg in ARMS.items():
        if arm not in ports:
            continue
        cal_path = cfg["cal_path"]
        if not cal_path.exists():
            print(f"  Skipping {arm} — no calibration file at {cal_path}")
            continue
        with open(cal_path) as f:
            cal = json.load(f)
        updated = []
        for motor_id, name in ID_NAMES.items():
            if name not in cal:
                continue
            r = ranges[arm][motor_id]
            if r["min"] == 9999 or r["max"] == -9999:
                print(f"  {arm}/{name}: no data, skipping")
                continue
            old = f"[{cal[name]['range_min']},{cal[name]['range_max']}]"
            cal[name]["range_min"] = r["min"]
            cal[name]["range_max"] = r["max"]
            updated.append(f"  {arm}/{name}: {old} → [{r['min']},{r['max']}]")
        with open(cal_path, "w") as f:
            json.dump(cal, f, indent=4)
        for line in updated:
            print(line)
        print(f"  Saved: {cal_path}")
    for p in ports.values():
        try:
            p.closePort()
        except Exception:
            pass
    sys.exit(0)

signal.signal(signal.SIGINT, save_and_exit)
signal.signal(signal.SIGTERM, save_and_exit)

# Main loop
while True:
    line_parts = []
    for arm in list(ports.keys()):
        try:
            positions = read_positions(ports[arm], handlers[arm])
            for mid, pos in positions.items():
                ranges[arm][mid]["min"] = min(ranges[arm][mid]["min"], pos)
                ranges[arm][mid]["max"] = max(ranges[arm][mid]["max"], pos)
            part = f"{arm}: " + " ".join(f"{ID_NAMES[m]}={v}" for m, v in sorted(positions.items()))
            line_parts.append(part)
        except Exception:
            line_parts.append(f"{arm}: disconnected")
    print("\r" + " | ".join(line_parts) + "   ", end="", flush=True)
    time.sleep(0.05)
