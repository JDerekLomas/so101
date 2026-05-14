"""
Identify which USB port is the leader and which is the follower.

Because both controller boards share the same USB serial number, macOS assigns
port names non-deterministically on replug. This script scans all connected
boards, streams live motor positions, and asks you to wiggle each arm so you
can confirm the assignment — then writes robot.env automatically.

Usage:
    source ~/so101/robot-workspace/lerobot-env-312/bin/activate   # or wherever your env is
    python ~/so101/robot-workspace/detect_ports.py
"""

import glob
import os
import sys
import time
from pathlib import Path

try:
    from scservo_sdk import PortHandler, PacketHandler
except ImportError:
    print("scservo_sdk not found. Activate your lerobot env first:")
    print("  source ~/lerobot-env-312/bin/activate")
    sys.exit(1)

ENV_FILE = Path(__file__).parent / "robot.env"
PRESENT_POSITION_ADDR = 56
BAUDRATE = 1_000_000
ID_NAMES = {1: "shoulder_pan", 2: "shoulder_lift", 3: "elbow_flex",
            4: "wrist_flex", 5: "wrist_roll", 6: "gripper"}


def open_port(path):
    ph = PortHandler(path)
    if ph.openPort():
        ph.setBaudRate(BAUDRATE)
        return ph
    return None


def ping_all(ph, handler):
    found = []
    for mid in range(1, 20):
        _, result, _ = handler.ping(ph, mid)
        if result == 0:
            found.append(mid)
    return found


def read_positions(ph, handler, ids):
    positions = {}
    for mid in ids:
        val, result, _ = handler.read2ByteTxRx(ph, mid, PRESENT_POSITION_ADDR)
        if result == 0:
            positions[mid] = val & 0x0FFF
    return positions


def detect_movement(ph, handler, ids, samples=8, interval=0.05):
    """Return total positional variance across all motors over a short window."""
    readings = []
    for _ in range(samples):
        pos = read_positions(ph, handler, ids)
        readings.append(pos)
        time.sleep(interval)
    total_delta = 0
    for mid in ids:
        vals = [r[mid] for r in readings if mid in r]
        if len(vals) >= 2:
            total_delta += max(vals) - min(vals)
    return total_delta


def find_boards():
    paths = sorted(glob.glob("/dev/tty.usbmodem*"))
    boards = {}
    for path in paths:
        ph = open_port(path)
        if ph is None:
            continue
        handler = PacketHandler(0)
        ids = ping_all(ph, handler)
        if ids:
            boards[path] = {"ph": ph, "handler": handler, "ids": ids}
        else:
            ph.closePort()
    return boards


def close_all(boards):
    for b in boards.values():
        try:
            b["ph"].closePort()
        except Exception:
            pass


def write_env(follower_port, leader_port):
    content = f"""# Robot Configuration
# Source this file before running robot commands:
#   source ~/so101/robot-workspace/robot.env

ROBOT_ENV=~/lerobot-env-312
FOLLOWER_PORT={follower_port}
LEADER_PORT={leader_port}
ROBOT_ID=my_follower
TELEOP_ID=my_leader
"""
    ENV_FILE.write_text(content)
    print(f"\nWritten to {ENV_FILE}")


def main():
    print("Scanning for controller boards...\n")
    boards = find_boards()

    if not boards:
        print("No boards found. Check USB connections.")
        sys.exit(1)

    if len(boards) == 1:
        port = list(boards.keys())[0]
        ids = boards[port]["ids"]
        print(f"Only one board found: {port}  ({len(ids)} motors: {ids})")
        close_all(boards)
        sys.exit(0)

    ports = list(boards.keys())
    print(f"Found {len(ports)} boards:")
    for p in ports:
        ids = boards[p]["ids"]
        print(f"  {p}: {len(ids)} motors responding  (IDs: {ids})")

    if len(ports) > 2:
        print("\nMore than 2 boards found — only 2-arm setups are supported.")
        close_all(boards)
        sys.exit(1)

    port_a, port_b = ports[0], ports[1]

    # ------------------------------------------------------------------ #
    # Step 1: wiggle the LEADER arm                                       #
    # ------------------------------------------------------------------ #
    print(f"\n--- Step 1 of 2: Identify the LEADER arm ---")
    print("Physically wiggle the LEADER arm (the one you hold) back and forth.")
    input("Press Enter when ready, then wiggle for ~2 seconds...")

    delta_a = detect_movement(boards[port_a]["ph"], boards[port_a]["handler"], boards[port_a]["ids"])
    delta_b = detect_movement(boards[port_b]["ph"], boards[port_b]["handler"], boards[port_b]["ids"])

    if delta_a == delta_b == 0:
        print("No movement detected on either port. Try wiggling more vigorously.")
        print("Falling back to manual assignment.")
        leader_port = input(f"Which port is the LEADER? [{port_a} / {port_b}]: ").strip()
        follower_port = port_b if leader_port == port_a else port_a
    else:
        leader_port = port_a if delta_a > delta_b else port_b
        follower_port = port_b if leader_port == port_a else port_a
        print(f"  Movement detected — {port_a}: {delta_a} ticks  |  {port_b}: {delta_b} ticks")
        print(f"  Leader identified as: {leader_port}")

    # ------------------------------------------------------------------ #
    # Step 2: confirm with FOLLOWER                                       #
    # ------------------------------------------------------------------ #
    print(f"\n--- Step 2 of 2: Confirm the FOLLOWER arm ---")
    print(f"That means the FOLLOWER should be: {follower_port}")
    print("Wiggle the FOLLOWER arm now to confirm.")
    input("Press Enter when ready, then wiggle for ~2 seconds...")

    delta_follower = detect_movement(
        boards[follower_port]["ph"], boards[follower_port]["handler"], boards[follower_port]["ids"]
    )

    if delta_follower > 10:
        print(f"  Movement confirmed on {follower_port} ({delta_follower} ticks). Assignment looks correct.")
    else:
        print(f"  Low movement on {follower_port} ({delta_follower} ticks).")
        swap = input("  Swap leader/follower assignment? [y/N]: ").strip().lower()
        if swap == "y":
            leader_port, follower_port = follower_port, leader_port
            print(f"  Swapped. Leader: {leader_port}  Follower: {follower_port}")

    close_all(boards)

    # ------------------------------------------------------------------ #
    # Summary + write robot.env                                           #
    # ------------------------------------------------------------------ #
    print(f"\nAssignment:")
    print(f"  Follower: {follower_port}")
    print(f"  Leader:   {leader_port}")

    save = input("\nWrite this to robot.env? [Y/n]: ").strip().lower()
    if save != "n":
        write_env(follower_port, leader_port)
        print("\nDone. Run:  source ~/so101/robot-workspace/robot.env")
    else:
        print("Not saved.")


if __name__ == "__main__":
    main()
