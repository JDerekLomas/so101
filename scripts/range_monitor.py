"""
Range-of-motion monitor — runs indefinitely, monitors both arms.

Usage:
    python ~/so101/scripts/range_monitor.py

Move both arms through their full range. Press Ctrl+C when done.
Results are printed and saved to ~/so101/range_results.json.

Ports are auto-detected via a 10-second wiggle test at startup
so leader/follower are never confused.
"""
import json
import signal
import sys
import time
from pathlib import Path
from scservo_sdk import PortHandler, PacketHandler

NAMES  = {1:"shoulder_pan", 2:"shoulder_lift", 3:"elbow_flex",
          4:"wrist_flex",   5:"wrist_roll",    6:"gripper"}
OUT    = Path.home() / "so101/range_results.json"

# Known port assignments (re-confirmed by wiggle test 2026-05-14).
# These SWAP on every replug — re-run wiggle test if arms are unplugged.
KNOWN_PORTS = {
    "/dev/tty.usbmodem5B141123331": "follower",
    "/dev/tty.usbmodem5B141116761": "leader",
}


def open_port(path):
    ph = PortHandler(path)
    h  = PacketHandler(0)
    if ph.openPort():
        ph.setBaudRate(1_000_000)
        return ph, h
    return None, None


def read_all(ph, h):
    positions = {}
    for mid in range(1, 7):
        val, comm, _ = h.read2ByteTxRx(ph, mid, 56)
        if comm == 0:
            positions[mid] = val
    return positions


def wiggle_identify(handlers, seconds=12):
    """Move each arm one at a time. Returns {port: 'follower'|'leader'}"""
    print(f"\n── Wiggle test ({seconds}s each) ─────────────────────────")

    labels = {}
    for role in ["follower", "leader"]:
        print(f"\n  Move the {role.upper()} arm now ({seconds}s)...", flush=True)
        prev = {p: read_all(ph, h) for p, (ph, h) in handlers.items()}
        activity = {p: 0 for p in handlers}
        deadline = time.time() + seconds
        while time.time() < deadline:
            for p, (ph, h) in handlers.items():
                cur = read_all(ph, h)
                for mid, val in cur.items():
                    if abs(val - prev[p].get(mid, val)) > 8:
                        activity[p] += 1
                prev[p] = {**prev[p], **read_all(ph, h)}
            time.sleep(0.05)

        active_port = max(activity, key=activity.get)
        if activity[active_port] == 0:
            print(f"  No movement detected — skipping {role}")
        else:
            labels[active_port] = role
            print(f"  {role} → {active_port}  ({activity[active_port]} changes)")

    # Assign any unidentified port
    for p in handlers:
        if p not in labels:
            other = [r for r in ["follower", "leader"] if r not in labels.values()]
            labels[p] = other[0] if other else "unknown"

    return labels


def main():
    # Open ports using known assignments
    handlers = {}
    port_labels = {}
    for p, role in KNOWN_PORTS.items():
        ph, h = open_port(p)
        if ph:
            handlers[p] = (ph, h)
            port_labels[p] = role
            print(f"  opened {p}  ({role})")
        else:
            print(f"  WARNING: could not open {p}  ({role})")

    if not handlers:
        print("No ports available. Check USB connections.")
        sys.exit(1)

    # Optional: verify with a quick wiggle test if both arms are present
    if len(handlers) > 1:
        print("\n  Tip: if arms are mixed up, unplug/replug and re-run wiggle_identify().")
    print(f"\n  Assignments: {port_labels}")

    # Initialize min/max tracking
    mins = {label: {NAMES[i]: 4095 for i in range(1, 7)} for label in port_labels.values()}
    maxs = {label: {NAMES[i]: 0    for i in range(1, 7)} for label in port_labels.values()}

    def update_ranges(label, positions):
        for mid, val in positions.items():
            name = NAMES[mid]
            if val < mins[label][name]: mins[label][name] = val
            if val > maxs[label][name]: maxs[label][name] = val

    def print_ranges():
        print("\n── Current ranges ───────────────────────────────────────")
        for label in sorted(mins.keys()):
            print(f"\n  {label.upper()}")
            for name in [NAMES[i] for i in range(1, 7)]:
                lo = mins[label][name]
                hi = maxs[label][name]
                span = hi - lo if hi > lo else 0
                bar = "█" * min(40, span // 100)
                print(f"    {name:<16} {lo:4d} – {hi:4d}  ({span:4d})  {bar}")

    def save_and_exit(sig=None, frame=None):
        print_ranges()
        results = {"generated": time.time(), "ranges": {}}
        for label in mins:
            results["ranges"][label] = {
                name: {"min": mins[label][name], "max": maxs[label][name]}
                for name in mins[label]
            }
        OUT.write_text(json.dumps(results, indent=2))
        print(f"\n  Saved to {OUT}")
        for ph, _ in handlers.values():
            ph.closePort()
        sys.exit(0)

    signal.signal(signal.SIGINT,  save_and_exit)
    signal.signal(signal.SIGTERM, save_and_exit)

    print("\n── Recording range of motion ────────────────────────────")
    print("  Move BOTH arms through their full range.")
    print("  Press Ctrl+C when done.\n")

    last_print = time.time()
    while True:
        for p, (ph, h) in handlers.items():
            label = port_labels.get(p, p)
            pos = read_all(ph, h)
            if pos:
                update_ranges(label, pos)

        if time.time() - last_print > 10:
            print_ranges()
            last_print = time.time()

        time.sleep(0.05)


if __name__ == "__main__":
    main()
