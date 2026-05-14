#!/usr/bin/env python3
"""Stand the follower arm up straight with cybernetic monitoring.

Uses direct serial — bypasses the server to avoid lock contention.
Sense-act-verify at every step. Reports intention-action mismatch.

KEY INSIGHT: elbow_flex encoder wraps at 0/4095. When arm is on desk,
elbow reads ~165. When shoulder lifts high enough, elbow swings through
gravity and wraps to ~3400+. Must raise shoulder FIRST, let elbow swing,
THEN command elbow from its post-swing position.
"""
import sys
sys.path.insert(0, "/Users/dereklomas/lerobot-env-312/lib/python3.12/site-packages")

import time
import urllib.request

from scservo_sdk import PortHandler, PacketHandler

PORT = "/dev/tty.usbmodem5B141123331"  # follower
BAUD = 1000000
PROTOCOL = 0

TORQUE_ENABLE = 40
GOAL_POSITION = 42
PRESENT_POSITION = 56
PRESENT_LOAD = 60
PRESENT_TEMP = 63
SPEED_ADDR = 46

IDS = {"shoulder_pan": 1, "shoulder_lift": 2, "elbow_flex": 3,
       "wrist_flex": 4, "wrist_roll": 5, "gripper": 6}

TEMP_LIMIT = 55
COLLISION_LOAD = 500
SPEED = 300

# Track last known-good temps for spike filtering
_last_temps = {}


def write2(ph, h, mid, addr, val):
    ph.is_using = False
    h.write2ByteTxRx(ph, mid, addr, val)
    time.sleep(0.005)

def write1(ph, h, mid, addr, val):
    ph.is_using = False
    h.write1ByteTxRx(ph, mid, addr, val)
    time.sleep(0.005)

def read2(ph, h, mid, addr, retries=2):
    for attempt in range(retries):
        ph.is_using = False
        try:
            val, _, _ = h.read2ByteTxRx(ph, mid, addr)
            time.sleep(0.005)
            if val == 0 and attempt < retries - 1:
                time.sleep(0.01)
                continue  # retry zero reads
            return val
        except Exception:
            time.sleep(0.02)
    return None

def read1(ph, h, mid, addr, retries=2):
    for attempt in range(retries):
        ph.is_using = False
        try:
            val, _, _ = h.read1ByteTxRx(ph, mid, addr)
            time.sleep(0.005)
            if val == 0 and attempt < retries - 1:
                time.sleep(0.01)
                continue
            return val
        except Exception:
            time.sleep(0.02)
    return None

def sense_all(ph, h):
    state = {}
    for name, mid in IDS.items():
        pos = read2(ph, h, mid, PRESENT_POSITION)
        raw_load = read2(ph, h, mid, PRESENT_LOAD)
        load = (raw_load & 0x3FF) if raw_load is not None else 0
        temp = read1(ph, h, mid, PRESENT_TEMP)
        # Filter temp spikes (corrupt bus reads)
        if temp is not None and name in _last_temps:
            if abs(temp - _last_temps[name]) > 15:
                temp = _last_temps[name]  # reject spike, use last good
        if temp is not None and temp > 0:
            _last_temps[name] = temp
        time.sleep(0.003)  # inter-motor gap to reduce bus contention
        state[name] = {"pos": pos, "load": load, "temp": temp}
    return state

def move_and_monitor(ph, h, targets, label, speed=SPEED, timeout=10):
    """Cybernetic move: sense-act-verify with intention-action tracking."""
    print(f"\n{'='*60}")
    print(f"INTENTION: {label}")
    state = sense_all(ph, h)
    for name, target in targets.items():
        cur = state[name]["pos"] or 0
        print(f"  {name}: {cur} -> {target} (delta={target-cur})")

    # Enable torque + set speed + command
    for name, target in targets.items():
        mid = IDS[name]
        write1(ph, h, mid, TORQUE_ENABLE, 1)
        write2(ph, h, mid, SPEED_ADDR, speed)
        write2(ph, h, mid, GOAL_POSITION, target)

    # Monitor loop
    start = time.time()
    arrived = {name: False for name in targets}
    adaptive_k = {name: 1.0 for name in targets}
    error_hist = {name: [] for name in targets}
    cycle = 0

    while time.time() - start < timeout:
        cycle += 1
        state = sense_all(ph, h)

        parts = []
        all_done = True
        for name, target in targets.items():
            mid = IDS[name]
            pos = state[name]["pos"] or 0
            load = state[name]["load"]
            temp = state[name]["temp"]
            error = abs(pos - target)

            # Error history for adaptive gain
            error_hist[name].append(error)
            if len(error_hist[name]) > 20:
                error_hist[name].pop(0)

            # Adaptive gain
            k = adaptive_k[name]
            if len(error_hist[name]) >= 5:
                recent = sum(error_hist[name][-3:]) / 3
                avg = sum(error_hist[name]) / len(error_hist[name])
                if recent > avg * 1.1 and recent > 30:
                    k = min(2.5, k + 0.05)
                elif recent < 20:
                    k = max(0.5, k - 0.1)
            adaptive_k[name] = k

            # Safety
            if temp is not None and temp > TEMP_LIMIT:
                print(f"  SAFETY: {name} HOT {temp}C")
                write1(ph, h, mid, TORQUE_ENABLE, 0)
                arrived[name] = True
                continue
            if load > COLLISION_LOAD:
                print(f"  COLLISION: {name} load={load} pos={pos}")
                write2(ph, h, mid, GOAL_POSITION, pos)
                arrived[name] = True
                continue

            # Predictive slowdown
            if load > COLLISION_LOAD * 0.7:
                write2(ph, h, mid, SPEED_ADDR, max(80, speed // 2))
            else:
                new_speed = int(max(80, min(800, k * error + 50)))
                write2(ph, h, mid, SPEED_ADDR, new_speed)

            if error < 30:
                arrived[name] = True
            else:
                all_done = False

            sym = "OK" if error < 30 else f"K={k:.1f}"
            parts.append(f"{name[:8]}={pos:>5}(e={error:>4} L={load:>3} {sym})")

        if cycle % 5 == 1:
            print(f"  [{label}] c={cycle} {' '.join(parts)}")

        if all_done:
            elapsed = time.time() - start
            print(f"  [{label}] ARRIVED in {elapsed:.1f}s")
            break
        time.sleep(0.15)

    # Report
    print(f"  RESULT:")
    state = sense_all(ph, h)
    for name, target in targets.items():
        pos = state[name]["pos"] or 0
        gap = abs(pos - target)
        status = "MATCH" if gap < 40 else f"MISMATCH gap={gap}"
        print(f"    {name}: intended={target} actual={pos} K={adaptive_k[name]:.2f} -> {status}")
    return arrived


def main():
    # Stop server teleop
    try:
        req = urllib.request.Request("http://127.0.0.1:5833/api/teleop/stop",
                                     data=b'{}', headers={"Content-Type": "application/json"})
        urllib.request.urlopen(req, timeout=2)
    except Exception:
        pass

    ph = PortHandler(PORT)
    if not ph.openPort():
        print("Failed to open port — kill the server first:")
        print("  pkill -f app.py")
        return
    ph.setBaudRate(BAUD)
    h = PacketHandler(PROTOCOL)

    print("="*60)
    print("SO-101 STAND UP — Cybernetic Diagnostic")
    print("="*60)
    state = sense_all(ph, h)
    print("\nInitial state:")
    for name, s in state.items():
        p = s['pos'] if s['pos'] is not None else '?'
        print(f"  {name:15s}  pos={str(p):>5}  load={s['load']:>4}  temp={s['temp']}C")

    zeros = sum(1 for s in state.values()
                if s['pos'] is not None and s['pos'] == 0 and s['load'] == 0)
    if zeros >= 3:
        print("ABORT: bus dead"); ph.closePort(); return

    # ════════════════════════════════════════════════════════════
    # Phase 1: Raise shoulder HIGH (2300) so elbow swings free
    # The elbow encoder wraps at 0/4095. On the desk it reads ~165.
    # Raising shoulder lets gravity swing the elbow through the
    # wrap point to ~3400+.
    # ════════════════════════════════════════════════════════════
    move_and_monitor(ph, h,
        {"shoulder_lift": 2300},
        "Phase 1: RAISE SHOULDER HIGH", speed=500, timeout=8)

    # Wait for elbow to settle after gravity swing
    print("\n  Waiting for elbow to settle after gravity swing...")
    time.sleep(1.0)
    for i in range(5):
        ef = read2(ph, h, IDS["elbow_flex"], PRESENT_POSITION)
        print(f"    elbow_flex = {ef}")
        time.sleep(0.3)

    ef_settled = read2(ph, h, IDS["elbow_flex"], PRESENT_POSITION)
    print(f"  Elbow settled at: {ef_settled} (was ~165 on desk)")
    if ef_settled is not None and ef_settled > 3000:
        print(f"  Elbow wrapped through 4095->0 and swung to {ef_settled}. Good.")
    elif ef_settled is not None and ef_settled < 500:
        print(f"  Elbow near bottom ({ef_settled}). May still be on desk or just past wrap.")

    # ════════════════════════════════════════════════════════════
    # Phase 2: Command elbow to straight-up position
    # With shoulder at 2300 and elbow swung free (~3400),
    # straight arm is around 3800-4000 (toward 4095).
    # ════════════════════════════════════════════════════════════
    # Pick elbow target based on where it settled
    if ef_settled is not None and ef_settled > 2000:
        elbow_target = 3900  # toward straight up
    else:
        elbow_target = 2500  # fallback

    move_and_monitor(ph, h, {
        "elbow_flex":   elbow_target,
        "wrist_flex":   2048,
        "wrist_roll":   2048,
        "gripper":      2400,
    }, "Phase 2: STRAIGHTEN ARM", speed=300, timeout=8)

    # ════════════════════════════════════════════════════════════
    # Phase 3: Fine-tune shoulder + center pan
    # ════════════════════════════════════════════════════════════
    move_and_monitor(ph, h, {
        "shoulder_pan":  2048,
        "shoulder_lift": 2000,
    }, "Phase 3: CENTER & ADJUST", speed=200, timeout=5)

    # ════════════════════════════════════════════════════════════
    # Phase 4: Wave!
    # ════════════════════════════════════════════════════════════
    print(f"\n{'='*60}")
    print("Phase 4: WAVE")
    print("="*60)

    # Re-enable torque on all holding joints
    for name in ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex"]:
        mid = IDS[name]
        write1(ph, h, mid, TORQUE_ENABLE, 1)
    write2(ph, h, IDS["shoulder_lift"], GOAL_POSITION, 2000)
    write2(ph, h, IDS["shoulder_pan"], GOAL_POSITION, 2048)
    ef_cur = read2(ph, h, IDS["elbow_flex"], PRESENT_POSITION) or elbow_target
    write2(ph, h, IDS["elbow_flex"], GOAL_POSITION, ef_cur)
    write2(ph, h, IDS["wrist_flex"], GOAL_POSITION, 2048)

    # Enable wrist_roll for waving
    write1(ph, h, IDS["wrist_roll"], TORQUE_ENABLE, 1)

    for i in range(6):
        target = 2800 if i % 2 == 0 else 1200
        write2(ph, h, IDS["wrist_roll"], SPEED_ADDR, 800)
        write2(ph, h, IDS["wrist_roll"], GOAL_POSITION, target)
        time.sleep(0.6)
        wr_pos = read2(ph, h, IDS["wrist_roll"], PRESENT_POSITION)
        wr_load_raw = read2(ph, h, IDS["wrist_roll"], PRESENT_LOAD)
        wr_load = (wr_load_raw & 0x3FF) if wr_load_raw else 0
        print(f"  wave {i+1}: wrist_roll={wr_pos:>5} (target={target}) load={wr_load}")
        if wr_load > COLLISION_LOAD:
            print("  COLLISION — stopping wave")
            break

    # Return wrist
    write2(ph, h, IDS["wrist_roll"], SPEED_ADDR, 300)
    write2(ph, h, IDS["wrist_roll"], GOAL_POSITION, 2048)
    time.sleep(0.5)

    # ════════════════════════════════════════════════════════════
    # Final state
    # ════════════════════════════════════════════════════════════
    print(f"\n{'='*60}")
    print("FINAL STATE")
    print("="*60)
    state = sense_all(ph, h)
    for name, s in state.items():
        p = s['pos'] if s['pos'] is not None else '?'
        print(f"  {name:15s}  pos={str(p):>5}  load={s['load']:>4}  temp={s['temp']}C")

    print("\nTorque off.")
    for mid in IDS.values():
        write1(ph, h, mid, TORQUE_ENABLE, 0)
    ph.closePort()


if __name__ == "__main__":
    main()
