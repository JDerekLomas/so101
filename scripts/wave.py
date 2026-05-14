#!/usr/bin/env python3
"""Cybernetic wave: sense-act-verify at every step."""
import time
import sys
sys.path.insert(0, "/Users/dereklomas/lerobot-env-312/lib/python3.12/site-packages")
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

TEMP_LIMIT = 60
LOAD_WARN = 800

def write(ph, h, mid, addr, val, length=2):
    try:
        if length == 1:
            h.write1ByteTxRx(ph, mid, addr, val)
        else:
            h.write2ByteTxRx(ph, mid, addr, val)
        time.sleep(0.002)
    except Exception:
        time.sleep(0.02)

def read2(ph, h, mid, addr):
    try:
        val, _, _ = h.read2ByteTxRx(ph, mid, addr)
        time.sleep(0.002)
        return val
    except Exception:
        return None

def read1(ph, h, mid, addr):
    try:
        val, _, _ = h.read1ByteTxRx(ph, mid, addr)
        time.sleep(0.002)
        return val
    except Exception:
        return None

def sense_all(ph, h):
    state = {}
    for name, mid in IDS.items():
        pos = read2(ph, h, mid, PRESENT_POSITION)
        raw_load = read2(ph, h, mid, PRESENT_LOAD)
        # STS3215: bit 10 = direction, bits 0-9 = magnitude
        load = (raw_load & 0x3FF) if raw_load is not None else None
        temp = read1(ph, h, mid, PRESENT_TEMP)
        state[name] = {"pos": pos, "load": load, "temp": temp}
    return state

def check_safety(state):
    problems = []
    for name, s in state.items():
        if s["temp"] is not None and s["temp"] > TEMP_LIMIT:
            problems.append(f"{name} HOT: {s['temp']}C")
        if s["load"] is not None and s["load"] > LOAD_WARN:
            problems.append(f"{name} OVERLOAD: {s['load']}")
    return problems

def move_and_verify(ph, h, targets, speed=200, timeout=8.0, label=""):
    for name, target in targets.items():
        mid = IDS[name]
        write(ph, h, mid, TORQUE_ENABLE, 1, length=1)
        write(ph, h, mid, SPEED_ADDR, speed)
        write(ph, h, mid, GOAL_POSITION, target)

    start = time.time()
    arrived = {name: False for name in targets}

    while time.time() - start < timeout:
        state = sense_all(ph, h)
        problems = check_safety(state)
        if problems:
            print(f"  SAFETY STOP: {problems}")
            for name in targets:
                write(ph, h, IDS[name], TORQUE_ENABLE, 0, length=1)
            return arrived

        status_parts = []
        all_done = True
        for name, target in targets.items():
            pos = state[name]["pos"]
            load = state[name]["load"] or 0
            if pos is not None:
                gap = abs(pos - target)
                if gap < 30:
                    arrived[name] = True
                else:
                    all_done = False
                status_parts.append(f"{name[:8]}={pos:>5}(g={gap:>4},L={load:>3})")

        print(f"  [{label}] {' '.join(status_parts)}")

        if all_done:
            print(f"  [{label}] ARRIVED")
            return arrived

        time.sleep(0.2)

    for name, ok in arrived.items():
        if not ok:
            print(f"  [{label}] TIMEOUT: {name} didn't reach {targets[name]}")
    return arrived

def main():
    # Stop the server first to avoid port conflict
    import urllib.request
    try:
        urllib.request.urlopen("http://127.0.0.1:5833/api/teleop/stop",
                               data=b'{}', timeout=2)
    except Exception:
        pass

    ph = PortHandler(PORT)
    if not ph.openPort():
        print("Failed to open port — is the server still using it?")
        return
    ph.setBaudRate(BAUD)
    h = PacketHandler(PROTOCOL)

    # === SENSE ===
    print("=== SENSE ===")
    state = sense_all(ph, h)
    zero_count = 0
    for name, s in state.items():
        p = s['pos'] if s['pos'] is not None else '?'
        t = s['temp'] if s['temp'] is not None else '?'
        l = s['load'] if s['load'] is not None else '?'
        print(f"  {name:15s}  pos={str(p):>5}  load={str(l):>4}  temp={t}C")
        if s['pos'] == 0 and s['load'] == 0 and s['temp'] == 0:
            zero_count += 1
    if zero_count >= 3:
        print("ABORT: bus dead (too many zero reads). Replug USB or wait.")
        ph.closePort()
        return
    problems = check_safety(state)
    if problems:
        print(f"ABORT: {problems}")
        ph.closePort()
        return

    # === PHASE 1: Raise shoulder only (elbow swings free under gravity) ===
    print("\n=== PHASE 1: raise shoulder ===")
    # Raise shoulder high — needs to be high enough that elbow clears the desk
    result = move_and_verify(ph, h, {"shoulder_lift": 1900}, speed=300, timeout=8, label="raise")

    # Sense where elbow ended up
    state = sense_all(ph, h)
    elbow_pos = state["elbow_flex"]["pos"]
    print(f"\n  Elbow swung to: {elbow_pos} (was on desk at ~150)")

    # Skip elbow — it's in the encoder wrap zone near 0/4095

    # === PHASE 2: Straighten all joints ===
    # Elbow encoder is inverted: high counts = physically up, low = desk
    # From elbow ~50 (desk), we need ~3500 (arm straight up)
    print("\n=== PHASE 2: stand straight ===")
    state = sense_all(ph, h)
    ef_now = state["elbow_flex"]["pos"] or 0
    # Skip elbow — it's in the encoder wrap zone and can't be commanded safely.
    # The raised shoulder + gravity lets it hang naturally.
    print(f"  Elbow at {ef_now} (letting it hang free)")
    result = move_and_verify(ph, h, {
        "shoulder_lift": 1900,
        "wrist_flex": 2100,
        "wrist_roll": 2030,
        "gripper": 2400,
    }, speed=300, timeout=6, label="stand")

    # Verify
    state = sense_all(ph, h)
    sl = state["shoulder_lift"]["pos"]
    standing = sl is not None and abs(sl - 1900) < 100
    print(f"\n  Shoulder up: {'YES' if standing else 'NO'} (sl={sl})")

    if not standing:
        print("  Can't raise shoulder — aborting wave.")
        for mid in IDS.values():
            write(ph, h, mid, TORQUE_ENABLE, 0, length=1)
        ph.closePort()
        return

    # === PHASE 3: Wave ===
    print("\n=== PHASE 3: wave! ===")
    for i in range(6):
        target = 2800 if i % 2 == 0 else 1200
        write(ph, h, IDS["wrist_roll"], SPEED_ADDR, 800)
        write(ph, h, IDS["wrist_roll"], GOAL_POSITION, target)
        # Hold shoulder while waving
        write(ph, h, IDS["shoulder_lift"], GOAL_POSITION, 1900)

        time.sleep(0.35)
        state = sense_all(ph, h)
        wr = state["wrist_roll"]
        print(f"  wave {i+1}: wr={wr['pos']:>5}(target={target}) load={wr['load']}")
        problems = check_safety(state)
        if problems:
            print(f"  SAFETY STOP: {problems}")
            break
        time.sleep(0.15)

    # Return
    write(ph, h, IDS["wrist_roll"], SPEED_ADDR, 300)
    write(ph, h, IDS["wrist_roll"], GOAL_POSITION, 2030)
    time.sleep(0.5)

    # === FINAL SENSE ===
    print("\n=== FINAL STATE ===")
    state = sense_all(ph, h)
    for name, s in state.items():
        p = s['pos'] if s['pos'] is not None else '?'
        print(f"  {name:15s}  pos={str(p):>5}  temp={s['temp']}C")

    print("\nTorque off.")
    for mid in IDS.values():
        write(ph, h, mid, TORQUE_ENABLE, 0, length=1)
    ph.closePort()

if __name__ == "__main__":
    main()
