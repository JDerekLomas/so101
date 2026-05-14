"""
Full 6-joint teleoperation: leader -> follower, all joints.

Control loop (cybernetic):
  - Speed is proportional to tracking error: small error = slow, large = fast
  - Leader velocity (reg 58) read as feedforward — anticipates leader motion
  - Follower tracking error monitored every cycle, displayed
  - If follower falls >MAX_LAG counts behind target for >LAG_TIMEOUT cycles: warn

Safety layers:
  1. Calibrated range margins (5% each side)
  2. Startup equalization — follower moves to match leader before tracking starts
  3. Proportional speed (not fixed) — smooth, no teleporting
  4. Encoder wrap detection — halt joint if position jumps >1500 counts
  5. Load monitoring — emergency stop if load >75%

Usage: python3.12 teleop_6joint.py [duration_seconds]
"""
import os, sys, time, signal, json
sys.stderr = open(os.devnull, "w")
from scservo_sdk import PortHandler, PacketHandler

FOLLOWER_PORT = "/dev/tty.usbmodem5B141123331"
LEADER_PORT   = "/dev/tty.usbmodem5B141116761"
DURATION      = int(sys.argv[1]) if len(sys.argv) > 1 else 60
MARGIN        = 0.05

# Proportional speed control: speed = clamp(K_SPEED * error + feedforward, SPEED_MIN, SPEED_MAX)
K_SPEED    = 1.2    # counts of speed per count of error
SPEED_MIN  = 80     # minimum speed when nearly at target
SPEED_MAX  = 500    # maximum speed
FF_GAIN    = 0.6    # leader velocity feedforward gain

# Safety
LOAD_LIMIT    = 75   # % load -> emergency stop (any joint)
STALL_LOAD    = 50   # % load threshold for stall detection
STALL_MOVE    = 8    # counts — if position change < this while load > STALL_LOAD = stall
STALL_CYCLES  = 4    # consecutive cycles to confirm stall before halting joint
WRAP_THRESH   = 1500 # position jump -> encoder wrap, halt joint
MAX_LAG       = 200  # counts behind target before warning
LAG_TIMEOUT   = 15   # cycles behind before printing lag warning
DEADBAND      = 2    # counts — don't write if target unchanged within this

CAL_F = os.path.expanduser("~/.cache/huggingface/lerobot/calibration/robots/so_follower/my_follower.json")
CAL_L = os.path.expanduser("~/.cache/huggingface/lerobot/calibration/teleoperators/so_leader/my_leader.json")

with open(CAL_F) as f: cal_f = json.load(f)
with open(CAL_L) as f: cal_l = json.load(f)

JOINTS = {}
for name, ld in cal_l.items():
    if name not in cal_f:
        continue
    fd = cal_f[name]
    mid = fd["id"]
    l_min, l_max = ld["range_min"], ld["range_max"]
    f_min, f_max = fd["range_min"], fd["range_max"]
    span_l = l_max - l_min
    span_f = f_max - f_min
    JOINTS[name] = {
        "id":   mid,
        "l_lo": int(l_min + span_l * MARGIN),
        "l_hi": int(l_max - span_l * MARGIN),
        "f_lo": int(f_min + span_f * MARGIN),
        "f_hi": int(f_max - span_f * MARGIN),
    }

def map_val(v, in_lo, in_hi, out_lo, out_hi):
    v = max(in_lo, min(in_hi, v))
    return int(out_lo + (v - in_lo) / (in_hi - in_lo) * (out_hi - out_lo))

def clamp(v, lo, hi):
    return int(max(lo, min(hi, v)))

def read2(h, p, mid, addr):
    v, c, _ = h.read2ByteTxRx(p, mid, addr)
    return v if c == 0 else None

def read1(h, p, mid, addr):
    v, c, _ = h.read1ByteTxRx(p, mid, addr)
    return v if c == 0 else None

def prop_speed(error, leader_vel):
    """Proportional speed: small error = gentle, large error = fast. Feedforward from leader velocity."""
    base = K_SPEED * abs(error)
    ff   = FF_GAIN * abs(leader_vel) if leader_vel is not None else 0
    return clamp(int(base + ff), SPEED_MIN, SPEED_MAX)

# Open ports
lp = PortHandler(LEADER_PORT);   lh = PacketHandler(0)
fp = PortHandler(FOLLOWER_PORT);  fh = PacketHandler(0)
lp.openPort(); lp.setBaudRate(1_000_000)
fp.openPort(); fp.setBaudRate(1_000_000)
time.sleep(0.3)

running = True
halted_joints = set()

def shutdown(sig, frame):
    global running
    running = False

signal.signal(signal.SIGINT, shutdown)
signal.signal(signal.SIGTERM, shutdown)

def torque_all_off():
    for name, j in JOINTS.items():
        fh.write1ByteTxRx(fp, j["id"], 40, 0)

# ── Phase 0: Home follower to midrange (with torque) ──────────────────────────
# Follower falls under gravity when torque off — we bring it to safe midpoints
# before trying to match the leader. Slow speed, stall-safe.
print("=== PHASE 0: homing follower to safe midpoints ===")
for name, j in JOINTS.items():
    fh.write1ByteTxRx(fp, j["id"], 40, 1)
    fh.write2ByteTxRx(fp, j["id"], 46, 120)   # very slow

time.sleep(0.1)

home_targets = {name: (j["f_lo"] + j["f_hi"]) // 2 for name, j in JOINTS.items()}
print(f"  {'joint':<16} {'now':>6} {'home':>6}")
for name, j in JOINTS.items():
    mid = j["id"]
    fval = read2(fh, fp, mid, 56)
    htgt = home_targets[name]
    fh.write2ByteTxRx(fp, mid, 42, htgt)
    print(f"  {name:<16} {fval if fval else '?':>6} {htgt:>6}")

print("  Moving to midpoints (up to 8s, load-watched)...")
home_start = time.time()
home_stall = {name: 0 for name in JOINTS}
home_prev  = {}
while time.time() - home_start < 8.0:
    done = True
    for name, j in JOINTS.items():
        mid = j["id"]
        fval = read2(fh, fp, mid, 56)
        if fval is None: continue
        load_raw = read2(fh, fp, mid, 60)
        load = (load_raw & 0x3FF) / 10.0 if load_raw else 0
        prev = home_prev.get(name, fval)
        if load > STALL_LOAD and abs(fval - prev) < STALL_MOVE:
            home_stall[name] += 1
            if home_stall[name] >= STALL_CYCLES:
                print(f"  {name}: stall at {fval} — skipping homing for this joint")
                halted_joints.add(name)
                fh.write1ByteTxRx(fp, mid, 40, 0)
        else:
            home_stall[name] = 0
        home_prev[name] = fval
        if name not in halted_joints and abs(fval - home_targets[name]) > 25:
            done = False
    if done:
        break
    time.sleep(0.1)

print(f"  Homed in {time.time()-home_start:.1f}s\n")

# ── Phase 1: Startup equalization ─────────────────────────────────────────────
print("=== PHASE 1: equalization (match leader) ===")

for name, j in JOINTS.items():
    if name not in halted_joints:
        fh.write2ByteTxRx(fp, j["id"], 46, 150)  # slow speed for eq

time.sleep(0.1)

eq_targets = {}
LARGE_GAP   = 600   # counts — if follower this far from target, skip eq (needs manual positioning)

print(f"  {'joint':<16} {'f_now':>7} {'target':>7}  (leader)  note")
for name, j in JOINTS.items():
    mid = j["id"]
    lval = read2(lh, lp, mid, 56)
    fval = read2(fh, fp, mid, 56)
    if lval is None or fval is None:
        print(f"  {name:<16} READ ERROR")
        continue
    target = map_val(lval, j["l_lo"], j["l_hi"], j["f_lo"], j["f_hi"])
    gap = abs(fval - target)
    if gap > LARGE_GAP:
        # Skip equalization — joint too far away, would overload motor
        halted_joints.add(name)
        print(f"  {name:<16} {fval:>7} {target:>7}  (leader={lval})  SKIPPED: gap={gap} > {LARGE_GAP}, move manually to ~{target}")
        continue
    eq_targets[name] = target
    fh.write2ByteTxRx(fp, mid, 42, target)
    print(f"  {name:<16} {fval:>7} {target:>7}  (leader={lval})")

print("  Settling (up to 6s)...")
settle_start = time.time()
while time.time() - settle_start < 6.0:
    settled = True
    for name, j in JOINTS.items():
        if name not in eq_targets or name in halted_joints:
            continue
        mid = j["id"]
        fval = read2(fh, fp, mid, 56)
        if fval is None:
            continue
        err = abs(fval - eq_targets[name])
        if err > 25:
            settled = False
            # Check load during equalization too
            load_raw = read2(fh, fp, mid, 60)
            if load_raw is not None:
                load = (load_raw & 0x3FF) / 10.0
                if load > LOAD_LIMIT:
                    print(f"\n  !!! LOAD {load:.0f}% on {name} during equalization — halting joint")
                    halted_joints.add(name)
                    fh.write1ByteTxRx(fp, mid, 40, 0)
    if settled:
        break
    time.sleep(0.1)

print(f"  Settled in {time.time()-settle_start:.1f}s\n")

# ── Phase 2: Cybernetic tracking loop ─────────────────────────────────────────
print(f"=== TRACKING — {DURATION}s  (Ctrl-C to stop) ===")
print(f"  Speed: proportional (K={K_SPEED}, FF={FF_GAIN})  Load stop: {LOAD_LIMIT}%  Wrap halt: {WRAP_THRESH}")
print()
hdr = f"  {'joint':<14} {'leader':>6} {'lvel':>5} {'target':>7} {'fpos':>6} {'err':>5} {'spd':>4} {'load%':>6}"
print(hdr)
print(f"  {'-'*14} {'-'*6} {'-'*5} {'-'*7} {'-'*6} {'-'*5} {'-'*4} {'-'*6}")

last_targets  = {name: eq_targets.get(name, -9999) for name in JOINTS}
last_fpos     = {}
lag_counter   = {name: 0 for name in JOINTS}
stall_counter = {name: 0 for name in JOINTS}
start = time.time()

while running and (time.time() - start) < DURATION:
    tick_start = time.time()

    for name, j in JOINTS.items():
        mid = j["id"]

        if name in halted_joints:
            # Re-check: if follower has been moved close enough to target, re-engage
            fval = read2(fh, fp, mid, 56)
            lval = read2(lh, lp, mid, 56)
            if fval is not None and lval is not None:
                target_now = map_val(lval, j["l_lo"], j["l_hi"], j["f_lo"], j["f_hi"])
                if abs(fval - target_now) <= LARGE_GAP // 2:
                    halted_joints.discard(name)
                    fh.write1ByteTxRx(fp, mid, 40, 1)   # re-enable torque
                    fh.write2ByteTxRx(fp, mid, 46, SPEED_MIN)
                    last_fpos[name] = fval
                    print(f"\n  {name} re-engaged (gap now {abs(fval-target_now)})")
                else:
                    print(f"  {name:<14} {'HALTED':>10} fpos={fval} target={target_now} gap={abs(fval-target_now)} (move manually)")
            continue

        lval  = read2(lh, lp, mid, 56)   # leader position
        lvel  = read2(lh, lp, mid, 58)   # leader velocity (feedforward)
        fval  = read2(fh, fp, mid, 56)   # follower position
        if lval is None or fval is None:
            continue

        # Encoder wrap detection on follower
        prev_f = last_fpos.get(name, fval)
        if abs(fval - prev_f) > WRAP_THRESH:
            halted_joints.add(name)
            fh.write1ByteTxRx(fp, mid, 40, 0)
            print(f"\n  WRAP on {name}: {prev_f}->{fval}. HALTED.")
            last_fpos[name] = fval
            continue
        last_fpos[name] = fval

        # Load + stall detection
        load_raw = read2(fh, fp, mid, 60)
        load = (load_raw & 0x3FF) / 10.0 if load_raw is not None else 0

        # Global overload: emergency stop all
        if load > LOAD_LIMIT:
            running = False
            print(f"\n  !!! EMERGENCY STOP: {name} load={load:.0f}%")
            break

        # Stall detection: high load + no movement = grinding → halt this joint only
        pos_delta = abs(fval - last_fpos.get(name, fval))
        if load > STALL_LOAD and pos_delta < STALL_MOVE:
            stall_counter[name] += 1
            if stall_counter[name] >= STALL_CYCLES:
                halted_joints.add(name)
                fh.write1ByteTxRx(fp, mid, 40, 0)
                print(f"\n  STALL detected on {name} (load={load:.0f}%, move={pos_delta}cts). Joint halted.")
                stall_counter[name] = 0
                continue
        else:
            stall_counter[name] = 0

        # Map leader -> target
        target = map_val(lval, j["l_lo"], j["l_hi"], j["f_lo"], j["f_hi"])

        # Tracking error (follower behind target)
        error = target - fval

        # Proportional speed with leader velocity feedforward
        speed = prop_speed(error, lvel)

        # Lag monitor
        if abs(error) > MAX_LAG:
            lag_counter[name] += 1
            if lag_counter[name] == LAG_TIMEOUT:
                print(f"\n  LAG WARNING: {name} is {abs(error)} counts behind target")
        else:
            lag_counter[name] = 0

        # Write speed then position (only if meaningfully changed)
        if abs(target - last_targets.get(name, target)) >= DEADBAND:
            fh.write2ByteTxRx(fp, mid, 46, speed)
            fh.write2ByteTxRx(fp, mid, 42, target)
            last_targets[name] = target

        lvel_str = f"{lvel:5d}" if lvel is not None else "  ???"
        load_flag = " !" if load > 50 else ""
        print(f"  {name:<14} {lval:>6} {lvel_str} {target:>7} {fval:>6} {error:>+5} {speed:>4} {load:>5.1f}{load_flag}")

    # Pace loop to ~10Hz
    elapsed = time.time() - tick_start
    sleep_t = max(0, 0.1 - elapsed)
    if sleep_t > 0:
        time.sleep(sleep_t)

torque_all_off()
lp.closePort()
fp.closePort()
print("\nDone. Torque off.")
