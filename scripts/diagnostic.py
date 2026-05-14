"""
SO-101 Automated Diagnostic
Exercises each joint, measures tracking, checks all subsystems.
Outputs a clear pass/fail report.
"""
import requests, json, time, sys

BASE = "http://localhost:5833"
JOINT_IDS = {
    "shoulder_pan": 1, "shoulder_lift": 2, "elbow_flex": 3,
    "wrist_flex": 4, "wrist_roll": 5, "gripper": 6,
}

def api(method, path, data=None, timeout=3):
    url = f"{BASE}{path}"
    try:
        if method == "GET":
            r = requests.get(url, timeout=timeout)
        else:
            r = requests.post(url, json=data or {}, timeout=timeout)
        return r.json() if r.ok else {"error": r.status_code}
    except Exception as e:
        return {"error": str(e)}

def get_positions():
    return api("GET", "/api/positions")

def get_state():
    return api("GET", "/api/state")

def move_joint(joint_name, target, ramped=True):
    mid = JOINT_IDS[joint_name]
    return api("POST", "/api/move", {
        "label": "follower",
        "positions": {str(mid): target},
        "ramped": ramped,
    })

def wait_for_position(joint_name, target, timeout=8, tolerance=30):
    """Wait until joint reaches target within tolerance. Returns (reached, final_pos, elapsed)."""
    t0 = time.time()
    while time.time() - t0 < timeout:
        pos = get_positions()
        fp = pos.get("follower", {}).get(joint_name)
        if fp is not None and abs(fp - target) <= tolerance:
            return True, fp, time.time() - t0
        time.sleep(0.2)
    fp = get_positions().get("follower", {}).get(joint_name, "?")
    return False, fp, time.time() - t0

# ── Results collector ──
results = []
def test(name, passed, detail=""):
    status = "PASS" if passed else "FAIL"
    results.append((name, passed, detail))
    print(f"  [{status}] {name}" + (f" — {detail}" if detail else ""))

print("=" * 60)
print("SO-101 AUTOMATED DIAGNOSTIC")
print("=" * 60)

# ── 1. Server connectivity ──
print("\n1. SERVER CONNECTIVITY")
t0 = time.time()
state = get_state()
latency = (time.time() - t0) * 1000
test("Server reachable", "error" not in state, f"{latency:.0f}ms")

# ── 2. Board detection ──
print("\n2. BOARD DETECTION")
conn = state.get("connection", {})
follower_conn = conn.get("follower", {})
test("Follower connected", follower_conn.get("connected") == True,
     f"port={follower_conn.get('port','?')}, motors={follower_conn.get('motor_count','?')}")

motors = state.get("motors", {})
leader_motors = motors.get("leader", {})
test("Leader detected", len(leader_motors) > 0, f"{len(leader_motors)} motors")
follower_motors = motors.get("follower", {})
test("Follower motors", len(follower_motors) == 6, f"{len(follower_motors)}/6 motors")

# ── 3. Temperature check ──
print("\n3. TEMPERATURE CHECK")
for role in ("follower", "leader"):
    for jname, info in motors.get(role, {}).items():
        temp = info.get("temperature", 0)
        if role == "leader" and temp == 0:
            test(f"{role}/{jname} temp", True, f"0C (known — 7.4V variant reads 0)")
        else:
            ok = 0 < temp < 55
            test(f"{role}/{jname} temp", ok, f"{temp}C")

# ── 4. Calibration check ──
print("\n4. CALIBRATION")
cal = state.get("calibration", {})
test("Calibration recording", cal.get("recording") == True)

# Load cal ranges
try:
    r = requests.get(f"{BASE}/api/state", timeout=2)
    # Check if cal files exist by looking at position mapping ability
    pos = get_positions()
    test("Positions readable", "follower" in pos and "leader" in pos)
except:
    test("Positions readable", False, "API error")

# ── 5. Joint movement test ──
print("\n5. JOINT MOVEMENT TEST (follower)")
print("   Moving each joint ±100 counts from current position...")

pos = get_positions()
fpos = pos.get("follower", {})

# Stop teleop if running
if pos.get("teleop"):
    api("POST", "/api/teleop/stop")
    time.sleep(1)

joints_to_test = ["shoulder_pan", "shoulder_lift", "wrist_flex", "wrist_roll", "gripper"]
# Skip elbow_flex if it's near desk (< 300)
ef_pos = fpos.get("elbow_flex", 0)
if ef_pos > 300:
    joints_to_test.append("elbow_flex")
else:
    test("elbow_flex movement", False, f"SKIPPED — pos={ef_pos}, too close to desk")

for jname in joints_to_test:
    cur = fpos.get(jname)
    if cur is None:
        test(f"{jname} movement", False, "no position data")
        continue

    # Move +100 from current
    target = cur + 100
    move_joint(jname, target)
    reached, final, elapsed = wait_for_position(jname, target)

    if reached:
        # Move back
        move_joint(jname, cur)
        reached_back, final_back, elapsed_back = wait_for_position(jname, cur)
        err = abs(final_back - cur) if isinstance(final_back, (int, float)) else "?"
        test(f"{jname} movement", reached_back,
             f"+100 in {elapsed:.1f}s, back in {elapsed_back:.1f}s, final_err={err}")
    else:
        test(f"{jname} movement", False, f"target={target}, stuck at {final}, {elapsed:.1f}s")
        # Try to move back
        move_joint(jname, cur)
        time.sleep(2)

# ── 6. HTTP responsiveness under load ──
print("\n6. HTTP RESPONSIVENESS")
latencies = []
for _ in range(10):
    t0 = time.time()
    r = api("GET", "/api/positions")
    latencies.append((time.time() - t0) * 1000)
    time.sleep(0.05)
avg = sum(latencies) / len(latencies)
p95 = sorted(latencies)[int(len(latencies) * 0.95)]
test("Avg latency < 100ms", avg < 100, f"avg={avg:.0f}ms, p95={p95:.0f}ms")

# ── 7. Teleop start/stop ──
print("\n7. TELEOP START/STOP")
result = api("POST", "/api/teleop/start")
test("Teleop starts", result.get("ok") == True, f"phase={result.get('phase')}")
time.sleep(1)

# Check it's actually running
debug = api("GET", "/api/teleop/debug")
cycles = debug.get("cycle_count", 0)
time.sleep(1)
debug2 = api("GET", "/api/teleop/debug")
cycles2 = debug2.get("cycle_count", 0)
hz = cycles2 - cycles
test("Teleop running", hz > 5, f"{hz} cycles/sec")

result = api("POST", "/api/teleop/stop")
test("Teleop stops", result.get("ok") == True)

# ── Summary ──
print("\n" + "=" * 60)
passed = sum(1 for _, p, _ in results if p)
failed = sum(1 for _, p, _ in results if not p)
print(f"RESULTS: {passed} passed, {failed} failed, {len(results)} total")

if failed:
    print("\nFAILURES:")
    for name, p, detail in results:
        if not p:
            print(f"  - {name}: {detail}")

print("=" * 60)
