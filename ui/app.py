"""
SO-101 web tool — port detection + range calibration + shared state.

Usage:
    source ~/lerobot-env-312/bin/activate
    python ~/so101/robot-workspace/port_detector/app.py
    # Open http://localhost:5833
"""

import glob
import json
import os
import threading
import time
from pathlib import Path

import cv2
from flask import Flask, Response, jsonify, request, send_from_directory

try:
    from scservo_sdk import PacketHandler, PortHandler, GroupSyncWrite, SCS_LOBYTE, SCS_HIBYTE
    SDK_AVAILABLE = True
except ImportError:
    SDK_AVAILABLE = False

app = Flask(__name__, static_folder="static")

ENV_FILE    = Path.home() / "so101/robot.env"
SHARED_DIR  = Path.home() / "so101/shared"
STATE_FILE  = SHARED_DIR / "robot_state.json"
DATA_LOG    = Path.home() / "so101/shared/motor_data.jsonl"  # continuous log
MSG_IN      = SHARED_DIR / "messages" / "to_robot"
MSG_OUT     = SHARED_DIR / "messages" / "to_kb"
MSG_TO_USER = SHARED_DIR / "messages" / "to_user"
MSG_FROM_USER = SHARED_DIR / "messages" / "from_user"

BAUDRATE             = 1_000_000
PRESENT_POSITION_ADDR = 56
GOAL_POSITION_ADDR   = 42
SPEED_ADDR           = 46
PRESENT_SPEED_ADDR   = 58
TORQUE_ENABLE_ADDR   = 40
TEMPERATURE_ADDR     = 63
LOAD_ADDR            = 60
P_GAIN_ADDR          = 29
D_GAIN_ADDR          = 27
PING_INTERVAL        = 0.05
STATE_WRITE_INTERVAL = 0.5   # write shared state at 2 Hz
AUTO_SCAN_INTERVAL   = 3.0   # check for new/gone boards every 3s

# ── Safety thresholds (from shared/safety.json) ───────────────────
SAFETY_FILE = Path(__file__).parent.parent / "shared" / "safety.json"
try:
    _safety = json.loads(SAFETY_FILE.read_text())
    TEMP_CUTOFF_C             = _safety["thresholds"]["temp_cutoff_c"]
    STALL_LOAD_THRESHOLD      = _safety["thresholds"]["stall_load_threshold"]
    STALL_COUNT_LIMIT         = _safety["thresholds"]["stall_count_limit"]
    COLLISION_LOAD_THRESHOLD  = _safety["thresholds"]["collision_load_threshold"]
    COLLISION_RETREAT_LOAD    = _safety["thresholds"]["collision_retreat_load"]
except Exception:
    TEMP_CUTOFF_C             = 65
    STALL_LOAD_THRESHOLD      = 800
    STALL_COUNT_LIMIT         = 6
    COLLISION_LOAD_THRESHOLD  = 350
    COLLISION_RETREAT_LOAD    = 150

CAL_PATHS = {
    "follower": Path.home() / ".cache/huggingface/lerobot/calibration/robots/so_follower/my_follower.json",
    "leader":   Path.home() / ".cache/huggingface/lerobot/calibration/teleoperators/so_leader/my_leader.json",
}
CAL_HISTORY_DIR = Path.home() / "so101/shared/calibration_history"
CAL_HISTORY_DIR.mkdir(parents=True, exist_ok=True)
CAL_RANGES_FILE = Path.home() / "so101/shared/cal_ranges_live.json"  # persistent always-on ranges

ID_NAMES = {
    1: "shoulder_pan",
    2: "shoulder_lift",
    3: "elbow_flex",
    4: "wrist_flex",
    5: "wrist_roll",
    6: "gripper",
}

state_lock      = threading.Lock()
boards          = {}   # port -> {ph, handler, ids, positions, temps, loads, errors, label}
assignment      = {}   # "leader"/"follower" -> port
move_grace      = {}   # motor_id -> timestamp — suppress stall detection until this time
MOVE_GRACE_S    = 2.0  # seconds to suppress stall detection after a commanded move
cal_ranges      = {}   # role -> {motor_id -> {min, max}}  — always-on tracking
event_log       = []   # recent events, capped at 50
notifications   = []   # messages shown in UI, capped at 20
empty_ports     = set()  # USB ports that are connected but have no motors (don't re-probe every 3s)
teleop_active      = False
teleop_equalizing  = False  # True during startup equalization phase
teleop_eq_targets  = {}     # {motor_id: target} — snapshot of leader pos at start
teleop_cal         = {}     # loaded once on start: {"leader": {...}, "follower": {...}}
teleop_positions   = {}     # {motor_id: current_sent_position} — used for ramp-limiting
teleop_prev_pos    = {}     # {motor_id: last_read_follower_pos} — for wrap detection
teleop_halted      = set()  # motor IDs halted due to encoder wrap this session
teleop_collision   = {}     # {motor_id: frozen_position} — joints frozen due to collision load
teleop_slow_mode   = False  # True = conservative limits for initial exploration
teleop_log         = []     # rolling buffer of teleop cycle snapshots

# Camera state
camera_frame      = None   # latest JPEG bytes
camera_lock       = threading.Lock()
camera_active     = False
TELEOP_LOG_MAX     = 200    # keep last N cycles (~10s at 20Hz)
teleop_cycle_count = 0      # total cycles since teleop started

TELEOP_WRAP_THRESH = 1500   # counts — position jump this large = encoder wrap, halt joint
TELEOP_LARGE_GAP   = 600   # counts — skip equalization if follower too far from target
CAL_LEARN_MARGIN   = 100   # counts — buffer beyond collision point when tightening limits

# Max counts to move per poll cycle (20 Hz). 60 counts/step = 1200 counts/s ≈ 3.4 s full sweep.
TELEOP_MAX_DELTA   = 150   # counts/cycle — 3000 counts/s at 20Hz, full sweep in ~1.2s
TELEOP_EQ_THRESH   = 25    # counts — follower must be within this of target to end equalization

# Proportional speed control for teleop (matches teleop_6joint.py)
TELEOP_SPEED_K     = 1.2   # speed units per count of error
TELEOP_SPEED_FF    = 0.6   # feedforward gain from leader velocity
TELEOP_SPEED_MIN   = 80    # minimum speed (keeps motion smooth near target)
TELEOP_SPEED_MAX   = 800   # maximum speed (tracks fast leader movements)

# Slow-mode overrides — gentle exploration (half speed, quarter delta)
TELEOP_SLOW_MAX_DELTA  = 15   # counts/cycle → ~300 counts/s = very slow sweep
TELEOP_SLOW_SPEED_MIN  = 30
TELEOP_SLOW_SPEED_MAX  = 100


# ------------------------------------------------------------------ #
# Persistent calibration range tracking                               #
# ------------------------------------------------------------------ #

def _load_persistent_cal_ranges():
    """Load accumulated cal ranges from disk so they survive restarts."""
    global cal_ranges
    try:
        if CAL_RANGES_FILE.exists():
            data = json.loads(CAL_RANGES_FILE.read_text())
            # Convert string motor IDs back to ints
            for role, motors in data.get("ranges", {}).items():
                cal_ranges[role] = {int(mid): v for mid, v in motors.items()}
            print(f"  Loaded persistent cal ranges from {CAL_RANGES_FILE.name}")
            for role, motors in cal_ranges.items():
                for mid, r in motors.items():
                    name = ID_NAMES.get(mid, str(mid))
                    spread = r["max"] - r["min"]
                    if spread > 50:
                        print(f"    {role}/{name}: {r['min']}-{r['max']} ({spread} ticks)")
    except Exception as e:
        print(f"  Could not load persistent cal ranges: {e}")


def _save_persistent_cal_ranges():
    """Write accumulated cal ranges to disk. Called periodically from poll loop."""
    try:
        with state_lock:
            snapshot = {role: {str(mid): dict(v) for mid, v in motors.items()}
                        for role, motors in cal_ranges.items()}
        if not snapshot:
            return
        data = {
            "_updated": time.time(),
            "_doc": "Always-on calibration range tracking. Accumulates across server restarts.",
            "ranges": snapshot,
        }
        tmp = CAL_RANGES_FILE.with_suffix(".tmp")
        tmp.write_text(json.dumps(data, indent=2))
        tmp.replace(CAL_RANGES_FILE)
    except Exception:
        pass


# ------------------------------------------------------------------ #
# Events                                                              #
# ------------------------------------------------------------------ #

def push_event(kind, detail):
    entry = {"ts": time.time(), "kind": kind, "detail": detail}
    with state_lock:
        event_log.append(entry)
        if len(event_log) > 50:
            event_log.pop(0)
    drop_message("to_kb", kind, detail)


def drop_message(direction, msg_type, payload):
    ts = time.time()
    msg_id = f"{int(ts)}_{msg_type}"
    path = SHARED_DIR / "messages" / direction / f"{msg_id}.json"
    try:
        path.write_text(json.dumps({
            "id": msg_id, "from": "robot", "to": "kb",
            "type": msg_type, "ts": ts, "payload": payload,
        }, indent=2))
    except Exception:
        pass


# ------------------------------------------------------------------ #
# Hardware helpers                                                    #
# ------------------------------------------------------------------ #

def open_port(path):
    ph = PortHandler(path)
    if ph.openPort():
        ph.setBaudRate(BAUDRATE)
        return ph
    return None


def ping_ids(ph, handler):
    found = []
    for mid in range(1, 20):
        _, result, _ = handler.ping(ph, mid)
        if result == 0:
            found.append(mid)
    return found


def read_register(ph, handler, mid, addr, length=2, retries=3):
    for _ in range(retries):
        try:
            if length == 1:
                val, result, _ = handler.read1ByteTxRx(ph, mid, addr)
            else:
                val, result, _ = handler.read2ByteTxRx(ph, mid, addr)
            if result == 0:
                return val
        except Exception:
            time.sleep(0.1)
    return None


def write_register(ph, handler, mid, addr, value, length=2):
    """Fire-and-forget write — uses TxOnly so it never blocks waiting for a status packet.
    Servos with return-level=1 (lerobot default) don't send status on writes, so TxRx
    would time out on every call and stall the loop."""
    try:
        if length == 1:
            result = handler.write1ByteTxOnly(ph, mid, addr, value)
        else:
            result = handler.write2ByteTxOnly(ph, mid, addr, value)
        return result == 0
    except Exception:
        return False


def _load_teleop_cal():
    """Load both cal files for position mapping. Returns {role: {name: {...}}}."""
    result = {}
    for role, path in CAL_PATHS.items():
        try:
            result[role] = json.loads(path.read_text())
        except Exception:
            pass
    return result


def _raw_to_calibrated(raw, cal):
    """Convert raw encoder value to calibrated position using homing offset and drive mode.
    This is how lerobot normalizes positions — the homing offset determines the zero point
    and drive_mode can invert the direction."""
    offset = cal.get("homing_offset", 0)
    drive = cal.get("drive_mode", 0)
    if drive == 0:
        return raw - offset
    else:
        return offset - raw


def _calibrated_to_raw(calibrated, cal):
    """Inverse of _raw_to_calibrated."""
    offset = cal.get("homing_offset", 0)
    drive = cal.get("drive_mode", 0)
    if drive == 0:
        return calibrated + offset
    else:
        return offset - calibrated


def _map_position(leader_raw, lc, fc):
    """Map leader raw position to follower raw position via calibrated space.

    1. Convert leader raw → calibrated (applying homing offset + drive mode)
    2. Normalize to [0, 1] within leader's calibrated range
    3. Denormalize to follower's calibrated range
    4. Convert follower calibrated → raw

    This correctly handles joints where leader and follower homing offsets
    have different signs (physically mirrored directions)."""
    # Leader: raw → calibrated
    l_cal = _raw_to_calibrated(leader_raw, lc)
    l_min_cal = _raw_to_calibrated(lc["range_min"], lc)
    l_max_cal = _raw_to_calibrated(lc["range_max"], lc)
    # Ensure min < max in calibrated space
    if l_min_cal > l_max_cal:
        l_min_cal, l_max_cal = l_max_cal, l_min_cal

    # Normalize to [0, 1]
    l_span = l_max_cal - l_min_cal
    if l_span < 1:
        t = 0.5
    else:
        t = (l_cal - l_min_cal) / l_span
    t = max(0.0, min(1.0, t))

    # Follower: calibrated range
    f_min_cal = _raw_to_calibrated(fc["range_min"], fc)
    f_max_cal = _raw_to_calibrated(fc["range_max"], fc)
    if f_min_cal > f_max_cal:
        f_min_cal, f_max_cal = f_max_cal, f_min_cal

    # Denormalize to follower calibrated space
    f_cal = f_min_cal + t * (f_max_cal - f_min_cal)

    # Follower: calibrated → raw
    f_raw = _calibrated_to_raw(f_cal, fc)

    # Clamp to raw range
    f_raw_min = min(fc["range_min"], fc["range_max"])
    f_raw_max = max(fc["range_min"], fc["range_max"])
    return round(max(f_raw_min, min(f_raw_max, f_raw)))


def _apply_workspace_limits(position, joint_name, label="follower"):
    """Clamp position to workspace limits from safety.json (P9: environment-aware)."""
    try:
        ws = _safety.get("workspace", {}).get(label, {}).get(joint_name, {})
        if "hard_min" in ws:
            position = max(ws["hard_min"], position)
        if "hard_max" in ws:
            position = min(ws["hard_max"], position)
    except Exception:
        pass
    return position


def _load_cal_limits(role):
    """Return {motor_id: (range_min, range_max)} from calibration file, or {}."""
    cal_file = CAL_PATHS.get(role)
    if cal_file is None or not cal_file.exists():
        return {}
    try:
        cal = json.loads(cal_file.read_text())
        return {
            info["id"]: (info["range_min"], info["range_max"])
            for info in cal.values()
            if "id" in info and "range_min" in info and "range_max" in info
        }
    except Exception:
        return {}


def _check_safety(board, path, pending_events):
    """Called INSIDE state_lock. Appends (kind, detail) to pending_events — do not call push_event here."""
    ph, handler = board["ph"], board["handler"]
    for mid in board["ids"]:
        if board["safety_disabled"].get(mid):
            continue  # already tripped — don't re-check until explicitly cleared

        name = ID_NAMES.get(mid, str(mid))

        # ── 1. Temperature cutoff ────────────────────────────────────
        temp = board["temps"].get(mid)
        if temp is not None and temp >= TEMP_CUTOFF_C:
            write_register(ph, handler, mid, TORQUE_ENABLE_ADDR, 0, length=1)
            board["safety_disabled"][mid] = True
            board["safety_cause"][mid]    = "temp"
            board["stall_counts"][mid]    = 0
            pending_events.append(("safety_temp_cutoff", {
                "port": path, "motor": mid, "name": name, "temp": temp,
            }))
            continue  # skip stall check for this motor

        # ── 2. Stall detection ───────────────────────────────────────
        # Skip stall trips while teleop is active — position tracking causes
        # brief high load that should not be treated as a stall.
        if teleop_active and board.get("label") == "follower":
            board["stall_counts"][mid] = 0
            continue

        # Skip stall detection during move grace period (P5: allow recovery
        # from stalled positions — motor needs time to push through high-load zone)
        if move_grace.get(mid, 0) > time.time():
            board["stall_counts"][mid] = 0
            continue

        load = board["loads"].get(mid)
        if load is None:
            # Comm error — reset counter (don't false-trip on packet loss)
            board["stall_counts"][mid] = 0
        elif load > STALL_LOAD_THRESHOLD:
            board["stall_counts"][mid] = board["stall_counts"].get(mid, 0) + 1
            if board["stall_counts"][mid] >= STALL_COUNT_LIMIT:
                write_register(ph, handler, mid, TORQUE_ENABLE_ADDR, 0, length=1)
                board["safety_disabled"][mid] = True
                board["safety_cause"][mid]    = "stall"
                pending_events.append(("safety_stall_trip", {
                    "port": path, "motor": mid, "name": name,
                    "load": load, "count": board["stall_counts"][mid],
                }))
        else:
            board["stall_counts"][mid] = 0  # load is fine — reset counter


def scan_boards():
    global empty_ports
    paths = sorted(glob.glob("/dev/tty.usbmodem*"))
    found = {}
    new_empty = set()
    for path in paths:
        ph = open_port(path)
        if ph is None:
            continue
        handler = PacketHandler(0)
        ids = ping_ids(ph, handler)
        if ids:
            found[path] = {
                "ph": ph, "handler": handler, "ids": ids,
                "positions":       {mid: 0     for mid in ids},
                "temps":           {mid: None  for mid in ids},
                "loads":           {mid: None  for mid in ids},
                "stall_counts":    {mid: 0     for mid in ids},
                "safety_disabled": {mid: False for mid in ids},
                "safety_cause":    {mid: None  for mid in ids},  # "temp" | "stall" | None
                "errors":    0,
                "label":     None,
            }
        else:
            ph.closePort()
            new_empty.add(path)
    empty_ports = new_empty
    return found


# ------------------------------------------------------------------ #
# Adaptive calibration learning                                       #
# ------------------------------------------------------------------ #

def _learn_collision_limit(joint_name, collision_pos):
    """Update follower calibration range to exclude a collision position.

    Called when the teleop collision detector freezes a joint (load > threshold).
    Permanently tightens the calibration range so future sessions never command
    the arm to that position again. Implements P5+: learn from obstacles.

    collision_pos near range_min → raise range_min
    collision_pos near range_max → lower range_max
    """
    global teleop_cal, teleop_eq_targets

    cal_file = CAL_PATHS.get("follower")
    if not cal_file or not cal_file.exists():
        return
    try:
        cal = json.loads(cal_file.read_text())
        if joint_name not in cal:
            return
        entry = cal[joint_name]
        midpoint = (entry["range_min"] + entry["range_max"]) / 2

        if collision_pos <= midpoint:
            new_val = int(collision_pos + CAL_LEARN_MARGIN)
            if new_val <= entry["range_min"]:
                return  # already tighter, no change needed
            old_val = entry["range_min"]
            entry["range_min"] = new_val
            boundary = "min"
        else:
            new_val = int(collision_pos - CAL_LEARN_MARGIN)
            if new_val >= entry["range_max"]:
                return
            old_val = entry["range_max"]
            entry["range_max"] = new_val
            boundary = "max"

        # Write calibration file
        cal_file.write_text(json.dumps(cal, indent=2))

        # Update shared calibration JSON too
        shared_cal = Path.home() / "so101/shared/calibration.json"
        try:
            if shared_cal.exists():
                sc = json.loads(shared_cal.read_text())
                if "follower" in sc and joint_name in sc["follower"]:
                    sc["follower"][joint_name]["range_min"] = entry["range_min"]
                    sc["follower"][joint_name]["range_max"] = entry["range_max"]
                    shared_cal.write_text(json.dumps(sc, indent=2))
        except Exception:
            pass

        # Update in-memory teleop_cal so THIS session uses new limits immediately
        with state_lock:
            if "follower" in teleop_cal and joint_name in teleop_cal["follower"]:
                teleop_cal["follower"][joint_name]["range_min"] = entry["range_min"]
                teleop_cal["follower"][joint_name]["range_max"] = entry["range_max"]
            # Clamp any existing equalization target for this joint inside new range
            mid = next((k for k, v in ID_NAMES.items() if v == joint_name), None)
            if mid is not None and mid in teleop_eq_targets:
                clamped = max(entry["range_min"], min(entry["range_max"], teleop_eq_targets[mid]))
                teleop_eq_targets[mid] = clamped

        push_event("cal_limit_learned", {
            "joint": joint_name,
            "boundary": boundary,
            "collision_pos": int(collision_pos),
            "old": old_val,
            "new": new_val,
        })
    except Exception:
        pass  # never let learning failures break anything


# ------------------------------------------------------------------ #
# Data logging                                                        #
# ------------------------------------------------------------------ #

_log_file = None

def _log_motor_data(ts, pending_events):
    """Append one JSONL record per poll cycle: all motors + any events."""
    global _log_file
    try:
        if _log_file is None:
            _log_file = open(DATA_LOG, "a", buffering=1)  # line-buffered

        with state_lock:
            snapshot = {}
            for path, board in boards.items():
                role = board.get("label")
                if not role:
                    continue
                snapshot[role] = {}
                for mid in board["ids"]:
                    name = ID_NAMES.get(mid, str(mid))
                    snapshot[role][name] = {
                        "pos":  board["positions"].get(mid),
                        "load": board["loads"].get(mid),
                        "temp": board["temps"].get(mid),
                    }
            t_active = teleop_active
            t_eq     = teleop_equalizing

        record = {
            "ts":     round(ts, 3),
            "teleop": t_active,
            "eq":     t_eq,
            "motors": snapshot,
        }
        if pending_events:
            record["events"] = [{"kind": k, "detail": d} for k, d in pending_events]

        _log_file.write(json.dumps(record) + "\n")
    except Exception:
        _log_file = None  # reopen next cycle if file handle broke


# ------------------------------------------------------------------ #
# Background threads                                                  #
# ------------------------------------------------------------------ #

def poll_loop():
    """Read positions, temps, loads from all boards continuously."""
    global teleop_active, teleop_equalizing, teleop_eq_targets, teleop_cal, teleop_positions, teleop_prev_pos, teleop_halted, teleop_collision, teleop_slow_mode, teleop_cycle_count
    last_state_write = 0
    last_auto_scan   = 0
    last_tl_read     = 0   # last time we read temps+loads

    while True:
        now = time.time()
        pending_events = []  # safety events to push after lock is released

        # Auto-scan: detect boards appearing or disappearing
        if now - last_auto_scan >= AUTO_SCAN_INTERVAL:
            last_auto_scan = now
            try:
                _auto_scan()
            except Exception:
                pass  # SerialException during teleop — don't kill the thread

        do_tl = (now - last_tl_read >= 0.25)  # temps+loads at ~4 Hz
        if do_tl:
            last_tl_read = now

        with state_lock:
            for path, board in list(boards.items()):
                try:
                    ph, handler, ids = board["ph"], board["handler"], board["ids"]
                    ph.is_using = False  # reset stale port lock from prior cycle
                    for mid in ids:
                        pos = read_register(ph, handler, mid, PRESENT_POSITION_ADDR, 2)
                        if pos is not None:
                            board["positions"][mid] = pos & 0x0FFF
                            board["errors"] = max(0, board["errors"] - 1)
                        else:
                            board["errors"] = min(board["errors"] + 1, 100)  # cap at 100

                    # Read temps + loads at ~4 Hz
                    if do_tl:
                        for mid in ids:
                            t = read_register(ph, handler, mid, TEMPERATURE_ADDR, 1)
                            l = read_register(ph, handler, mid, LOAD_ADDR, 2)
                            if t is not None:
                                board["temps"][mid] = t
                            if l is not None:
                                board["loads"][mid] = l & 0x03FF

                        # Safety checks run right after fresh temp+load data
                        _check_safety(board, path, pending_events)

                    # Calibration range tracking — always on
                    if board["label"] in ("leader", "follower"):
                        role = board["label"]
                        if role not in cal_ranges:
                            cal_ranges[role] = {}
                        for mid, val in board["positions"].items():
                            if mid not in cal_ranges[role]:
                                cal_ranges[role][mid] = {"min": val, "max": val}
                            else:
                                cal_ranges[role][mid]["min"] = min(cal_ranges[role][mid]["min"], val)
                                cal_ranges[role][mid]["max"] = max(cal_ranges[role][mid]["max"], val)
                except Exception:
                    pass

            # ── Teleop: mirror leader -> follower at 20 Hz ───────────
            # Uses GroupSyncWrite (broadcast, no status packet expected) so writes
            # never block on timeout. Individual write2ByteTxRx times out when
            # return-level=1 (lerobot default), stalling the loop.
            if teleop_active:
                l_port = assignment.get("leader")
                f_port = assignment.get("follower")
                if l_port and f_port and l_port in boards and f_port in boards:
                    l_board = boards[l_port]
                    f_board = boards[f_port]
                    lc_map  = teleop_cal.get("leader",   {})
                    fc_map  = teleop_cal.get("follower", {})
                    ph      = f_board["ph"]
                    handler = f_board["handler"]
                    all_equalized = True
                    max_delta = TELEOP_SLOW_MAX_DELTA if teleop_slow_mode else TELEOP_MAX_DELTA

                    # Compute goal positions for all active joints this cycle
                    goals = {}  # mid -> target position (int)
                    skip_reasons = {}  # mid -> reason string (for debug)

                    for mid in f_board["ids"]:
                        name = ID_NAMES.get(mid, str(mid))

                        # ── Encoder wrap detection ────────────────────────
                        cur_pos = f_board["positions"].get(mid)
                        prev_pos = teleop_prev_pos.get(mid)
                        if cur_pos is not None and prev_pos is not None:
                            if abs(cur_pos - prev_pos) > TELEOP_WRAP_THRESH:
                                if mid not in teleop_halted:
                                    teleop_halted.add(mid)
                                    write_register(ph, handler, mid, TORQUE_ENABLE_ADDR, 0, length=1)
                                    pending_events.append(("teleop_wrap_halt", {
                                        "motor": mid, "name": name,
                                        "prev": prev_pos, "cur": cur_pos,
                                        "jump": cur_pos - prev_pos,
                                    }))
                        if cur_pos is not None:
                            teleop_prev_pos[mid] = cur_pos

                        # Skip halted joints
                        if mid in teleop_halted:
                            skip_reasons[mid] = "wrap_halted"
                            continue

                        # Clear safety trips — re-enable if temperature has dropped
                        if f_board["safety_disabled"].get(mid):
                            if f_board["safety_cause"].get(mid) == "temp":
                                cur_temp = f_board["temps"].get(mid)
                                if cur_temp is not None and cur_temp >= TEMP_CUTOFF_C:
                                    skip_reasons[mid] = f"temp_cutoff(t={cur_temp})"
                                    continue  # still too hot, skip
                                # Temp has dropped — clear the trip and re-enable
                                print(f"[teleop] {ID_NAMES.get(mid,'?')} temp recovered ({cur_temp}C < {TEMP_CUTOFF_C}C), re-enabling")
                            f_board["safety_disabled"][mid] = False
                            f_board["safety_cause"][mid]    = None
                            f_board["stall_counts"][mid]    = 0
                            write_register(ph, handler, mid, TORQUE_ENABLE_ADDR, 1, length=1)

                        name = ID_NAMES.get(mid)
                        if not name:
                            skip_reasons[mid] = "no_name"
                            continue
                        lc = lc_map.get(name)
                        fc = fc_map.get(name)
                        if lc is None or fc is None:
                            skip_reasons[mid] = f"no_cal(lc={'Y' if lc else 'N'},fc={'Y' if fc else 'N'})"
                            continue

                        # ── Collision detection: freeze joint if load too high ──
                        # Use higher threshold during equalization (gravity + ramp),
                        # normal threshold during live tracking.
                        f_load = f_board["loads"].get(mid)
                        col_thresh = COLLISION_LOAD_THRESHOLD * 2 if teleop_equalizing else COLLISION_LOAD_THRESHOLD
                        if f_load is not None and f_load > col_thresh:
                            if mid not in teleop_collision:
                                frozen_pos = f_board["positions"].get(mid, teleop_positions.get(mid, 0))
                                teleop_collision[mid] = frozen_pos
                                pending_events.append(("teleop_collision", {
                                    "motor": mid, "name": name, "load": f_load,
                                    "frozen_at": frozen_pos,
                                }))
                            goals[mid] = int(teleop_collision[mid])
                            continue
                        elif mid in teleop_collision:
                            if f_load is not None and f_load < COLLISION_RETREAT_LOAD:
                                del teleop_collision[mid]
                                pending_events.append(("teleop_collision_cleared", {
                                    "motor": mid, "name": name, "load": f_load,
                                }))
                            else:
                                goals[mid] = int(teleop_collision[mid])
                                continue

                        if teleop_equalizing:
                            target = teleop_eq_targets.get(mid)
                            if target is None:
                                all_equalized = False
                                continue
                            prev = teleop_positions.get(mid, f_board["positions"].get(mid, target))
                            delta = target - prev
                            if abs(delta) > max_delta:
                                target = prev + max_delta * (1 if delta > 0 else -1)
                                all_equalized = False
                            elif abs(target - f_board["positions"].get(mid, target)) > TELEOP_EQ_THRESH:
                                all_equalized = False
                        else:
                            l_raw = l_board["positions"].get(mid)
                            if l_raw is None:
                                continue
                            target = _map_position(l_raw, lc, fc)
                            prev = teleop_positions.get(mid, f_board["positions"].get(mid, target))
                            delta = target - prev
                            if abs(delta) > max_delta:
                                target = prev + max_delta * (1 if delta > 0 else -1)

                        # P9: workspace limits — clamp to environment boundaries
                        target = _apply_workspace_limits(target, name, "follower")
                        teleop_positions[mid] = target
                        goals[mid] = int(target)

                    # ── Proportional speed per joint ─────────────────────────
                    speeds = {}  # mid -> speed value
                    cycle_details = {}  # mid -> {pos, target, error, speed, ...} for logging
                    spd_min = TELEOP_SLOW_SPEED_MIN if teleop_slow_mode else TELEOP_SPEED_MIN
                    spd_max = TELEOP_SLOW_SPEED_MAX if teleop_slow_mode else TELEOP_SPEED_MAX
                    for mid, target_pos in goals.items():
                        current_pos = f_board["positions"].get(mid, target_pos)
                        error = abs(target_pos - current_pos)
                        leader_vel = 0  # velocity feedforward removed — read2ByteTxRx blocks loop
                        speed = int(max(spd_min, min(spd_max,
                                        TELEOP_SPEED_K * error + TELEOP_SPEED_FF * leader_vel)))
                        speeds[mid] = speed
                        name = ID_NAMES.get(mid, str(mid))
                        cycle_details[name] = {
                            "pos": current_pos, "target": target_pos,
                            "error": int(error), "speed": speed, "lvel": leader_vel,
                        }

                    # ── Reset port state before writes ─────────────────────
                    # The SDK's txPacket() checks port.is_using and returns
                    # COMM_PORT_BUSY if True. After read retries/exceptions,
                    # is_using can be left True permanently. We're inside
                    # state_lock so no other thread can be using this port.
                    ph.is_using = False
                    try:
                        ph.ser.reset_input_buffer()  # flush stale RX from reads
                    except Exception:
                        pass

                    # ── Write speeds via GroupSyncWrite (broadcast, no ACK) ──
                    if speeds and SDK_AVAILABLE:
                        gsw_spd = GroupSyncWrite(ph, handler, SPEED_ADDR, 2)
                        for mid, spd in speeds.items():
                            gsw_spd.addParam(mid, [SCS_LOBYTE(spd), SCS_HIBYTE(spd)])
                        gsw_spd.txPacket()
                        gsw_spd.clearParam()

                    # ── Write goal positions via GroupSyncWrite ──
                    gsw_ok = -1
                    if goals and SDK_AVAILABLE:
                        gsw = GroupSyncWrite(ph, handler, GOAL_POSITION_ADDR, 2)
                        for mid, pos in goals.items():
                            gsw.addParam(mid, [SCS_LOBYTE(pos), SCS_HIBYTE(pos)])
                        gsw_ok = gsw.txPacket()
                        gsw.clearParam()

                    # ── Teleop cycle logging ─────────────────────────────────
                    teleop_cycle_count += 1
                    phase = "eq" if teleop_equalizing else "track"
                    log_entry = {"t": round(now, 3), "cycle": teleop_cycle_count,
                                 "phase": phase, "goals": len(goals),
                                 "skipped": {ID_NAMES.get(m, str(m)): r for m, r in skip_reasons.items()},
                                 "joints": cycle_details}
                    teleop_log.append(log_entry)
                    if len(teleop_log) > TELEOP_LOG_MAX:
                        teleop_log.pop(0)
                    # Print summary every 20 cycles (~1s)
                    if teleop_cycle_count % 20 == 1:
                        parts = []
                        for jname in ("shoulder_pan", "shoulder_lift", "elbow_flex",
                                      "wrist_flex", "wrist_roll", "gripper"):
                            d = cycle_details.get(jname)
                            if d:
                                parts.append(f"{jname[:4]}:e={d['error']:>4} s={d['speed']:>3}")
                        skip_str = ""
                        if skip_reasons:
                            skip_str = " SKIP:" + ",".join(f"{ID_NAMES.get(m,'?')}={r}" for m, r in skip_reasons.items())
                        print(f"[teleop:{phase}] c={teleop_cycle_count} goals={len(goals)} gsw={gsw_ok} {' | '.join(parts)}{skip_str}")

                    # Transition out of equalization once all motors are close
                    if teleop_equalizing and all_equalized and teleop_eq_targets:
                        teleop_equalizing = False
                        pending_events.append(("teleop_tracking", {}))
                        print(f"[teleop] Equalization complete after {teleop_cycle_count} cycles")

        # Push safety events outside state_lock (push_event acquires it)
        for kind, detail in pending_events:
            push_event(kind, detail)
            # Adaptive learning: when a collision is confirmed, tighten cal limits
            if kind == "teleop_collision" and "name" in detail:
                _learn_collision_limit(detail["name"], detail["frozen_at"])

        # Write shared state file at 2 Hz
        if now - last_state_write >= STATE_WRITE_INTERVAL:
            last_state_write = now
            _write_shared_state()
            _save_persistent_cal_ranges()

        # Continuous data log — every cycle, all motors, all fields
        _log_motor_data(now, pending_events)

        # Process any incoming messages
        _process_messages()

        time.sleep(PING_INTERVAL)


def _restore_labels():
    """Re-apply assignment labels to boards after a rescan. Call inside state_lock."""
    for role, port in assignment.items():
        if port in boards:
            boards[port]["label"] = role


def _auto_assign_by_pgain():
    """Auto-detect leader/follower by P gain register (29): follower=16, leader=0.
    Call OUTSIDE state_lock (does serial I/O and push_event)."""
    P_GAIN_ADDR = 29
    with state_lock:
        unlabeled = [(p, b["ph"], b["handler"], b["ids"][0])
                     for p, b in boards.items()
                     if b.get("label") is None and b["ids"]]
    for port, ph, handler, mid in unlabeled:
        p_gain = read_register(ph, handler, mid, P_GAIN_ADDR, 1)
        if p_gain is not None:
            role = "follower" if p_gain > 0 else "leader"
            with state_lock:
                if role not in assignment:
                    boards[port]["label"] = role
                    assignment[role] = port
            push_event("auto_assigned", {"port": port, "role": role, "p_gain": p_gain})


def _auto_scan():
    """Detect new/disappeared boards without blocking poll_loop for long."""
    current_paths = set(glob.glob("/dev/tty.usbmodem*"))
    with state_lock:
        known_paths  = set(boards.keys())
        known_empty  = set(empty_ports)

    added   = current_paths - known_paths - known_empty
    removed = known_paths - current_paths
    # Also remove empty_ports entries that have physically disappeared
    empty_gone = known_empty - current_paths

    for path in empty_gone:
        with state_lock:
            empty_ports.discard(path)

    for path in removed:
        role = None
        with state_lock:
            role = boards[path].get("label")
            try:
                boards[path]["ph"].closePort()
            except Exception:
                pass
            del boards[path]
            if role and assignment.get(role) == path:
                del assignment[role]
        push_event("disconnected", {"port": path, "role": role})

    for path in added:
        ph = open_port(path)
        if ph is None:
            continue
        handler = PacketHandler(0)
        ids = ping_ids(ph, handler)
        if ids:
            with state_lock:
                boards[path] = {
                    "ph": ph, "handler": handler, "ids": ids,
                    "positions":       {mid: 0     for mid in ids},
                    "temps":           {mid: None  for mid in ids},
                    "loads":           {mid: None  for mid in ids},
                    "stall_counts":    {mid: 0     for mid in ids},
                    "safety_disabled": {mid: False for mid in ids},
                    "errors":    0,
                    "label":     None,
                }
            with state_lock:
                _restore_labels()
            _auto_assign_by_pgain()
            push_event("connected", {"port": path, "motor_count": len(ids)})
        else:
            ph.closePort()
            with state_lock:
                empty_ports.add(path)  # don't re-probe until it physically disconnects/reconnects


def _write_shared_state():
    """Write robot_state.json for the KB worker to read."""
    try:
        with state_lock:
            conn = {}
            motors_out = {"follower": {}, "leader": {}}
            for path, board in boards.items():
                role = board["label"]
                conn_key = role if role in ("follower", "leader") else path
                conn[conn_key] = {
                    "port": path,
                    "connected": board["errors"] < 20,
                    "motor_count": len(board["ids"]),
                    "packet_errors": board["errors"],
                }
                if role in ("follower", "leader"):
                    for mid, name in ID_NAMES.items():
                        if mid in board["ids"]:
                            motors_out[role][name] = {
                                "id": mid,
                                "position": board["positions"].get(mid),
                                "temperature": board["temps"].get(mid),
                                "load": board["loads"].get(mid),
                            }
            events_copy = list(event_log[-10:])
            cal_copy = {
                "recording": True,  # always-on calibration tracking
                "follower_file": str(CAL_PATHS["follower"]),
                "leader_file":   str(CAL_PATHS["leader"]),
                "last_saved": None,
            }

        state = {
            "_schema": "robot_state/v1",
            "_updated": time.time(),
            "connection": conn,
            "motors": motors_out,
            "calibration": cal_copy,
            "teleop_active": teleop_active,
            "events": events_copy,
        }
        # Atomic write via temp file
        tmp = STATE_FILE.with_suffix(".tmp")
        tmp.write_text(json.dumps(state, indent=2))
        tmp.replace(STATE_FILE)
    except Exception:
        pass


def _process_messages():
    """Check to_robot/ and to_user/ mailboxes."""
    # Commands for the robot tool
    try:
        for msg_file in sorted(MSG_IN.glob("*.json")):
            try:
                msg = json.loads(msg_file.read_text())
                _handle_message(msg)
                msg_file.unlink()
            except Exception:
                pass
    except Exception:
        pass

    # Notifications to show the user in the UI
    try:
        for msg_file in sorted(MSG_TO_USER.glob("*.json")):
            try:
                notif = json.loads(msg_file.read_text())
                with state_lock:
                    # Avoid duplicates by id
                    existing_ids = {n.get("id") for n in notifications}
                    if notif.get("id") not in existing_ids:
                        notif.setdefault("dismissed", False)
                        notif.setdefault("ts", time.time())
                        notifications.append(notif)
                        if len(notifications) > 20:
                            notifications.pop(0)
                msg_file.unlink()
            except Exception:
                pass
    except Exception:
        pass


def _handle_message(msg):
    msg_type = msg.get("type")
    if msg_type == "scan_request":
        global boards
        with state_lock:
            for board in boards.values():
                try:
                    board["ph"].closePort()
                except Exception:
                    pass
            boards = scan_boards()
            _restore_labels()
        _auto_assign_by_pgain()
        push_event("scan_complete", {"board_count": len(boards)})

    elif msg_type == "calibrate_start":
        global cal_ranges
        with state_lock:
            cal_ranges = {}
        push_event("calibration_started", {})

    elif msg_type == "calibrate_stop":
        pass  # calibration tracking is always on


# ------------------------------------------------------------------ #
# Routes — port detection                                             #
# ------------------------------------------------------------------ #

STATIC_DIR = Path(__file__).parent / "static"

@app.route("/")
def index():
    return send_from_directory(str(STATIC_DIR), "index.html")


@app.route("/stream")
def stream():
    def generate():
        while True:
            with state_lock:
                snapshot = {}
                for path, board in boards.items():
                    snapshot[path] = {
                        "positions":       {str(k): v for k, v in board["positions"].items()},
                        "temps":           {str(k): v for k, v in board["temps"].items()},
                        "loads":           {str(k): v for k, v in board["loads"].items()},
                        "stall_counts":    {str(k): v for k, v in board["stall_counts"].items()},
                        "safety_disabled": {str(k): v for k, v in board["safety_disabled"].items()},
                        "errors":    board["errors"],
                        "label":     board["label"],
                        "ids":       board["ids"],
                    }
                cal_snap = {
                    "active": True,  # always-on calibration tracking
                    "ranges": {
                        role: {str(mid): r for mid, r in motors.items()}
                        for role, motors in cal_ranges.items()
                    },
                }
                events = list(event_log[-5:])
                notifs = [n for n in notifications if not n.get("dismissed")]
                empty  = list(empty_ports)
            yield f"data: {json.dumps({'boards': snapshot, 'cal': cal_snap, 'events': events, 'notifications': notifs, 'empty_ports': empty, 'teleop': teleop_active, 'teleop_phase': 'equalizing' if teleop_equalizing else 'tracking', 'teleop_slow': teleop_slow_mode, 'teleop_halted': list(teleop_halted), 'teleop_collision': {ID_NAMES.get(k, str(k)): v for k, v in teleop_collision.items()}})}\n\n"
            time.sleep(0.1)

    return Response(generate(), mimetype="text/event-stream",
                    headers={"Cache-Control": "no-cache", "X-Accel-Buffering": "no"})


@app.route("/api/assign", methods=["POST"])
def api_assign():
    data = request.json
    port = data.get("port")
    role = data.get("role")
    if not port or role not in ("leader", "follower", "clear"):
        return jsonify({"error": "invalid"}), 400
    with state_lock:
        if port not in boards:
            return jsonify({"error": "unknown port"}), 404
        if role == "clear":
            old = boards[port]["label"]
            boards[port]["label"] = None
            if old and assignment.get(old) == port:
                del assignment[old]
        else:
            for b in boards.values():
                if b["label"] == role:
                    b["label"] = None
            boards[port]["label"] = role
            assignment[role] = port
    return jsonify({"ok": True})


@app.route("/api/save-ports", methods=["POST"])
def api_save_ports():
    with state_lock:
        follower = assignment.get("follower")
        leader   = assignment.get("leader")
    if not follower or not leader:
        return jsonify({"error": "Assign both leader and follower first."}), 400
    if follower == leader:
        return jsonify({"error": "Leader and follower cannot be the same port."}), 400
    content = f"""# Robot Configuration
# Source this file before running robot commands:
#   source ~/so101/robot-workspace/robot.env

ROBOT_ENV=~/lerobot-env-312
FOLLOWER_PORT={follower}
LEADER_PORT={leader}
ROBOT_ID=my_follower
TELEOP_ID=my_leader
"""
    ENV_FILE.write_text(content)
    push_event("port_assigned", {"follower": follower, "leader": leader})
    return jsonify({"ok": True, "follower": follower, "leader": leader})


@app.route("/api/rescan", methods=["POST"])
def api_rescan():
    global boards
    with state_lock:
        for board in boards.values():
            try:
                board["ph"].closePort()
            except Exception:
                pass
        boards = scan_boards()
        _restore_labels()
    _auto_assign_by_pgain()
    push_event("scan_complete", {"board_count": len(boards)})
    return jsonify({"ok": True, "count": len(boards)})


# ------------------------------------------------------------------ #
# Routes — notifications (Claude <-> user via UI)                    #
# ------------------------------------------------------------------ #

@app.route("/api/notify", methods=["POST"])
def api_notify():
    """
    Push a notification to the UI. Called by Claude or any worker.

    Body:
    {
      "id": "unique-id",          # optional, auto-generated if absent
      "type": "info|warning|action_request",
      "title": "Short title",
      "body": "Longer description",
      "actions": [                # optional buttons
        {"label": "Start Calibration", "trigger": "cal_start"},
        {"label": "Dismiss",           "trigger": "dismiss"}
      ]
    }
    """
    data = request.json or {}
    notif = {
        "id":      data.get("id") or f"{int(time.time()*1000)}",
        "type":    data.get("type", "info"),
        "title":   data.get("title", ""),
        "body":    data.get("body", ""),
        "actions": data.get("actions", []),
        "ts":      time.time(),
        "dismissed": False,
    }
    with state_lock:
        existing_ids = {n.get("id") for n in notifications}
        if notif["id"] not in existing_ids:
            notifications.append(notif)
            if len(notifications) > 20:
                notifications.pop(0)
    return jsonify({"ok": True, "id": notif["id"]})


@app.route("/api/notify/dismiss", methods=["POST"])
def api_notify_dismiss():
    notif_id = (request.json or {}).get("id")
    with state_lock:
        for n in notifications:
            if n.get("id") == notif_id:
                n["dismissed"] = True
    return jsonify({"ok": True})


@app.route("/api/notify/action", methods=["POST"])
def api_notify_action():
    """User clicked an action button on a notification."""
    data      = request.json or {}
    notif_id  = data.get("id")
    trigger   = data.get("trigger")

    # Dismiss the notification
    with state_lock:
        for n in notifications:
            if n.get("id") == notif_id:
                n["dismissed"] = True

    # Handle built-in triggers directly
    if trigger == "cal_start":
        global cal_ranges
        with state_lock:
            cal_ranges = {}
        push_event("calibration_started", {"source": "notification"})
    elif trigger == "cal_stop":
        pass  # calibration tracking is always on
    elif trigger == "rescan":
        global boards
        with state_lock:
            for board in boards.values():
                try: board["ph"].closePort()
                except Exception: pass
            boards = scan_boards()
            _restore_labels()
        _auto_assign_by_pgain()
        push_event("scan_complete", {"board_count": len(boards)})

    # Write response to from_user/ for Claude to read
    ts = time.time()
    resp = {"id": f"{int(ts*1000)}_response", "notif_id": notif_id,
            "trigger": trigger, "ts": ts}
    try:
        (MSG_FROM_USER / f"{int(ts*1000)}_response.json").write_text(
            json.dumps(resp, indent=2))
    except Exception:
        pass

    return jsonify({"ok": True, "trigger": trigger})


@app.route("/api/notify/responses")
def api_notify_responses():
    """Poll pending user responses (for Claude to read)."""
    responses = []
    try:
        for f in sorted(MSG_FROM_USER.glob("*.json")):
            try:
                responses.append(json.loads(f.read_text()))
            except Exception:
                pass
    except Exception:
        pass
    return jsonify(responses)


@app.route("/api/notify/responses/clear", methods=["POST"])
def api_notify_responses_clear():
    """Delete all processed responses."""
    try:
        for f in MSG_FROM_USER.glob("*.json"):
            f.unlink()
    except Exception:
        pass
    return jsonify({"ok": True})


# ------------------------------------------------------------------ #
# Routes — calibration                                                #
# ------------------------------------------------------------------ #

@app.route("/api/cal/start", methods=["POST"])
def api_cal_start():
    global cal_ranges
    with state_lock:
        cal_ranges = {}
    push_event("calibration_started", {})
    return jsonify({"ok": True, "note": "Ranges reset. Calibration tracking is always on."})


@app.route("/api/cal/reset", methods=["POST"])
def api_cal_reset():
    global cal_ranges
    with state_lock:
        cal_ranges = {}
    return jsonify({"ok": True})


@app.route("/api/cal/stop", methods=["POST"])
def api_cal_stop():
    # No-op — calibration tracking is always on. Use /api/cal/reset to clear ranges.
    return jsonify({"ok": True, "note": "Calibration tracking is always on. Use /api/cal/reset to clear."})


@app.route("/api/cal/save", methods=["POST"])
def api_cal_save():
    with state_lock:
        ranges_copy = {r: {mid: dict(v) for mid, v in motors.items()}
                       for r, motors in cal_ranges.items()}

    if not ranges_copy:
        return jsonify({"error": "No calibration data recorded yet."}), 400

    saved = []
    skipped = []
    warnings = []

    ts = time.strftime("%Y%m%d_%H%M%S")
    for role, motors in ranges_copy.items():
        cal_path = CAL_PATHS.get(role)
        if not cal_path or not cal_path.exists():
            skipped.append(f"{role}: no calibration file at {cal_path}")
            continue

        # Back up current calibration before overwriting
        with open(cal_path) as f:
            cal = json.load(f)
        backup_path = CAL_HISTORY_DIR / f"{role}_{ts}.json"
        with open(backup_path, "w") as f:
            json.dump({"_backup_of": str(cal_path), "_timestamp": ts, **cal}, f, indent=4)

        for mid, name in ID_NAMES.items():
            if name not in cal or mid not in motors:
                continue
            r = motors[mid]
            new_spread = r["max"] - r["min"]
            old_min = cal[name].get("range_min", 0)
            old_max = cal[name].get("range_max", 4095)
            old_spread = old_max - old_min

            if new_spread < 50:
                warnings.append(f"{role}/{name}: only {new_spread} ticks — skipping (would narrow from {old_spread})")
                continue  # NEVER overwrite good calibration with bad data

            # Only widen ranges, never narrow — take the union of old and new
            cal[name]["range_min"] = min(r["min"], old_min)
            cal[name]["range_max"] = max(r["max"], old_max)
        with open(cal_path, "w") as f:
            json.dump(cal, f, indent=4)

        # Also save the new calibration to history
        new_path = CAL_HISTORY_DIR / f"{role}_{ts}_new.json"
        with open(new_path, "w") as f:
            json.dump({"_saved_at": ts, **cal}, f, indent=4)

        saved.append(str(cal_path))

    push_event("calibration_saved", {"saved": saved, "warnings": warnings})
    return jsonify({"ok": True, "saved": saved, "skipped": skipped, "warnings": warnings})


@app.route("/api/cal/history", methods=["GET"])
def api_cal_history():
    """List all calibration backups."""
    files = sorted(CAL_HISTORY_DIR.glob("*.json"), reverse=True)
    history = []
    for f in files[:50]:  # cap at 50
        try:
            data = json.loads(f.read_text())
            history.append({
                "filename": f.name,
                "timestamp": data.get("_timestamp") or data.get("_saved_at", ""),
                "role": f.name.split("_")[0],
                "is_backup": "_new" not in f.name,
            })
        except Exception:
            pass
    return jsonify(history)


@app.route("/api/cal/restore", methods=["POST"])
def api_cal_restore():
    """Restore a calibration from history.

    Body: {"filename": "follower_20260514_134500.json"}
    """
    data = request.get_json(force=True) or {}
    filename = data.get("filename")
    if not filename:
        return jsonify({"error": "filename required"}), 400

    src = CAL_HISTORY_DIR / filename
    if not src.exists():
        return jsonify({"error": f"file not found: {filename}"}), 404

    hist = json.loads(src.read_text())
    role = filename.split("_")[0]
    cal_path = CAL_PATHS.get(role)
    if not cal_path:
        return jsonify({"error": f"unknown role in filename: {role}"}), 400

    # Back up current before restoring
    if cal_path.exists():
        ts = time.strftime("%Y%m%d_%H%M%S")
        backup = CAL_HISTORY_DIR / f"{role}_{ts}_prerestore.json"
        current = json.loads(cal_path.read_text())
        with open(backup, "w") as f:
            json.dump({"_backup_of": str(cal_path), "_timestamp": ts, "_reason": "pre-restore", **current}, f, indent=4)

    # Strip metadata keys and write
    cal_data = {k: v for k, v in hist.items() if not k.startswith("_")}
    with open(cal_path, "w") as f:
        json.dump(cal_data, f, indent=4)

    push_event("calibration_restored", {"role": role, "from": filename})
    return jsonify({"ok": True, "restored": str(cal_path), "from": filename})


@app.route("/api/cal/check", methods=["GET"])
def api_cal_check():
    """Check if calibration data looks valid (ranges wide enough for safe operation)."""
    results = {}
    for role, cal_path in CAL_PATHS.items():
        if not cal_path.exists():
            results[role] = {"status": "missing", "path": str(cal_path)}
            continue
        try:
            cal = json.loads(cal_path.read_text())
            issues = []
            for name, info in cal.items():
                rmin = info.get("range_min", 0)
                rmax = info.get("range_max", 0)
                spread = rmax - rmin
                if spread < 50:
                    issues.append(f"{name}: spread={spread} (need >50, got min={rmin} max={rmax})")
                elif spread < 500:
                    issues.append(f"{name}: spread={spread} (narrow, may need re-sweep)")
            results[role] = {
                "status": "invalid" if any("need >50" in i for i in issues) else "ok" if not issues else "warning",
                "issues": issues,
                "joints": {name: {"min": info["range_min"], "max": info["range_max"], "spread": info["range_max"] - info["range_min"]}
                           for name, info in cal.items() if "range_min" in info},
            }
        except Exception as e:
            results[role] = {"status": "error", "error": str(e)}
    return jsonify(results)


# ------------------------------------------------------------------ #
# Routes — self-test                                                   #
# ------------------------------------------------------------------ #

@app.route("/api/selftest", methods=["POST"])
def api_selftest():
    """Physical self-test: probe each joint with small movements to verify
    motor responsiveness and calibration validity.

    For each joint on the requested arm:
      1. Read current position
      2. Check position is within calibrated range
      3. Nudge +60 counts, wait, read position (did it move?)
      4. Nudge back -60 counts, wait, read position (did it return?)
      5. Report pass/fail per joint

    Body: {"label": "follower"}  (default: follower)
    """
    data = request.get_json(force=True) or {}
    label = data.get("label", "follower")

    # Check teleop
    if teleop_active:
        return jsonify({"error": "Stop teleop before running self-test"}), 409

    # Find board
    with state_lock:
        target_port = assignment.get(label)
        board = boards.get(target_port) if target_port else None
    if board is None:
        return jsonify({"error": f"No board for '{label}'. Assign arm first."}), 404

    # Pre-flight temperature check — don't actuate hot motors
    SELFTEST_TEMP_LIMIT = TEMP_CUTOFF_C - 5
    with state_lock:
        hot = {ID_NAMES.get(mid, str(mid)): t
               for mid in board["ids"]
               if (t := board["temps"].get(mid)) is not None and t >= SELFTEST_TEMP_LIMIT}
    if hot:
        return jsonify({"error": f"Motors too hot to self-test: {hot}. Wait for cooling."}), 409

    # Load calibration
    cal_path = CAL_PATHS.get(label)
    cal = {}
    if cal_path and cal_path.exists():
        try:
            cal = json.loads(cal_path.read_text())
        except Exception:
            pass

    PROBE_DELTA = 60   # counts to nudge
    SETTLE_TIME = 0.8  # seconds to wait for motor to settle
    MOVE_THRESH = 15   # must move at least this many counts to pass

    results = {}

    for mid, name in ID_NAMES.items():
        if mid not in board["ids"]:
            results[name] = {"status": "skipped", "reason": "motor not found on bus"}
            continue

        joint_cal = cal.get(name, {})
        cal_min = joint_cal.get("range_min")
        cal_max = joint_cal.get("range_max")

        # 1. Read current position
        with state_lock:
            pos_start = board["positions"].get(mid)
        if pos_start is None:
            results[name] = {"status": "fail", "reason": "cannot read position"}
            continue

        # 2. Check calibration range
        cal_status = "ok"
        if cal_min is not None and cal_max is not None:
            spread = cal_max - cal_min
            if spread < 50:
                cal_status = "wiped"
            elif pos_start < cal_min or pos_start > cal_max:
                cal_status = "out_of_range"
        else:
            cal_status = "missing"

        # 3. Determine nudge direction (stay inside range)
        if cal_max is not None and pos_start + PROBE_DELTA > cal_max:
            nudge = -PROBE_DELTA
        else:
            nudge = PROBE_DELTA

        # 4. Nudge forward
        target_fwd = max(0, min(4095, pos_start + nudge))
        move_grace[mid] = time.time() + SETTLE_TIME + 0.5  # suppress stall detection
        with state_lock:
            ph, handler = board["ph"], board["handler"]
            board["safety_disabled"][mid] = False
            board["stall_counts"][mid] = 0
            write_register(ph, handler, mid, TORQUE_ENABLE_ADDR, 1, length=1)
            write_register(ph, handler, mid, GOAL_POSITION_ADDR, target_fwd, length=2)

        time.sleep(SETTLE_TIME)

        with state_lock:
            pos_after_fwd = board["positions"].get(mid, pos_start)

        fwd_moved = abs(pos_after_fwd - pos_start)

        # 5. Nudge back to original
        move_grace[mid] = time.time() + SETTLE_TIME + 0.5
        with state_lock:
            write_register(ph, handler, mid, GOAL_POSITION_ADDR, pos_start, length=2)

        time.sleep(SETTLE_TIME)

        with state_lock:
            pos_after_back = board["positions"].get(mid, pos_start)
            write_register(ph, handler, mid, TORQUE_ENABLE_ADDR, 0, length=1)

        back_returned = abs(pos_after_back - pos_start)

        # 6. Determine result
        motor_ok = fwd_moved >= MOVE_THRESH
        returned_ok = back_returned < MOVE_THRESH

        if motor_ok and returned_ok:
            status = "pass"
        elif not motor_ok:
            status = "fail"
            reason = f"motor did not move (delta={fwd_moved}, need >={MOVE_THRESH})"
        else:
            status = "warning"
            reason = f"moved but didn't return (off by {back_returned})"

        result = {
            "status": status,
            "pos_start": pos_start,
            "pos_after_nudge": pos_after_fwd,
            "pos_after_return": pos_after_back,
            "nudge_delta": fwd_moved,
            "return_error": back_returned,
            "calibration": cal_status,
        }
        if status != "pass":
            result["reason"] = reason
        results[name] = result

    # Summary
    passed = sum(1 for r in results.values() if r["status"] == "pass")
    total = len(results)
    push_event("selftest_complete", {"label": label, "passed": passed, "total": total})

    return jsonify({
        "label": label,
        "passed": passed,
        "total": total,
        "all_pass": passed == total,
        "joints": results,
    })


# ------------------------------------------------------------------ #
# Routes — auto-calibrate (cybernetic limit probing)                   #
# ------------------------------------------------------------------ #

NAME_TO_ID = {v: k for k, v in ID_NAMES.items()}  # reverse of ID_NAMES

@app.route("/api/autocalibrate", methods=["POST"])
def api_autocalibrate():
    """Cybernetic self-calibration: probe each joint to find its real physical
    limits by slowly creeping until load resistance is detected.

    For each joint:
      1. From current position, creep toward 0 at PROBE_STEP counts/cycle
      2. When load > threshold for 3 consecutive reads, OR motor stalls, record as range_min
      3. Return to start, then creep toward 4095
      4. Same detection for range_max
      5. Apply safety margin (5% inward on each side)

    Finds actual workspace boundaries — desk, cables, mechanical stops —
    not just encoder range. The feedback loop is the calibration.

    Body: {"label": "follower", "joints": ["elbow_flex"], "save": true}
    """
    data = request.get_json(force=True) or {}
    label = data.get("label", "follower")
    requested_joints = data.get("joints", list(NAME_TO_ID.keys()))
    margin = data.get("margin", 0.05)
    save_results = data.get("save", False)

    if teleop_active:
        return jsonify({"error": "Stop teleop before auto-calibrating"}), 409

    with state_lock:
        target_port = assignment.get(label)
        board = boards.get(target_port) if target_port else None
    if board is None:
        return jsonify({"error": f"No board for '{label}'. Assign arm first."}), 404

    PROBE_STEP = 10          # counts per cycle — very slow creep
    PROBE_LOAD_THRESH = 200  # load indicating resistance
    PROBE_CONFIRM = 3        # consecutive high-load reads to confirm
    PROBE_SPEED = 60         # servo speed during probing
    SETTLE = 0.15            # seconds between steps
    MAX_STEPS = 500          # don't probe more than 5000 counts per direction

    results = {}

    for joint_name in requested_joints:
        mid = NAME_TO_ID.get(joint_name)
        if mid is None or mid not in board["ids"]:
            results[joint_name] = {"status": "skipped", "reason": "not found"}
            continue

        with state_lock:
            start_pos = board["positions"].get(mid)
        if start_pos is None:
            results[joint_name] = {"status": "skipped", "reason": "no position read"}
            continue

        # Suppress stall detection for the full probe duration
        move_grace[mid] = time.time() + (MAX_STEPS * SETTLE * 2) + 30
        with state_lock:
            ph, handler = board["ph"], board["handler"]
            board["safety_disabled"][mid] = False
            board["stall_counts"][mid] = 0
            write_register(ph, handler, mid, SPEED_ADDR, PROBE_SPEED, length=2)
            write_register(ph, handler, mid, TORQUE_ENABLE_ADDR, 1, length=1)

        def _probe_direction(start, step):
            """Probe in one direction. Returns the position where limit was found."""
            high_count = 0
            current = start
            for i in range(MAX_STEPS):
                target = max(0, min(4095, current + step))
                with state_lock:
                    write_register(ph, handler, mid, GOAL_POSITION_ADDR, target, length=2)
                time.sleep(SETTLE)
                with state_lock:
                    actual = board["positions"].get(mid, target)
                    load = board["loads"].get(mid, 0) or 0

                moved = abs(actual - current) >= 2
                if load > PROBE_LOAD_THRESH:
                    high_count += 1
                elif not moved and i > 5:
                    high_count += 1  # mechanical stop (no load sensor response)
                else:
                    high_count = 0

                if high_count >= PROBE_CONFIRM:
                    return actual
                current = actual
            return current  # hit MAX_STEPS

        # Probe toward min (negative direction)
        found_min = _probe_direction(start_pos, -PROBE_STEP)

        # Return to start
        with state_lock:
            write_register(ph, handler, mid, GOAL_POSITION_ADDR, start_pos, length=2)
        time.sleep(1.0)

        # Probe toward max (positive direction)
        with state_lock:
            current_after_return = board["positions"].get(mid, start_pos)
        found_max = _probe_direction(current_after_return, PROBE_STEP)

        # Return to start, disable torque
        with state_lock:
            write_register(ph, handler, mid, GOAL_POSITION_ADDR, start_pos, length=2)
        time.sleep(0.8)
        with state_lock:
            write_register(ph, handler, mid, TORQUE_ENABLE_ADDR, 0, length=1)

        # Apply margin
        spread = found_max - found_min
        margin_counts = int(spread * margin)
        safe_min = found_min + margin_counts
        safe_max = found_max - margin_counts

        results[joint_name] = {
            "status": "ok",
            "raw_min": found_min, "raw_max": found_max,
            "spread": spread,
            "safe_min": safe_min, "safe_max": safe_max,
            "margin": margin,
            "start_pos": start_pos,
        }

    # Optionally write to calibration file
    if save_results:
        cal_path = CAL_PATHS.get(label)
        if cal_path and cal_path.exists():
            try:
                cal = json.loads(cal_path.read_text())
                ts = time.strftime("%Y%m%d_%H%M%S")
                backup = CAL_HISTORY_DIR / f"{label}_{ts}_pre_autocal.json"
                with open(backup, "w") as f:
                    json.dump({"_backup_of": str(cal_path), "_timestamp": ts, **cal}, f, indent=4)
                for jname, r in results.items():
                    if r["status"] == "ok" and jname in cal:
                        cal[jname]["range_min"] = r["safe_min"]
                        cal[jname]["range_max"] = r["safe_max"]
                with open(cal_path, "w") as f:
                    json.dump(cal, f, indent=4)
                push_event("autocalibration_saved", {"label": label, "joints": list(results.keys())})
            except Exception as e:
                return jsonify({"error": f"Save failed: {e}", "results": results}), 500

    push_event("autocalibration_complete", {"label": label, "results": results})
    return jsonify({"label": label, "joints": results, "saved": save_results})


# ------------------------------------------------------------------ #
# Routes — workspace mapping (pose-dependent obstacle detection)       #
# ------------------------------------------------------------------ #

WORKSPACE_FILE = SHARED_DIR / "workspace.json"

@app.route("/api/workspace/probe", methods=["POST"])
def api_workspace_probe():
    """Level 2 workspace mapping: probe one joint's limits at multiple poses
    of a coupled joint to build a pose-dependent obstacle map.

    Example: probe elbow_flex limits at 8 different shoulder_lift positions
    to find where the desk is at each shoulder height.

    Body: {
        "label": "follower",
        "probe_joint": "elbow_flex",         -- joint to probe for limits
        "sweep_joint": "shoulder_lift",       -- joint to vary
        "sweep_min": 800, "sweep_max": 2400,  -- range to sweep (or auto from cal)
        "sweep_steps": 8,                     -- number of positions to sample
        "save": true                          -- persist to workspace.json
    }
    """
    data = request.get_json(force=True) or {}
    label = data.get("label", "follower")
    probe_name = data.get("probe_joint", "elbow_flex")
    sweep_name = data.get("sweep_joint", "shoulder_lift")
    sweep_steps = data.get("sweep_steps", 8)
    save = data.get("save", False)

    if teleop_active:
        return jsonify({"error": "Stop teleop first"}), 409

    probe_mid = NAME_TO_ID.get(probe_name)
    sweep_mid = NAME_TO_ID.get(sweep_name)
    if probe_mid is None or sweep_mid is None:
        return jsonify({"error": f"Unknown joint. Valid: {list(NAME_TO_ID.keys())}"}), 400

    with state_lock:
        target_port = assignment.get(label)
        board = boards.get(target_port) if target_port else None
    if board is None:
        return jsonify({"error": f"No board for '{label}'"}), 404

    # Get sweep range from calibration or request
    cal_path = CAL_PATHS.get(label)
    cal = {}
    if cal_path and cal_path.exists():
        try:
            cal = json.loads(cal_path.read_text())
        except Exception:
            pass

    sweep_cal = cal.get(sweep_name, {})
    sweep_min = data.get("sweep_min", sweep_cal.get("range_min", 500))
    sweep_max = data.get("sweep_max", sweep_cal.get("range_max", 3500))

    # Generate sweep positions (evenly spaced)
    if sweep_steps < 2:
        sweep_steps = 2
    sweep_positions = [
        int(sweep_min + i * (sweep_max - sweep_min) / (sweep_steps - 1))
        for i in range(sweep_steps)
    ]

    PROBE_STEP = 10
    PROBE_LOAD_THRESH = 200
    PROBE_CONFIRM = 3
    PROBE_SPEED = 60
    SETTLE = 0.15
    MAX_STEPS = 400
    MOVE_SPEED = 200

    # Save starting positions
    with state_lock:
        start_probe = board["positions"].get(probe_mid)
        start_sweep = board["positions"].get(sweep_mid)

    samples = []  # [{sweep_pos, probe_min, probe_max, probe_min_load, probe_max_load}]

    for sweep_target in sweep_positions:
        # Move sweep joint to target position
        move_grace[sweep_mid] = time.time() + 15
        move_grace[probe_mid] = time.time() + (MAX_STEPS * SETTLE * 2) + 30
        with state_lock:
            ph, handler = board["ph"], board["handler"]
            board["safety_disabled"][sweep_mid] = False
            board["stall_counts"][sweep_mid] = 0
            write_register(ph, handler, sweep_mid, SPEED_ADDR, MOVE_SPEED, length=2)
            write_register(ph, handler, sweep_mid, TORQUE_ENABLE_ADDR, 1, length=1)
            write_register(ph, handler, sweep_mid, GOAL_POSITION_ADDR, sweep_target, length=2)
        time.sleep(1.5)  # let sweep joint settle

        # Move probe joint to mid-range so it has room to probe both directions.
        # Under gravity, joints swing to extremes when the arm pose changes.
        probe_cal = cal.get(probe_name, {})
        probe_mid_pos = (probe_cal.get("range_min", 0) + probe_cal.get("range_max", 4095)) // 2
        with state_lock:
            board["safety_disabled"][probe_mid] = False
            board["stall_counts"][probe_mid] = 0
            write_register(ph, handler, probe_mid, SPEED_ADDR, MOVE_SPEED, length=2)
            write_register(ph, handler, probe_mid, TORQUE_ENABLE_ADDR, 1, length=1)
            write_register(ph, handler, probe_mid, GOAL_POSITION_ADDR, probe_mid_pos, length=2)
        time.sleep(2.0)  # let probe joint reach mid-range
        with state_lock:
            write_register(ph, handler, probe_mid, SPEED_ADDR, PROBE_SPEED, length=2)
            probe_start = board["positions"].get(probe_mid, probe_mid_pos)

        # Probe toward min
        def _probe(start_pos, step):
            high_count = 0
            current = start_pos
            for i in range(MAX_STEPS):
                t = max(0, min(4095, current + step))
                with state_lock:
                    write_register(ph, handler, probe_mid, GOAL_POSITION_ADDR, t, length=2)
                time.sleep(SETTLE)
                with state_lock:
                    actual = board["positions"].get(probe_mid, t)
                    load = board["loads"].get(probe_mid, 0) or 0
                moved = abs(actual - current) >= 2
                if load > PROBE_LOAD_THRESH:
                    high_count += 1
                elif not moved and i > 5:
                    high_count += 1
                else:
                    high_count = 0
                if high_count >= PROBE_CONFIRM:
                    return actual, load
                current = actual
            return current, 0

        found_min, min_load = _probe(probe_start, -PROBE_STEP)

        # Return probe to start
        with state_lock:
            write_register(ph, handler, probe_mid, GOAL_POSITION_ADDR, probe_start, length=2)
        time.sleep(1.0)

        # Probe toward max
        with state_lock:
            cur = board["positions"].get(probe_mid, probe_start)
        found_max, max_load = _probe(cur, PROBE_STEP)

        # Return probe
        with state_lock:
            write_register(ph, handler, probe_mid, GOAL_POSITION_ADDR, probe_start, length=2)
        time.sleep(0.5)

        with state_lock:
            actual_sweep = board["positions"].get(sweep_mid, sweep_target)

        samples.append({
            "sweep_pos": actual_sweep,
            "probe_min": found_min,
            "probe_max": found_max,
            "probe_min_load": min_load,
            "probe_max_load": max_load,
        })

    # Return both joints to start, disable torque
    with state_lock:
        write_register(ph, handler, probe_mid, GOAL_POSITION_ADDR, start_probe or 2048, length=2)
        write_register(ph, handler, sweep_mid, GOAL_POSITION_ADDR, start_sweep or 2048, length=2)
    time.sleep(1.0)
    with state_lock:
        write_register(ph, handler, probe_mid, TORQUE_ENABLE_ADDR, 0, length=1)
        write_register(ph, handler, sweep_mid, TORQUE_ENABLE_ADDR, 0, length=1)

    result = {
        "label": label,
        "probe_joint": probe_name,
        "sweep_joint": sweep_name,
        "samples": samples,
        "sweep_range": [sweep_min, sweep_max],
        "sweep_steps": sweep_steps,
    }

    # Save to workspace.json
    if save:
        workspace = {}
        if WORKSPACE_FILE.exists():
            try:
                workspace = json.loads(WORKSPACE_FILE.read_text())
            except Exception:
                pass
        key = f"{label}/{probe_name}_vs_{sweep_name}"
        workspace[key] = {
            "ts": time.strftime("%Y-%m-%dT%H:%M:%S"),
            "samples": samples,
        }
        with open(WORKSPACE_FILE, "w") as f:
            json.dump(workspace, f, indent=2)
        result["saved_to"] = str(WORKSPACE_FILE)

    push_event("workspace_probe_complete", {
        "label": label, "probe": probe_name, "sweep": sweep_name,
        "samples": len(samples),
    })

    return jsonify(result)


@app.route("/api/workspace", methods=["GET"])
def api_workspace():
    """Return saved workspace map data."""
    if not WORKSPACE_FILE.exists():
        return jsonify({"maps": {}, "note": "No workspace data yet. Run /api/workspace/probe."})
    try:
        return jsonify(json.loads(WORKSPACE_FILE.read_text()))
    except Exception as e:
        return jsonify({"error": str(e)}), 500


# ------------------------------------------------------------------ #
# Routes — shared state                                               #
# ------------------------------------------------------------------ #

@app.route("/api/state")
def api_state():
    """Current robot state — same data written to robot_state.json."""
    try:
        return jsonify(json.loads(STATE_FILE.read_text()))
    except Exception:
        return jsonify({"error": "state file not yet written"}), 503


@app.route("/api/positions")
def api_positions():
    """Fast endpoint: current positions for both arms, plus teleop state.
    Used by the recorder at 30Hz — reads from memory, not disk."""
    with state_lock:
        result = {"teleop": teleop_active, "ts": time.time()}
        if teleop_collision:
            result["collision"] = {ID_NAMES.get(k, str(k)): v for k, v in teleop_collision.items()}
        for port, board in boards.items():
            label = board.get("label")
            if label in ("follower", "leader"):
                result[label] = {}
                for mid, name in ID_NAMES.items():
                    if mid in board["positions"]:
                        result[label][name] = board["positions"][mid]
    return jsonify(result)


@app.route("/api/events")
def api_events():
    with state_lock:
        return jsonify(list(event_log))


@app.route("/api/move", methods=["POST"])
def api_move():
    """Move motors to specified positions.

    Body: {"label": "follower", "positions": {motor_id: value, ...}}
    Or:   {"label": "follower", "preset": "middle"}
    """
    data = request.get_json(force=True) or {}
    label = data.get("label", "follower")

    # Load calibration to compute midpoints if preset=middle
    preset = data.get("preset")
    if preset == "middle":
        cal_file = CAL_PATHS.get(label)
        if cal_file is None or not cal_file.exists():
            return jsonify({"error": "calibration file not found"}), 404
        cal = json.loads(cal_file.read_text())
        positions = {}
        for name, info in cal.items():
            mid_val = (info["range_min"] + info["range_max"]) // 2
            positions[info["id"]] = mid_val
    else:
        positions = {int(k): int(v) for k, v in data.get("positions", {}).items()}

    if not positions:
        return jsonify({"error": "no positions specified"}), 400

    # ── Clamp to calibration limits ──────────────────────────────────
    limits = _load_cal_limits(label)
    clamped_positions = {}
    for mid, goal in positions.items():
        if mid in limits:
            lo, hi = limits[mid]
            clamped_positions[mid] = max(lo, min(hi, goal))
        else:
            clamped_positions[mid] = goal
    positions = clamped_positions

    # ── Clamp to workspace limits (P9: environment-aware) ────────────
    # Workspace limits from probing override calibration where more restrictive
    try:
        ws = _safety.get("workspace", {}).get(label, {})
        for jname, ws_limits in ws.items():
            mid = NAME_TO_ID.get(jname)
            if mid is not None and mid in positions:
                if "hard_min" in ws_limits:
                    positions[mid] = max(ws_limits["hard_min"], positions[mid])
                if "hard_max" in ws_limits:
                    positions[mid] = min(ws_limits["hard_max"], positions[mid])
    except Exception:
        pass  # workspace limits are advisory — don't block moves on parse errors

    with state_lock:
        # Find the board assigned to the requested label
        target_port = assignment.get(label)
        board = boards.get(target_port) if target_port else None
        # Fallback: if only one board, use it
        if board is None and len(boards) == 1:
            board = next(iter(boards.values()))

    if board is None:
        return jsonify({"error": f"no board found for label '{label}'"}), 404

    ramped = data.get("ramped", False)
    is_leader = (label == "leader")

    # Set move grace period — suppress stall detection while motor escapes high-load zone
    grace_until = time.time() + MOVE_GRACE_S
    for mid in positions:
        move_grace[mid] = grace_until

    if ramped:
        # P1: No teleport — ramp toward targets in background thread
        def _ramped_move():
            with state_lock:
                ph, handler = board["ph"], board["handler"]
                # Leader has P=0 D=0 (passive arm) — set temporary PID gains so it tracks
                if is_leader:
                    for mid in positions:
                        if mid in board["ids"]:
                            write_register(ph, handler, mid, P_GAIN_ADDR, 32, length=1)
                            write_register(ph, handler, mid, D_GAIN_ADDR, 32, length=1)
                for mid in positions:
                    if mid in board["ids"]:
                        board["safety_disabled"][mid] = False
                        board["stall_counts"][mid]    = 0
                        write_register(ph, handler, mid, TORQUE_ENABLE_ADDR, 1, length=1)

            max_delta = TELEOP_MAX_DELTA  # counts per cycle
            remaining = dict(positions)   # {mid: final_goal}
            collided = {}  # {mid: load} — joints stopped due to collision

            for _ in range(200):  # max 200 cycles = 10s at 20Hz
                if not remaining:
                    break
                done = []
                with state_lock:
                    ph, handler = board["ph"], board["handler"]
                    for mid, goal in remaining.items():
                        # P5: collision detection — if load > threshold, stop immediately
                        load = board["loads"].get(mid)
                        if load is not None and load > COLLISION_LOAD_THRESHOLD:
                            cur = board["positions"].get(mid, goal)
                            write_register(ph, handler, mid, GOAL_POSITION_ADDR, cur, length=2)
                            write_register(ph, handler, mid, TORQUE_ENABLE_ADDR, 0, length=1)
                            collided[mid] = load
                            done.append(mid)
                            push_event("collision_detected", {
                                "label": label, "motor": mid,
                                "name": ID_NAMES.get(mid, str(mid)),
                                "load": load, "position": cur,
                            })
                            continue

                        # P2: read actual position each cycle (poll loop updates at 20Hz)
                        current = board["positions"].get(mid, goal)
                        delta = goal - current
                        if abs(delta) <= 2:  # deadband
                            done.append(mid)
                            continue
                        step = max(-max_delta, min(max_delta, delta))
                        next_pos = current + step
                        write_register(ph, handler, mid, GOAL_POSITION_ADDR, next_pos, length=2)
                for mid in done:
                    del remaining[mid]
                time.sleep(PING_INTERVAL)

            # Disable torque after settling
            time.sleep(0.5)
            with state_lock:
                ph, handler = board["ph"], board["handler"]
                for mid in positions:
                    if mid in board["ids"]:
                        write_register(ph, handler, mid, TORQUE_ENABLE_ADDR, 0, length=1)
                # Restore leader PID gains to passive (P=0, D=0)
                if is_leader:
                    for mid in positions:
                        if mid in board["ids"]:
                            write_register(ph, handler, mid, P_GAIN_ADDR, 0, length=1)
                            write_register(ph, handler, mid, D_GAIN_ADDR, 0, length=1)
            result = {"label": label, "positions": positions, "ramped": True}
            if collided:
                result["collisions"] = {ID_NAMES.get(k, str(k)): v for k, v in collided.items()}
            push_event("move_complete", result)

        threading.Thread(target=_ramped_move, daemon=True).start()
        push_event("move_start", {"label": label, "positions": positions, "ramped": True})
        return jsonify({"status": "ramping", "positions": positions})

    # Legacy: instant move (no ramping)
    results = {}
    with state_lock:
        ph, handler = board["ph"], board["handler"]
        # Leader has P=0 D=0 (passive arm) — set temporary PID gains so it tracks
        if is_leader:
            for mid in positions:
                if mid in board["ids"]:
                    write_register(ph, handler, mid, P_GAIN_ADDR, 32, length=1)
                    write_register(ph, handler, mid, D_GAIN_ADDR, 32, length=1)
        for mid, goal in positions.items():
            if mid not in board["ids"]:
                results[mid] = "skipped (not found)"
                continue
            board["safety_disabled"][mid] = False
            board["stall_counts"][mid]    = 0
            write_register(ph, handler, mid, TORQUE_ENABLE_ADDR, 1, length=1)
            ok = write_register(ph, handler, mid, GOAL_POSITION_ADDR, goal, length=2)
            results[mid] = "ok" if ok else "write_error"

    # Disable torque after movement settles (run in background thread)
    def _disable_torque():
        time.sleep(2.5)
        with state_lock:
            ph, handler = board["ph"], board["handler"]
            for mid in positions:
                write_register(ph, handler, mid, TORQUE_ENABLE_ADDR, 0, length=1)
            # Restore leader PID gains to passive (P=0, D=0)
            if is_leader:
                for mid in positions:
                    if mid in board["ids"]:
                        write_register(ph, handler, mid, P_GAIN_ADDR, 0, length=1)
                        write_register(ph, handler, mid, D_GAIN_ADDR, 0, length=1)
        push_event("move_complete", {"label": label, "positions": positions})

    threading.Thread(target=_disable_torque, daemon=True).start()

    push_event("move_start", {"label": label, "positions": positions})
    return jsonify({"status": "moving", "results": results, "positions": positions})


# ------------------------------------------------------------------ #
# Routes — teleoperation                                              #
# ------------------------------------------------------------------ #

@app.route("/api/teleop/start", methods=["POST"])
def api_teleop_start():
    global teleop_active, teleop_equalizing, teleop_eq_targets, teleop_cal, teleop_positions, teleop_collision, teleop_slow_mode, teleop_cycle_count, teleop_log
    data = request.get_json(force=True) or {}
    slow = data.get("slow", False)
    exclude = set(data.get("exclude_joints", []))  # joint names to skip
    cal = _load_teleop_cal()
    if "leader" not in cal or "follower" not in cal:
        return jsonify({"error": "calibration files missing for leader or follower"}), 400
    with state_lock:
        f_port = assignment.get("follower")
        l_port = assignment.get("leader")
        if not f_port or f_port not in boards:
            return jsonify({"error": "follower not connected"}), 400
        if not l_port or l_port not in boards:
            return jsonify({"error": "leader not connected"}), 400
        f_board = boards[f_port]
        l_board = boards[l_port]

        # Pre-flight 1: refuse if any follower motor is overheated
        TELEOP_TEMP_LIMIT = TEMP_CUTOFF_C - 5
        hot = {ID_NAMES.get(mid, str(mid)): t
               for mid in f_board["ids"]
               if (t := f_board["temps"].get(mid)) is not None and t >= TELEOP_TEMP_LIMIT}
        if hot:
            return jsonify({"error": f"Motors too hot to start teleop: {hot}. Wait for cooling."}), 409

        # Pre-flight 2: refuse if follower joints are far outside calibrated range
        fc_map = cal.get("follower", {})
        lc_map = cal.get("leader", {})
        out_of_range = {}
        OUT_OF_RANGE_TOLERANCE = 50
        for mid in f_board["ids"]:
            name = ID_NAMES.get(mid)
            if not name:
                continue
            fc = fc_map.get(name)
            if fc is None:
                continue
            f_raw = f_board["positions"].get(mid)
            if f_raw is None:
                continue
            r_min = fc.get("range_min", 0)
            r_max = fc.get("range_max", 4095)
            if f_raw < r_min - OUT_OF_RANGE_TOLERANCE:
                out_of_range[name] = {"pos": f_raw, "range_min": r_min, "range_max": r_max,
                                      "below_by": r_min - f_raw}
            elif f_raw > r_max + OUT_OF_RANGE_TOLERANCE:
                out_of_range[name] = {"pos": f_raw, "range_min": r_min, "range_max": r_max,
                                      "above_by": f_raw - r_max}

        if out_of_range:
            print(f"[teleop] WARNING — follower joints outside calibrated range (will equalize safely):")
            for name, info in out_of_range.items():
                print(f"  {name}: pos={info['pos']} range=[{info['range_min']}, {info['range_max']}]")
            # Don't refuse — just log and proceed. The equalization ramp will
            # move these joints safely into range. Previous code deadlocked here
            # (push_event inside state_lock).

        ph, handler = f_board["ph"], f_board["handler"]
        for mid in f_board["ids"]:
            f_board["safety_disabled"][mid] = False
            f_board["safety_cause"][mid]    = None
            f_board["stall_counts"][mid]    = 0
            write_register(ph, handler, mid, SPEED_ADDR, TELEOP_SPEED_MIN, length=2)
            write_register(ph, handler, mid, TORQUE_ENABLE_ADDR, 1, length=1)

        # Seed ramp and wrap-detection from current follower positions
        teleop_positions = dict(f_board["positions"])
        teleop_prev_pos  = dict(f_board["positions"])
        teleop_halted.clear()
        teleop_collision = {}
        teleop_cycle_count = 0
        teleop_log.clear()
        # Equalization targets: map leader -> follower, clamped to safe range
        CAL_MARGIN = 0.05
        eq_targets = {}
        large_gaps = {}
        startup_state = {}
        for mid in f_board["ids"]:
            name = ID_NAMES.get(mid)
            if not name:
                continue
            if name in exclude:
                teleop_halted.add(mid)  # skip this joint entirely
                write_register(ph, handler, mid, TORQUE_ENABLE_ADDR, 0, length=1)
                continue
            lc = lc_map.get(name)
            fc = fc_map.get(name)
            l_raw = l_board["positions"].get(mid)
            f_raw = f_board["positions"].get(mid)
            if lc and fc and l_raw is not None:
                target = _map_position(l_raw, lc, fc)
                # Clamp target inside follower safe range
                r_min = fc.get("range_min", 0)
                r_max = fc.get("range_max", 4095)
                spread = r_max - r_min
                margin = int(spread * CAL_MARGIN)
                clamped = max(r_min + margin, min(r_max - margin, target))
                # P9: apply workspace limits to equalization targets too
                clamped = _apply_workspace_limits(clamped, name, "follower")
                gap = abs(f_raw - clamped) if f_raw is not None else 0
                eq_targets[mid] = clamped
                if gap > TELEOP_LARGE_GAP:
                    large_gaps[name] = int(gap)
                startup_state[name] = {
                    "f_pos": f_raw, "l_pos": l_raw,
                    "target": int(target), "clamped": int(clamped),
                    "gap": int(gap), "range": [r_min, r_max],
                }

        teleop_eq_targets  = eq_targets
        teleop_equalizing  = True
        teleop_slow_mode   = slow
        teleop_cal         = cal
        teleop_active      = True

    # Log startup state
    mode_desc = "slow/explore" if slow else "normal"
    print(f"\n[teleop] === START ({mode_desc}) ===")
    for name, s in startup_state.items():
        flag = " ** LARGE GAP" if name in large_gaps else ""
        print(f"  {name:<16} f={s['f_pos']:>5}  l={s['l_pos']:>5}"
              f"  target={s['clamped']:>5}  gap={s['gap']:>4}"
              f"  range=[{s['range'][0]},{s['range'][1]}]{flag}")

    push_event("teleop_started", {"phase": "equalizing", "joints": len(eq_targets),
                                   "mode": mode_desc, "large_gaps": large_gaps,
                                   "startup_state": startup_state})
    note = f"Equalizing {len(eq_targets)} joints"
    if large_gaps:
        note += f", {len(large_gaps)} with large gaps (will ramp safely): {list(large_gaps.keys())}"
    return jsonify({"ok": True, "phase": "equalizing", "mode": mode_desc,
                    "large_gaps": large_gaps, "startup_state": startup_state, "note": note})


@app.route("/api/teleop/debug", methods=["GET"])
def api_teleop_debug():
    """Return detailed teleop state for debugging."""
    with state_lock:
        f_port = assignment.get("follower")
        l_port = assignment.get("leader")
        f_safety = {}
        f_temps = {}
        if f_port and f_port in boards:
            fb = boards[f_port]
            for mid in fb["ids"]:
                n = ID_NAMES.get(mid, str(mid))
                f_safety[n] = {
                    "disabled": fb["safety_disabled"].get(mid),
                    "cause": fb["safety_cause"].get(mid),
                    "stall_count": fb["stall_counts"].get(mid),
                }
                f_temps[n] = fb["temps"].get(mid)
        return jsonify({
            "teleop_active": teleop_active,
            "teleop_equalizing": teleop_equalizing,
            "cycle_count": teleop_cycle_count,
            "halted": [ID_NAMES.get(m, str(m)) for m in teleop_halted],
            "collision": {ID_NAMES.get(m, str(m)): v for m, v in teleop_collision.items()},
            "positions_sent": {ID_NAMES.get(m, str(m)): v for m, v in teleop_positions.items()},
            "eq_targets": {ID_NAMES.get(m, str(m)): v for m, v in teleop_eq_targets.items()},
            "safety": f_safety,
            "temps": f_temps,
            "cal_loaded": {"leader": list(teleop_cal.get("leader", {}).keys()),
                           "follower": list(teleop_cal.get("follower", {}).keys())},
            "recent_log": teleop_log[-5:] if teleop_log else [],
        })


@app.route("/api/teleop/log", methods=["GET"])
def api_teleop_log():
    """Return recent teleop cycle log. ?last=N to limit (default 50)."""
    n = request.args.get("last", 50, type=int)
    return jsonify({
        "cycle_count": teleop_cycle_count,
        "active": teleop_active,
        "equalizing": teleop_equalizing,
        "entries": teleop_log[-n:],
    })


@app.route("/api/teleop/stop", methods=["POST"])
def api_teleop_stop():
    global teleop_active
    with state_lock:
        teleop_active = False
        f_port = assignment.get("follower")
        if f_port and f_port in boards:
            f_board = boards[f_port]
            ph, handler = f_board["ph"], f_board["handler"]
            for mid in f_board["ids"]:
                # Reset speed to safe default before disabling torque
                write_register(ph, handler, mid, SPEED_ADDR, TELEOP_SPEED_MIN, length=2)
                write_register(ph, handler, mid, TORQUE_ENABLE_ADDR, 0, length=1)
    push_event("teleop_stopped", {})
    return jsonify({"ok": True})


# ------------------------------------------------------------------ #
# Camera                                                              #
# ------------------------------------------------------------------ #

def camera_loop():
    global camera_frame, camera_active
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("[camera] Failed to open camera index 0")
        return
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    camera_active = True
    print("[camera] Started (640x480)")
    while True:
        ret, frame = cap.read()
        if ret:
            _, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
            with camera_lock:
                camera_frame = jpeg.tobytes()
        else:
            time.sleep(0.1)
        time.sleep(0.033)  # ~30 fps


def _gen_camera():
    while True:
        with camera_lock:
            frame = camera_frame
        if frame:
            yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")
        time.sleep(0.033)


@app.route("/camera/stream")
def camera_stream():
    return Response(_gen_camera(), mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/camera/snapshot")
def camera_snapshot():
    with camera_lock:
        frame = camera_frame
    if frame is None:
        return jsonify({"error": "no frame yet"}), 503
    return Response(frame, mimetype="image/jpeg")


@app.route("/camera/status")
def camera_status():
    with camera_lock:
        has_frame = camera_frame is not None
    return jsonify({"active": camera_active, "has_frame": has_frame})


# ------------------------------------------------------------------ #
# Main                                                                #
# ------------------------------------------------------------------ #

if __name__ == "__main__":
    if not SDK_AVAILABLE:
        print("ERROR: scservo_sdk not found. Activate your lerobot env:")
        print("  source ~/lerobot-env-312/bin/activate")
        raise SystemExit(1)

    SHARED_DIR.mkdir(parents=True, exist_ok=True)
    MSG_IN.mkdir(parents=True, exist_ok=True)
    MSG_OUT.mkdir(parents=True, exist_ok=True)
    MSG_TO_USER.mkdir(parents=True, exist_ok=True)
    MSG_FROM_USER.mkdir(parents=True, exist_ok=True)

    print("Scanning for controller boards...")
    boards = scan_boards()
    if boards:
        for path, board in boards.items():
            print(f"  {path}: {len(board['ids'])} motors ({board['ids']})")
        _auto_assign_by_pgain()
        for role in ("leader", "follower"):
            if role in assignment:
                print(f"  Auto-assigned {role} -> {assignment[role][-8:]}")
    else:
        print("  No boards found — will auto-detect when connected.")

    _load_persistent_cal_ranges()
    threading.Thread(target=poll_loop, daemon=True).start()
    threading.Thread(target=camera_loop, daemon=True).start()

    print(f"\nShared state: {STATE_FILE}")
    print(f"Mailbox in:   {MSG_IN}")
    print(f"Mailbox out:  {MSG_OUT}")
    print("\nOpen http://localhost:5833\n")
    app.run(host="127.0.0.1", port=5833, debug=False, threaded=True)
