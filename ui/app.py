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

from flask import Flask, Response, jsonify, request, send_from_directory

try:
    from scservo_sdk import PacketHandler, PortHandler
    SDK_AVAILABLE = True
except ImportError:
    SDK_AVAILABLE = False

app = Flask(__name__, static_folder="static")

ENV_FILE    = Path.home() / "so101/robot.env"
SHARED_DIR  = Path.home() / "so101/shared"
STATE_FILE  = SHARED_DIR / "robot_state.json"
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
PING_INTERVAL        = 0.05
STATE_WRITE_INTERVAL = 0.5   # write shared state at 2 Hz
AUTO_SCAN_INTERVAL   = 3.0   # check for new/gone boards every 3s

# ── Safety thresholds (from shared/safety.json) ───────────────────
SAFETY_FILE = Path(__file__).parent / "shared" / "safety.json"
try:
    _safety = json.loads(SAFETY_FILE.read_text())
    TEMP_CUTOFF_C        = _safety["thresholds"]["temp_cutoff_c"]
    STALL_LOAD_THRESHOLD = _safety["thresholds"]["stall_load_threshold"]
    STALL_COUNT_LIMIT    = _safety["thresholds"]["stall_count_limit"]
except Exception:
    TEMP_CUTOFF_C        = 65
    STALL_LOAD_THRESHOLD = 800
    STALL_COUNT_LIMIT    = 6

CAL_PATHS = {
    "follower": Path.home() / ".cache/huggingface/lerobot/calibration/robots/so_follower/my_follower.json",
    "leader":   Path.home() / ".cache/huggingface/lerobot/calibration/teleoperators/so_leader/my_leader.json",
}

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
cal_active      = False
cal_ranges      = {}   # role -> {motor_id -> {min, max}}
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

TELEOP_WRAP_THRESH = 1500   # counts — position jump this large = encoder wrap, halt joint

# Max counts to move per poll cycle (20 Hz). 60 counts/step = 1200 counts/s ≈ 3.4 s full sweep.
TELEOP_MAX_DELTA   = 60
TELEOP_EQ_THRESH   = 25    # counts — follower must be within this of target to end equalization

# Proportional speed control for teleop (matches teleop_6joint.py)
TELEOP_SPEED_K     = 1.2   # speed units per count of error
TELEOP_SPEED_FF    = 0.6   # feedforward gain from leader velocity
TELEOP_SPEED_MIN   = 80    # minimum speed (keeps motion smooth near target)
TELEOP_SPEED_MAX   = 500   # maximum speed (prevents violent motion)


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
        if length == 1:
            val, result, _ = handler.read1ByteTxRx(ph, mid, addr)
        else:
            val, result, _ = handler.read2ByteTxRx(ph, mid, addr)
        if result == 0:
            return val
    return None


def write_register(ph, handler, mid, addr, value, length=2):
    if length == 1:
        result, _ = handler.write1ByteTxRx(ph, mid, addr, value)
    else:
        result, _ = handler.write2ByteTxRx(ph, mid, addr, value)
    return result == 0


def _load_teleop_cal():
    """Load both cal files for position mapping. Returns {role: {name: {...}}}."""
    result = {}
    for role, path in CAL_PATHS.items():
        try:
            result[role] = json.loads(path.read_text())
        except Exception:
            pass
    return result


def _map_position(leader_raw, lc, fc):
    """Linear map from leader cal range to follower cal range, clamped."""
    l_min, l_max = lc["range_min"], lc["range_max"]
    f_min, f_max = fc["range_min"], fc["range_max"]
    if l_max <= l_min:
        return (f_min + f_max) // 2
    t = (leader_raw - l_min) / (l_max - l_min)
    t = max(0.0, min(1.0, t))
    return round(f_min + t * (f_max - f_min))


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
                "errors":    0,
                "label":     None,
            }
        else:
            ph.closePort()
            new_empty.add(path)
    empty_ports = new_empty
    return found


# ------------------------------------------------------------------ #
# Background threads                                                  #
# ------------------------------------------------------------------ #

def poll_loop():
    """Read positions, temps, loads from all boards continuously."""
    global teleop_active, teleop_equalizing, teleop_eq_targets, teleop_cal, teleop_positions, teleop_prev_pos, teleop_halted
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
                            continue

                        # Clear any stall trip so all motors stay active during teleop
                        if f_board["safety_disabled"].get(mid):
                            f_board["safety_disabled"][mid] = False
                            f_board["stall_counts"][mid]    = 0
                            write_register(ph, handler, mid, TORQUE_ENABLE_ADDR, 1, length=1)
                        name = ID_NAMES.get(mid)
                        if not name:
                            continue
                        lc = lc_map.get(name)
                        fc = fc_map.get(name)
                        if lc is None or fc is None:
                            continue

                        if teleop_equalizing:
                            # Phase 1: ramp toward snapshot of leader position at start
                            target = teleop_eq_targets.get(mid)
                            if target is None:
                                continue
                            prev = teleop_positions.get(mid, f_board["positions"].get(mid, target))
                            delta = target - prev
                            if abs(delta) > TELEOP_MAX_DELTA:
                                target = prev + TELEOP_MAX_DELTA * (1 if delta > 0 else -1)
                                all_equalized = False
                            elif abs(target - f_board["positions"].get(mid, target)) > TELEOP_EQ_THRESH:
                                all_equalized = False
                        else:
                            # Phase 2: live tracking
                            l_raw = l_board["positions"].get(mid)
                            if l_raw is None:
                                continue
                            target = _map_position(l_raw, lc, fc)
                            prev = teleop_positions.get(mid, f_board["positions"].get(mid, target))
                            delta = target - prev
                            if abs(delta) > TELEOP_MAX_DELTA:
                                target = prev + TELEOP_MAX_DELTA * (1 if delta > 0 else -1)

                        teleop_positions[mid] = target
                        # Proportional speed with leader velocity feedforward:
                        #   speed = K * |error| + FF * |leader_vel|
                        # Feedforward anticipates leader motion so follower
                        # doesn't fall behind during fast movements.
                        current_pos = f_board["positions"].get(mid, target)
                        error = abs(target - current_pos)
                        leader_vel = 0
                        if not teleop_equalizing and mid in l_board["ids"]:
                            l_ph, l_handler = l_board["ph"], l_board["handler"]
                            vel_raw, l_res, l_err = l_handler.read2ByteTxRx(l_ph, mid, PRESENT_SPEED_ADDR)
                            if l_res == 0 and l_err == 0:
                                # STS3215 speed: bit 10 = direction, bits 0-9 = magnitude
                                leader_vel = vel_raw & 0x3FF
                        speed = int(max(TELEOP_SPEED_MIN, min(TELEOP_SPEED_MAX,
                                        TELEOP_SPEED_K * error + TELEOP_SPEED_FF * leader_vel)))
                        write_register(ph, handler, mid, SPEED_ADDR, speed, length=2)
                        write_register(ph, handler, mid, GOAL_POSITION_ADDR, target, length=2)

                    # Transition out of equalization once all motors are close
                    if teleop_equalizing and all_equalized:
                        teleop_equalizing = False
                        pending_events.append(("teleop_tracking", {}))

        # Push safety events outside state_lock (push_event acquires it)
        for kind, detail in pending_events:
            push_event(kind, detail)

        # Write shared state file at 2 Hz
        if now - last_state_write >= STATE_WRITE_INTERVAL:
            last_state_write = now
            _write_shared_state()

        # Process any incoming messages
        _process_messages()

        time.sleep(PING_INTERVAL)


def _restore_labels():
    """Re-apply assignment labels to boards after a rescan. Call inside state_lock."""
    for role, port in assignment.items():
        if port in boards:
            boards[port]["label"] = role


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
                "recording": cal_active,
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
            yield f"data: {json.dumps({'boards': snapshot, 'cal': cal_snap, 'events': events, 'notifications': notifs, 'empty_ports': empty, 'teleop': teleop_active, 'teleop_phase': 'equalizing' if teleop_equalizing else 'tracking', 'teleop_halted': list(teleop_halted)})}\n\n"
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

    for role, motors in ranges_copy.items():
        cal_path = CAL_PATHS.get(role)
        if not cal_path or not cal_path.exists():
            skipped.append(f"{role}: no calibration file at {cal_path}")
            continue
        with open(cal_path) as f:
            cal = json.load(f)
        for mid, name in ID_NAMES.items():
            if name not in cal or mid not in motors:
                continue
            r = motors[mid]
            spread = r["max"] - r["min"]
            if spread < 50:
                warnings.append(f"{role}/{name}: only {spread} ticks — may not have been moved")
            cal[name]["range_min"] = r["min"]
            cal[name]["range_max"] = r["max"]
        with open(cal_path, "w") as f:
            json.dump(cal, f, indent=4)
        saved.append(str(cal_path))

    push_event("calibration_saved", {"saved": saved, "warnings": warnings})
    return jsonify({"ok": True, "saved": saved, "skipped": skipped, "warnings": warnings})


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

    # Set move grace period — suppress stall detection while motor escapes high-load zone
    grace_until = time.time() + MOVE_GRACE_S
    for mid in positions:
        move_grace[mid] = grace_until

    if ramped:
        # P1: No teleport — ramp toward targets in background thread
        def _ramped_move():
            with state_lock:
                ph, handler = board["ph"], board["handler"]
                for mid in positions:
                    if mid in board["ids"]:
                        board["safety_disabled"][mid] = False
                        board["stall_counts"][mid]    = 0
                        write_register(ph, handler, mid, TORQUE_ENABLE_ADDR, 1, length=1)

            max_delta = TELEOP_MAX_DELTA  # counts per cycle
            remaining = dict(positions)   # {mid: final_goal}

            for _ in range(200):  # max 200 cycles = 10s at 20Hz
                if not remaining:
                    break
                done = []
                with state_lock:
                    ph, handler = board["ph"], board["handler"]
                    for mid, goal in remaining.items():
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
            push_event("move_complete", {"label": label, "positions": positions, "ramped": True})

        threading.Thread(target=_ramped_move, daemon=True).start()
        push_event("move_start", {"label": label, "positions": positions, "ramped": True})
        return jsonify({"status": "ramping", "positions": positions})

    # Legacy: instant move (no ramping)
    results = {}
    with state_lock:
        ph, handler = board["ph"], board["handler"]
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
        push_event("move_complete", {"label": label, "positions": positions})

    threading.Thread(target=_disable_torque, daemon=True).start()

    push_event("move_start", {"label": label, "positions": positions})
    return jsonify({"status": "moving", "results": results, "positions": positions})


# ------------------------------------------------------------------ #
# Routes — teleoperation                                              #
# ------------------------------------------------------------------ #

@app.route("/api/teleop/start", methods=["POST"])
def api_teleop_start():
    global teleop_active, teleop_equalizing, teleop_eq_targets, teleop_cal, teleop_positions
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
        ph, handler = f_board["ph"], f_board["handler"]
        for mid in f_board["ids"]:
            f_board["safety_disabled"][mid] = False
            f_board["stall_counts"][mid]    = 0
            write_register(ph, handler, mid, TORQUE_ENABLE_ADDR, 1, length=1)
        # Seed ramp and wrap-detection from current follower positions
        teleop_positions = dict(f_board["positions"])
        teleop_prev_pos  = dict(f_board["positions"])
        teleop_halted.clear()
        # Snapshot leader positions mapped to follower — equalization target
        lc_map = cal.get("leader", {})
        fc_map = cal.get("follower", {})
        eq_targets  = {}
        large_gaps  = {}  # {mid: gap} — joints too far to equalize automatically
        EQ_MAX_GAP  = 600  # counts — above this, joint is skipped during equalization
        for mid in f_board["ids"]:
            name = ID_NAMES.get(mid)
            if not name:
                continue
            lc = lc_map.get(name)
            fc = fc_map.get(name)
            l_raw = l_board["positions"].get(mid)
            if lc and fc and l_raw is not None:
                target = _map_position(l_raw, lc, fc)
                f_now  = f_board["positions"].get(mid, target)
                gap    = abs(f_now - target)
                if gap > EQ_MAX_GAP:
                    large_gaps[mid] = gap
                    teleop_halted.add(mid)  # skip this joint — user must position manually
                else:
                    eq_targets[mid] = target
        teleop_eq_targets  = eq_targets
        teleop_equalizing  = True
        teleop_cal         = cal
        teleop_active      = True
    skipped = {ID_NAMES.get(mid, mid): gap for mid, gap in large_gaps.items()}
    push_event("teleop_started", {"phase": "equalizing", "skipped": skipped})
    return jsonify({"ok": True, "phase": "equalizing", "skipped": skipped,
                    "note": f"Joints with gap > {EQ_MAX_GAP} skipped (position manually): {list(skipped.keys())}" if skipped else "all joints equalizing"})


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
    else:
        print("  No boards found — will auto-detect when connected.")

    threading.Thread(target=poll_loop, daemon=True).start()

    print(f"\nShared state: {STATE_FILE}")
    print(f"Mailbox in:   {MSG_IN}")
    print(f"Mailbox out:  {MSG_OUT}")
    print("\nOpen http://localhost:5833\n")
    app.run(host="127.0.0.1", port=5833, debug=False, threaded=True)
