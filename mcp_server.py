"""
SO-101 MCP Server — exposes robot control as Claude Code native tools.

Wraps the UI app's HTTP API (localhost:5833) so Claude gets structured
tool access instead of shelling out to curl or scripts.

Safety principles (see shared/safety.json):
  P1 No teleport — large moves are ramped server-side
  P2 Know before move — pre-flight checks on every actuation
  P3 Margin of safety — 5% inside calibrated range
  P4 Torque is temporary — auto-disable after every move
  P5 Detect and halt — continuous load/temp/wrap monitoring
  P6 Equalize before track — teleop startup alignment
  P7 Human in the loop — warnings on anomalies and large moves

Run: /Users/dereklomas/lerobot-env-312/bin/python /Users/dereklomas/so101/mcp_server.py
"""

import json
import urllib.request
import urllib.error
from pathlib import Path
from mcp.server.fastmcp import FastMCP

UI_BASE = "http://localhost:5833"
SAFETY_FILE = Path(__file__).parent / "shared" / "safety.json"

# Load safety parameters
_safety = json.loads(SAFETY_FILE.read_text())
THRESHOLDS = _safety["thresholds"]
MOTION = _safety["motion"]

JOINT_IDS = {
    "shoulder_pan": 1, "shoulder_lift": 2, "elbow_flex": 3,
    "wrist_flex": 4, "wrist_roll": 5, "gripper": 6,
}

mcp = FastMCP("so101", instructions=(
    "SO-101 robot arm control. The UI server (localhost:5833) must be running. "
    "Tools talk to the follower and leader arms via the UI's HTTP API. "
    "Safety: all moves are pre-flighted (teleop check, temp/load check, large-move warning) "
    "and ramped (speed proportional to error, max delta per cycle). "
    "See shared/safety.json for principles and thresholds."
))


def _get(path: str) -> dict:
    """GET request to the UI server."""
    try:
        with urllib.request.urlopen(f"{UI_BASE}{path}", timeout=3) as r:
            return json.loads(r.read())
    except urllib.error.URLError as e:
        return {"error": f"UI server unreachable: {e}"}
    except Exception as e:
        return {"error": str(e)}


def _post(path: str, body: dict | None = None) -> dict:
    """POST request to the UI server."""
    try:
        data = json.dumps(body or {}).encode()
        req = urllib.request.Request(
            f"{UI_BASE}{path}",
            data=data,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with urllib.request.urlopen(req, timeout=10) as r:
            return json.loads(r.read())
    except urllib.error.URLError as e:
        return {"error": f"UI server unreachable: {e}"}
    except Exception as e:
        return {"error": str(e)}


# ── Pre-flight checks (P2: know before move) ─────────────────────────

def _preflight(label: str) -> tuple[dict | None, dict]:
    """Check state before any move. Returns (error_dict, state_dict).

    Checks:
      - UI server reachable
      - Teleop not active (P2)
      - Arm assigned and has motors
      - No motor above temp cutoff (P5)
    """
    state = _get("/api/state")
    if "error" in state:
        return state, {}

    # P2: teleop blocks external moves
    if state.get("teleop_active"):
        return {"error": "Teleop is active. Call stop_teleop first (P2: know before move)."}, state

    motors = state.get("motors", {}).get(label, {})
    if not motors:
        return {"error": f"No motors for '{label}'. Call assign_arm first."}, state

    # P5: refuse if any motor is over temp
    for name, info in motors.items():
        temp = info.get("temperature", 0)
        if temp and temp >= THRESHOLDS["temp_cutoff_c"]:
            return {"error": f"{name} is at {temp}C (limit {THRESHOLDS['temp_cutoff_c']}C). Let it cool down (P5)."}, state

    return None, state


def _check_large_move(label: str, positions: dict[int, int], state: dict) -> list[str]:
    """Return warnings for any joint moving more than large_move_warning counts (P7)."""
    warnings = []
    motors = state.get("motors", {}).get(label, {})
    id_to_name = {v: k for k, v in JOINT_IDS.items()}
    for mid, target in positions.items():
        name = id_to_name.get(mid, f"motor_{mid}")
        for mname, info in motors.items():
            if info.get("id") == mid:
                current = info.get("position", target)
                delta = abs(target - current)
                if delta > THRESHOLDS["large_move_warning"]:
                    warnings.append(f"{name}: {current}->{target} ({delta} counts, >{THRESHOLDS['large_move_warning']} threshold)")
                break
    return warnings


# ── Tools ────────────────────────────────────────────────────────────


@mcp.tool()
def get_state() -> dict:
    """Get current robot state: connections, motor positions, temperatures, loads, calibration status, teleop status, and recent events."""
    return _get("/api/state")


@mcp.tool()
def get_positions(label: str = "follower") -> dict:
    """Get current motor positions for an arm.

    Args:
        label: Which arm — "follower" or "leader"
    """
    state = _get("/api/state")
    if "error" in state:
        return state
    motors = state.get("motors", {}).get(label, {})
    if not motors:
        return {"error": f"No motor data for '{label}'. Is the arm connected and assigned?"}
    return {
        name: {
            "position": info.get("position"),
            "temperature": info.get("temperature"),
            "load": info.get("load"),
        }
        for name, info in motors.items()
    }


@mcp.tool()
def move_joint(joint: str, position: int, label: str = "follower") -> dict:
    """Move a single joint to a target position (0-4095).

    Safety: pre-flight checks (teleop, temp), position clamped to calibration
    limits with 5% margin, ramped motion (no teleporting), large-move warnings.
    Torque auto-disables after settling.

    Args:
        joint: Joint name (shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper)
        position: Target position in raw steps (0-4095)
        label: Which arm — "follower" or "leader"
    """
    mid = JOINT_IDS.get(joint)
    if mid is None:
        return {"error": f"Unknown joint '{joint}'. Valid: {list(JOINT_IDS.keys())}"}
    if not 0 <= position <= 4095:
        return {"error": "Position must be 0-4095"}

    # P2: pre-flight
    err, state = _preflight(label)
    if err:
        return err

    positions = {mid: position}

    # P7: warn on large moves
    warnings = _check_large_move(label, positions, state)

    result = _post("/api/move", {"label": label, "positions": {str(mid): position}, "ramped": True})
    if warnings:
        result["safety_warnings"] = warnings
    return result


@mcp.tool()
def move_joints(positions: dict, label: str = "follower") -> dict:
    """Move multiple joints at once.

    Safety: same pre-flight and ramping as move_joint.

    Args:
        positions: Dict of joint_name -> target_position, e.g. {"gripper": 2500, "wrist_roll": 2048}
        label: Which arm — "follower" or "leader"
    """
    motor_positions = {}
    for joint, pos in positions.items():
        mid = JOINT_IDS.get(joint)
        if mid is None:
            return {"error": f"Unknown joint '{joint}'. Valid: {list(JOINT_IDS.keys())}"}
        if not 0 <= int(pos) <= 4095:
            return {"error": f"Position for {joint} must be 0-4095"}
        motor_positions[mid] = int(pos)

    # P2: pre-flight
    err, state = _preflight(label)
    if err:
        return err

    # P7: warn on large moves
    warnings = _check_large_move(label, motor_positions, state)

    str_positions = {str(k): v for k, v in motor_positions.items()}
    result = _post("/api/move", {"label": label, "positions": str_positions, "ramped": True})
    if warnings:
        result["safety_warnings"] = warnings
    return result


@mcp.tool()
def move_to_middle(label: str = "follower") -> dict:
    """Move all joints to the midpoint of their calibrated range.

    Safety: pre-flight checks, ramped motion to prevent teleporting.
    Requires calibration data to exist.

    Args:
        label: Which arm — "follower" or "leader"
    """
    # P2: pre-flight
    err, state = _preflight(label)
    if err:
        return err

    return _post("/api/move", {"label": label, "preset": "middle", "ramped": True})


@mcp.tool()
def nudge_joint(joint: str, steps: int, label: str = "follower") -> dict:
    """Nudge a joint by a relative number of steps from its current position.

    Nudges are inherently safe (small relative moves) but still pre-flighted.
    Steps are capped at +-200 per call. Positive = increase position value.
    For the gripper: positive = open, negative = close.

    Args:
        joint: Joint name
        steps: Number of steps to move (positive or negative, max +-200)
        label: Which arm — "follower" or "leader"
    """
    mid = JOINT_IDS.get(joint)
    if mid is None:
        return {"error": f"Unknown joint '{joint}'. Valid: {list(JOINT_IDS.keys())}"}

    # Cap nudge size (P1: no teleport)
    steps = max(-200, min(200, steps))

    # P2: pre-flight
    err, state = _preflight(label)
    if err:
        return err

    # Read current position
    motors = state.get("motors", {}).get(label, {})
    current = None
    for name, info in motors.items():
        if info.get("id") == mid:
            current = info.get("position")
            break
    if current is None:
        return {"error": f"Could not read current position for {joint}"}

    target = max(0, min(4095, current + steps))
    result = _post("/api/move", {"label": label, "positions": {str(mid): target}})
    result["from"] = current
    result["to"] = target
    result["delta"] = steps
    return result


@mcp.tool()
def start_teleop() -> dict:
    """Start teleoperation (leader arm mirrors to follower).

    Safety: requires calibration data for both arms. Follower equalizes
    to leader position before tracking starts (P6). Motion is ramped
    with proportional speed control (P1).
    """
    return _post("/api/teleop/start")


@mcp.tool()
def stop_teleop() -> dict:
    """Stop teleoperation. Disables follower torque. Must be stopped before move commands will work."""
    return _post("/api/teleop/stop")


@mcp.tool()
def rescan() -> dict:
    """Rescan USB ports for connected motor controller boards."""
    return _post("/api/rescan")


@mcp.tool()
def assign_arm(port: str, role: str) -> dict:
    """Assign a USB port to a role (leader or follower).

    Args:
        port: USB port path, e.g. "/dev/tty.usbmodem5B141123331"
        role: "leader", "follower", or "clear"
    """
    return _post("/api/assign", {"port": port, "role": role})


@mcp.tool()
def start_calibration() -> dict:
    """Reset calibration ranges and start fresh. Calibration tracking is always on —
    this just clears accumulated ranges so you can do a clean sweep.
    Move each joint through its full range of motion, then call save_calibration."""
    result = _post("/api/cal/start")
    # Verify by reading state
    state = _get("/api/state")
    if isinstance(state, dict) and "motors" in state:
        motor_count = sum(len(m) for m in state["motors"].values() if isinstance(m, dict))
        result["motors_online"] = motor_count
    return result


@mcp.tool()
def stop_calibration() -> dict:
    """No-op — calibration tracking is always on. Use reset_calibration to clear ranges."""
    return {"ok": True, "note": "Calibration tracking is always on. Use save_calibration to save or reset_calibration to clear."}


@mcp.tool()
def save_calibration() -> dict:
    """Save current calibration ranges to JSON files. Works any time — calibration
    tracking is always on so ranges accumulate continuously. Returns warnings
    if any joint had less than 50 ticks of movement recorded."""
    # Check ranges before saving to give useful feedback
    state = _get("/api/state")
    cal_info = {}
    if isinstance(state, dict):
        cal_info["teleop_was_active"] = state.get("teleop_active", False)

    result = _post("/api/cal/save")
    if isinstance(result, dict) and "error" in result:
        result["hint"] = "No range data accumulated yet. Move joints around, then try again."
    return result


@mcp.tool()
def reset_calibration() -> dict:
    """Reset recorded calibration ranges (start fresh). Does not affect saved files."""
    return _post("/api/cal/reset")


@mcp.tool()
def check_calibration() -> dict:
    """Check if calibration data is valid (ranges wide enough for safe operation).

    Returns status per arm: 'ok', 'warning' (narrow ranges), 'invalid' (wiped/missing),
    or 'error'. Use this before any move or teleop to catch wiped calibration."""
    return _get("/api/cal/check")


@mcp.tool()
def list_calibration_history() -> dict:
    """List saved calibration backups. Every save creates a timestamped backup.
    Use restore_calibration to recover from a wipe."""
    return _get("/api/cal/history")


@mcp.tool()
def restore_calibration(filename: str) -> dict:
    """Restore a calibration from a history backup.

    Args:
        filename: Name from list_calibration_history, e.g. 'follower_20260514_133000_known_good.json'
    """
    return _post("/api/cal/restore", {"filename": filename})


@mcp.tool()
def get_events() -> dict:
    """Get recent event log (connections, disconnections, safety trips, calibration events, etc.)."""
    return _get("/api/events")


@mcp.tool()
def notify_user(title: str, body: str, type: str = "info") -> dict:
    """Send a notification to the user via the web UI.

    Args:
        title: Short notification title
        body: Longer description
        type: "info", "warning", or "action_request"
    """
    return _post("/api/notify", {"title": title, "body": body, "type": type})


@mcp.tool()
def selftest(label: str = "follower") -> dict:
    """Run a physical self-test on an arm: probe each joint with small movements
    to verify motor responsiveness and calibration validity.

    For each joint: reads position, nudges +-60 counts, checks it moved and returned.
    Reports pass/fail per joint plus calibration status. Takes ~10 seconds per arm.
    Teleop must be stopped first.

    Args:
        label: Which arm — "follower" or "leader"
    """
    # P2: pre-flight
    err, state = _preflight(label)
    if err:
        return err

    return _post("/api/selftest", {"label": label})


@mcp.tool()
def autocalibrate(joints: list[str] | None = None, label: str = "follower", save: bool = False) -> dict:
    """Cybernetic self-calibration: slowly probe each joint to find real physical
    limits by detecting load resistance (desk, cables, mechanical stops).

    Each joint creeps at 10 counts/cycle until load spikes or motor stalls,
    recording the boundary. Applies 5% safety margin. Much more accurate than
    manual sweep because it finds the actual workspace, not just encoder range.

    Takes 30-60 seconds per joint. Teleop must be stopped first.

    Args:
        joints: List of joints to probe, e.g. ["elbow_flex", "wrist_roll"]. Default: all joints.
        label: Which arm — "follower" or "leader"
        save: If True, write results to calibration file (with backup). Default: False (dry run).
    """
    # P2: pre-flight
    err, state = _preflight(label)
    if err:
        return err

    body = {"label": label, "save": save}
    if joints:
        body["joints"] = joints
    return _post("/api/autocalibrate", body)


@mcp.tool()
def workspace_probe(probe_joint: str, sweep_joint: str, sweep_steps: int = 8,
                    label: str = "follower", save: bool = True) -> dict:
    """Map workspace obstacles by probing one joint at multiple positions of another.

    Builds a pose-dependent obstacle map. Example: probe elbow_flex limits at
    8 shoulder_lift positions to find where the desk is at each height.

    Returns a table of (sweep_position, probe_min, probe_max) samples that
    describes the reachable workspace surface. Saves to shared/workspace.json.

    Takes 2-4 minutes total (30s per sweep step). Teleop must be stopped.

    Args:
        probe_joint: Joint to find limits of (e.g. "elbow_flex")
        sweep_joint: Joint to vary (e.g. "shoulder_lift")
        sweep_steps: Number of positions to sample (default 8)
        label: Which arm
        save: Persist results to workspace.json (default True)
    """
    err, state = _preflight(label)
    if err:
        return err

    return _post("/api/workspace/probe", {
        "label": label,
        "probe_joint": probe_joint,
        "sweep_joint": sweep_joint,
        "sweep_steps": sweep_steps,
        "save": save,
    })


@mcp.tool()
def get_workspace() -> dict:
    """Get the saved workspace obstacle map (from previous workspace_probe runs)."""
    return _get("/api/workspace")


@mcp.tool()
def get_cybernetics() -> dict:
    """Get the cybernetic intelligence state — adaptive gains, predictive collision,
    fatigue model, and bidirectional learning status.

    Shows how the robot's control system is adapting in real-time:
    - adaptive_gain: per-joint K values (grow when follower falls behind, decay when tracking well)
    - predictions: load slope analysis — detects rising forces before collision
    - fatigue: cumulative thermal stress derates collision thresholds for warm motors
    - collision_free_cycles: progress toward loosening tightened calibration limits

    No arguments needed. Returns the full cybernetic state snapshot.
    """
    return _get("/api/cybernetics")


@mcp.tool()
def search_conversations(query: str, max_results: int = 10) -> dict:
    """Search through conversation history (all Claude Code sessions with the user).

    Useful for finding past prompts, decisions, debugging sessions, and learnings.
    Searches the indexed transcripts first (run scripts/index_conversations.py to
    rebuild the index). Falls back to searching raw Claude Code transcripts.

    Args:
        query: Search term or phrase to look for
        max_results: Maximum number of matching excerpts to return (default 10)
    """
    import re
    pattern = re.compile(re.escape(query), re.IGNORECASE)

    # Try indexed data first
    index_file = Path(__file__).parent / "shared" / "conversation_history" / "index.jsonl"
    if index_file.exists():
        results = []
        for line_num, line in enumerate(index_file.read_text().splitlines(), 1):
            try:
                entry = json.loads(line)
                text = entry.get("text", "")
                if pattern.search(text):
                    results.append({
                        "session": entry.get("session_id", ""),
                        "line": line_num,
                        "role": entry.get("role", "?"),
                        "timestamp": entry.get("ts", ""),
                        "excerpt": text[:500],
                    })
                    if len(results) >= max_results:
                        break
            except Exception:
                continue
        return {"query": query, "source": "index", "matches": len(results), "results": results}

    # Fallback: search raw transcripts
    transcript_dir = Path.home() / ".claude/projects/-Users-dereklomas-so101"
    if not transcript_dir.exists():
        return {"error": "No conversation history found. Run scripts/index_conversations.py first."}

    results = []
    for f in sorted(transcript_dir.glob("*.jsonl")):
        try:
            for line in f.read_text().splitlines():
                record = json.loads(line)
                if record.get("type") not in ("user", "assistant"):
                    continue
                msg = record.get("message", {})
                content = msg.get("content", "")
                # Extract text from content blocks
                if isinstance(content, list):
                    text = " ".join(b.get("text", "") for b in content if isinstance(b, dict) and b.get("type") == "text")
                elif isinstance(content, str):
                    text = content
                else:
                    continue
                if pattern.search(text):
                    results.append({
                        "session": f.stem[:8],
                        "role": msg.get("role", "?"),
                        "timestamp": record.get("timestamp", ""),
                        "excerpt": text[:500],
                    })
                    if len(results) >= max_results:
                        break
        except Exception:
            continue
        if len(results) >= max_results:
            break

    return {"query": query, "source": "raw_transcripts", "matches": len(results), "results": results,
            "note": "Run scripts/index_conversations.py to build a faster index."}


@mcp.tool()
def list_conversations() -> dict:
    """List all recorded conversation history files with summary info."""
    history_dir = Path(__file__).parent / "shared" / "conversation_history"
    if not history_dir.exists():
        return {"conversations": [], "note": "No conversation history recorded yet."}

    files = []
    for f in sorted(history_dir.glob("*.jsonl")):
        try:
            lines = f.read_text().splitlines()
            first = json.loads(lines[0]) if lines else {}
            last = json.loads(lines[-1]) if lines else {}
            user_msgs = sum(1 for l in lines if '"role": "user"' in l)
            files.append({
                "file": f.name,
                "entries": len(lines),
                "user_messages": user_msgs,
                "first_ts": first.get("ts", ""),
                "last_ts": last.get("ts", ""),
            })
        except Exception:
            files.append({"file": f.name, "error": "could not parse"})

    return {"conversations": files, "total": len(files)}


# ── Digital Twin / Simulation Tools ───────────────────────────────────────────

@mcp.tool()
def sim_generate(policy: str = "reach", episodes: int = 10,
                 episode_length: int = 5, fps: int = 30,
                 task: str = "sim reach",
                 render_images: bool = False) -> dict:
    """Generate training data from the MuJoCo digital twin using scripted policies.

    Policies:
      - random: Smooth sinusoidal joint movements for data augmentation
      - reach: Reach to random targets, hold, retract (most useful)
      - pick_and_place: Scripted pick-and-place motion pattern

    Output is saved in the same format as the real robot recorder (JSON + optional JPG frames).
    """
    import subprocess
    sim_script = Path(__file__).parent / "sim" / "digital_twin.py"
    if not sim_script.exists():
        return {"error": "Digital twin not found at sim/digital_twin.py"}

    slug = task.lower().replace(" ", "-")
    output = str(Path(__file__).parent / "sim" / "datasets" / f"{slug}-{policy}")

    cmd = [
        "/Users/dereklomas/lerobot-env-312/bin/python3",
        str(sim_script), "headless",
        "--policy", policy,
        "--episodes", str(episodes),
        "--episode-length", str(episode_length),
        "--fps", str(fps),
        "--task", task,
        "--output", output,
    ]
    if render_images:
        cmd.append("--render-images")

    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=300)
        return {
            "status": "ok" if result.returncode == 0 else "error",
            "output_dir": output,
            "policy": policy,
            "episodes": episodes,
            "stdout": result.stdout[-500:] if result.stdout else "",
            "stderr": result.stderr[-500:] if result.stderr else "",
        }
    except subprocess.TimeoutExpired:
        return {"error": "Generation timed out after 300s"}
    except Exception as e:
        return {"error": str(e)}


@mcp.tool()
def sim_mirror() -> dict:
    """Launch the MuJoCo digital twin in mirror mode — shows the real robot's current pose in simulation.

    Requires the UI server (localhost:5833) to be running.
    Opens a MuJoCo viewer window. Close the window to stop.
    """
    import subprocess
    sim_script = Path(__file__).parent / "sim" / "digital_twin.py"
    if not sim_script.exists():
        return {"error": "Digital twin not found at sim/digital_twin.py"}

    # Launch in background since it's a GUI app
    subprocess.Popen([
        "/Users/dereklomas/lerobot-env-312/bin/python3",
        str(sim_script), "mirror",
    ])
    return {"status": "launched", "note": "MuJoCo viewer window opened. Close it to stop."}


@mcp.tool()
def sim_replay(dataset_path: str, fps: int = 30) -> dict:
    """Replay a recorded dataset in the MuJoCo simulation viewer.

    Works with both real robot datasets and sim-generated datasets.
    """
    import subprocess
    sim_script = Path(__file__).parent / "sim" / "digital_twin.py"
    if not sim_script.exists():
        return {"error": "Digital twin not found at sim/digital_twin.py"}

    if not Path(dataset_path).exists():
        return {"error": f"Dataset not found: {dataset_path}"}

    subprocess.Popen([
        "/Users/dereklomas/lerobot-env-312/bin/python3",
        str(sim_script), "replay",
        "--dataset", dataset_path,
        "--fps", str(fps),
    ])
    return {"status": "launched", "dataset": dataset_path,
            "note": "MuJoCo viewer replaying dataset. Close window to stop."}


if __name__ == "__main__":
    mcp.run(transport="stdio")
