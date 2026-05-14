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
    """Start recording calibration ranges. Move each joint through its full range of motion while recording is active."""
    return _post("/api/cal/start")


@mcp.tool()
def stop_calibration() -> dict:
    """Stop recording calibration ranges (without saving)."""
    return _post("/api/cal/stop")


@mcp.tool()
def save_calibration() -> dict:
    """Stop recording and save calibration ranges to the JSON files. Returns warnings if any joint wasn't moved enough."""
    return _post("/api/cal/save")


@mcp.tool()
def reset_calibration() -> dict:
    """Reset recorded calibration ranges (start fresh)."""
    return _post("/api/cal/reset")


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


if __name__ == "__main__":
    mcp.run(transport="stdio")
