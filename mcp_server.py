"""
SO-101 MCP Server — exposes robot control as Claude Code native tools.

Wraps the UI app's HTTP API (localhost:5833) so Claude gets structured
tool access instead of shelling out to curl or scripts.

Run: /Users/dereklomas/lerobot-env-312/bin/python /Users/dereklomas/so101/mcp_server.py
"""

import json
import urllib.request
import urllib.error
from mcp.server.fastmcp import FastMCP

UI_BASE = "http://localhost:5833"

mcp = FastMCP("so101", instructions=(
    "SO-101 robot arm control. The UI server (localhost:5833) must be running. "
    "Tools talk to the follower and leader arms via the UI's HTTP API. "
    "Safety: position clamping, stall detection, and temp cutoff are handled by the UI server."
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
        with urllib.request.urlopen(req, timeout=5) as r:
            return json.loads(r.read())
    except urllib.error.URLError as e:
        return {"error": f"UI server unreachable: {e}"}
    except Exception as e:
        return {"error": str(e)}


# ── Tools ────────────────────────────────────────────────────────────


@mcp.tool()
def get_state() -> dict:
    """Get current robot state: connections, motor positions, temperatures, loads, calibration status, and recent events."""
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

    Position is clamped to calibration limits by the server. Torque is
    automatically disabled after the move settles (~2.5s).

    Args:
        joint: Joint name (shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper)
        position: Target position in raw steps (0-4095)
        label: Which arm — "follower" or "leader"
    """
    joint_to_id = {
        "shoulder_pan": 1, "shoulder_lift": 2, "elbow_flex": 3,
        "wrist_flex": 4, "wrist_roll": 5, "gripper": 6,
    }
    mid = joint_to_id.get(joint)
    if mid is None:
        return {"error": f"Unknown joint '{joint}'. Valid: {list(joint_to_id.keys())}"}
    if not 0 <= position <= 4095:
        return {"error": "Position must be 0-4095"}
    return _post("/api/move", {"label": label, "positions": {str(mid): position}})


@mcp.tool()
def move_joints(positions: dict, label: str = "follower") -> dict:
    """Move multiple joints at once.

    Args:
        positions: Dict of joint_name -> target_position, e.g. {"gripper": 2500, "wrist_roll": 2048}
        label: Which arm — "follower" or "leader"
    """
    joint_to_id = {
        "shoulder_pan": 1, "shoulder_lift": 2, "elbow_flex": 3,
        "wrist_flex": 4, "wrist_roll": 5, "gripper": 6,
    }
    motor_positions = {}
    for joint, pos in positions.items():
        mid = joint_to_id.get(joint)
        if mid is None:
            return {"error": f"Unknown joint '{joint}'. Valid: {list(joint_to_id.keys())}"}
        if not 0 <= int(pos) <= 4095:
            return {"error": f"Position for {joint} must be 0-4095"}
        motor_positions[str(mid)] = int(pos)
    return _post("/api/move", {"label": label, "positions": motor_positions})


@mcp.tool()
def move_to_middle(label: str = "follower") -> dict:
    """Move all joints to the midpoint of their calibrated range.

    Requires calibration data to exist.

    Args:
        label: Which arm — "follower" or "leader"
    """
    return _post("/api/move", {"label": label, "preset": "middle"})


@mcp.tool()
def nudge_joint(joint: str, steps: int, label: str = "follower") -> dict:
    """Nudge a joint by a relative number of steps from its current position.

    Positive steps = increase position value, negative = decrease.
    For the gripper: positive = open, negative = close.

    Args:
        joint: Joint name
        steps: Number of steps to move (positive or negative, e.g. 80 or -80)
        label: Which arm — "follower" or "leader"
    """
    joint_to_id = {
        "shoulder_pan": 1, "shoulder_lift": 2, "elbow_flex": 3,
        "wrist_flex": 4, "wrist_roll": 5, "gripper": 6,
    }
    mid = joint_to_id.get(joint)
    if mid is None:
        return {"error": f"Unknown joint '{joint}'. Valid: {list(joint_to_id.keys())}"}

    # Get current position
    state = _get("/api/state")
    if "error" in state:
        return state
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
