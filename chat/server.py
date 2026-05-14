"""
SO-101 Chat Server — Claude as the robot brain.
Flask + Anthropic streaming, tool use, session memory.

Endpoints:
  POST /session/new   → SSE: Claude opening greeting with motor status
  POST /chat          → SSE: Claude response (agentic tool loop)
  GET  /status        → proxied motor state from :5833 (ui/app.py)
  GET  /teleop/status → teleop subprocess state
"""

import json
import os
import subprocess
import threading
import time
import urllib.request
import urllib.error
import uuid
from pathlib import Path
from flask import Flask, request, Response, send_from_directory
import anthropic

# ── Config ─────────────────────────────────────────────────────────────────
MOTOR_API   = "http://127.0.0.1:5833"
CAL_BASE    = Path.home() / ".cache/huggingface/lerobot/calibration"
FOLLOWER_CAL = CAL_BASE / "robots/so_follower/my_follower.json"
LEADER_CAL   = CAL_BASE / "teleoperators/so_leader/my_leader.json"
PYTHON      = "/Users/dereklomas/lerobot-env-312/bin/python3.12"
LEROBOT_BIN = "/Users/dereklomas/lerobot-env-312/bin/lerobot-teleoperate"
MODEL       = "claude-sonnet-4-6"

app = Flask(__name__, static_folder="static")
client = anthropic.Anthropic()

# ── State ───────────────────────────────────────────────────────────────────
sessions: dict[str, list] = {}         # session_id → message history
teleop_proc: subprocess.Popen | None = None
teleop_lock = threading.Lock()


# ── Helpers ─────────────────────────────────────────────────────────────────
def motor_get(path: str) -> dict:
    try:
        with urllib.request.urlopen(f"{MOTOR_API}{path}", timeout=2) as r:
            return json.loads(r.read())
    except Exception as e:
        return {"error": str(e)}


def motor_post(path: str, data: dict = {}) -> dict:
    try:
        body = json.dumps(data).encode()
        req  = urllib.request.Request(
            f"{MOTOR_API}{path}", data=body,
            headers={"Content-Type": "application/json"}, method="POST"
        )
        with urllib.request.urlopen(req, timeout=5) as r:
            return json.loads(r.read())
    except Exception as e:
        return {"error": str(e)}


def fmt_positions(pos: dict) -> str:
    if not pos:
        return "  (no data)"
    order = ["shoulder_pan", "shoulder_lift", "elbow_flex",
             "wrist_flex", "wrist_roll", "gripper"]
    lines = []
    for n in order:
        v = pos.get(n)
        lines.append(f"  {n:<16} {v if v is not None else '—'}")
    return "\n".join(lines)


def live_state_summary() -> str:
    s = motor_get("/api/state")
    if "error" in s:
        return f"UI server unreachable: {s['error']}"
    conn = s.get("connection", {})
    motors = s.get("motors", {})
    parts = []
    for role in ("follower", "leader"):
        joints = motors.get(role, {})
        connected_ports = [p for p, info in conn.items() if info.get("connected")]
        if joints:
            pos_lines = fmt_positions({name: d.get("position") for name, d in joints.items()})
            temps = {name: d.get("temperature") for name, d in joints.items() if d.get("temperature")}
            parts.append(f"{role.capitalize()} ({len(joints)}/6 motors):\n{pos_lines}")
            if temps:
                parts.append("  Temps: " + ", ".join(f"{n}={t}°C" for n, t in temps.items()))
    return "\n".join(parts) if parts else "No arms connected."


def system_prompt() -> str:
    state = live_state_summary()
    return f"""You are the SO-101 robot assistant — a friendly, capable AI brain for a \
physical robot arm system. You help the user set up, calibrate, teleoperate, and record \
demonstrations with their SO-101 arms.

CURRENT ROBOT STATE:
{state}

TOOLS you can use:
- check_motors: read live positions/ranges from follower arm
- scan_motors: probe USB ports for Feetech servo boards
- start_teleoperate: launch leader→follower teleoperation
- stop_teleoperate: stop teleop subprocess
- start_calibration: begin range-of-motion calibration pass
- save_calibration: persist current ranges to JSON files
- read_calibration: display saved calibration ranges
- run_shell: run a safe shell command (no destructive ops)

ROBOT SPECIFICS:
- Follower port: /dev/tty.usbmodem5B141123331 (ID: my_follower)
- Leader port:   /dev/tty.usbmodem5B141116761 (ID: my_leader)
- Protocol: Feetech STS3215, PacketHandler(0), 1 Mbps
- Motors: shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper (IDs 1-6)
- Calibration files: ~/.cache/huggingface/lerobot/calibration/...
- Python: {PYTHON}

Be concise but warm. When the user says something like "wave" or describes a motion, \
explain what you'd need to execute it (dataset + trained policy, or manual servo commands). \
Proactively offer to check arm status at the start of each session."""


# ── Tool definitions ────────────────────────────────────────────────────────
TOOLS = [
    {
        "name": "check_motors",
        "description": "Read current follower arm motor positions, min/max ranges, and connection status from the motor server.",
        "input_schema": {
            "type": "object",
            "properties": {
                "include_log": {
                    "type": "boolean",
                    "description": "Include last 10 log entries"
                }
            },
            "required": []
        }
    },
    {
        "name": "scan_motors",
        "description": "Scan USB ports to detect connected Feetech servo controller boards.",
        "input_schema": {
            "type": "object",
            "properties": {},
            "required": []
        }
    },
    {
        "name": "start_teleoperate",
        "description": "Start teleoperation: leader arm controls follower arm in real time.",
        "input_schema": {
            "type": "object",
            "properties": {},
            "required": []
        }
    },
    {
        "name": "stop_teleoperate",
        "description": "Stop the running teleoperation process.",
        "input_schema": {
            "type": "object",
            "properties": {},
            "required": []
        }
    },
    {
        "name": "start_calibration",
        "description": "Start a calibration pass. The user should move both arms through their full range of motion, then call save_calibration.",
        "input_schema": {
            "type": "object",
            "properties": {},
            "required": []
        }
    },
    {
        "name": "save_calibration",
        "description": "Save the current min/max ranges as calibration data.",
        "input_schema": {
            "type": "object",
            "properties": {},
            "required": []
        }
    },
    {
        "name": "read_calibration",
        "description": "Read and display the saved calibration files for follower and leader arms.",
        "input_schema": {
            "type": "object",
            "properties": {},
            "required": []
        }
    },
    {
        "name": "run_shell",
        "description": "Run a safe diagnostic shell command (read-only, no rm/kill/format). Use for checking processes, disk space, etc.",
        "input_schema": {
            "type": "object",
            "properties": {
                "command": {
                    "type": "string",
                    "description": "Shell command to run"
                }
            },
            "required": ["command"]
        }
    }
]

# Blocked patterns for run_shell safety
_BLOCKED = ["rm ", "kill", "mkfs", "format", "dd ", "sudo", "> /", "| sh", "| bash",
            "wget", "curl -o", "python -c", "eval", "exec"]


def run_tool(name: str, inp: dict) -> str:
    global teleop_proc

    if name == "check_motors":
        state = motor_get("/api/state")
        if "error" in state:
            return f"UI server unreachable: {state['error']}"
        return json.dumps(state, indent=2)

    elif name == "scan_motors":
        script = Path.home() / "so101/scripts/scan_motors.py"
        try:
            out = subprocess.check_output(
                [PYTHON, str(script)], timeout=15,
                stderr=subprocess.STDOUT, text=True
            )
            return out.strip() or "(no output)"
        except subprocess.TimeoutExpired:
            return "Scan timed out after 15s."
        except subprocess.CalledProcessError as e:
            return e.output or str(e)
        except FileNotFoundError:
            return f"Script not found: {script}"

    elif name == "start_teleoperate":
        with teleop_lock:
            if teleop_proc and teleop_proc.poll() is None:
                return "Teleoperation is already running."
            duration = inp.get("duration", 120)
            script = Path.home() / "so101/scripts/teleop_6joint.py"
            env = {**os.environ, "PATH": f"/Users/dereklomas/lerobot-env-312/bin:{os.environ.get('PATH', '')}"}
            try:
                teleop_proc = subprocess.Popen(
                    [PYTHON, str(script), str(duration)], env=env,
                    stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                    text=True
                )
                time.sleep(1.5)
                if teleop_proc.poll() is not None:
                    out = teleop_proc.stdout.read()
                    return f"Teleop exited immediately:\n{out}"
                return (
                    f"Teleoperation started (PID {teleop_proc.pid}, {duration}s). "
                    "Leader arm controls follower with proportional speed + safety limits. "
                    "Say 'stop teleoperation' to end early."
                )
            except Exception as e:
                return f"Failed to start teleoperation: {e}"

    elif name == "stop_teleoperate":
        with teleop_lock:
            if not teleop_proc or teleop_proc.poll() is not None:
                return "Teleoperation is not running."
            teleop_proc.terminate()
            try:
                teleop_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                teleop_proc.kill()
            teleop_proc = None
        return "Teleoperation stopped."

    elif name == "start_calibration":
        r = motor_post("/api/cal/reset")
        motor_post("/api/cal/start")
        return (
            "Calibration started — ranges reset and recording.\n"
            "Move both arms slowly through their FULL range of motion "
            "(every joint, from minimum to maximum).\n"
            "When done, say 'save calibration'."
        )

    elif name == "save_calibration":
        motor_post("/api/cal/stop")
        r = motor_post("/api/cal/save")
        if r.get("ok") or r.get("saved"):
            return "Calibration saved to calibration JSON files."
        return f"Save failed: {r}"

    elif name == "read_calibration":
        result = []
        for label, path in [("Follower", FOLLOWER_CAL), ("Leader", LEADER_CAL)]:
            try:
                cal = json.loads(path.read_text())
                result.append(f"{label} calibration ({path}):")
                for motor, vals in cal.items():
                    rmin = vals.get("range_min", "?")
                    rmax = vals.get("range_max", "?")
                    result.append(f"  {motor:<16} {rmin:>5} – {rmax}")
            except FileNotFoundError:
                result.append(f"{label} calibration: not found at {path}")
            except Exception as e:
                result.append(f"{label} calibration error: {e}")
        return "\n".join(result)

    elif name == "run_shell":
        cmd = inp.get("command", "").strip()
        if not cmd:
            return "No command provided."
        for blocked in _BLOCKED:
            if blocked in cmd:
                return f"Command blocked for safety: contains '{blocked}'"
        try:
            out = subprocess.check_output(
                cmd, shell=True, timeout=10,
                stderr=subprocess.STDOUT, text=True
            )
            return out.strip() or "(no output)"
        except subprocess.TimeoutExpired:
            return "Command timed out."
        except subprocess.CalledProcessError as e:
            return f"Exit {e.returncode}:\n{e.output}"

    return f"Unknown tool: {name}"


# ── Streaming agentic loop ──────────────────────────────────────────────────
def stream_claude(messages: list, session_id: str):
    """Generator: yields SSE strings from an agentic Claude loop."""
    sys = system_prompt()

    while True:
        # Collect full response (streaming + tool use)
        text_chunks = []
        tool_calls  = []
        stop_reason = None

        with client.messages.stream(
            model=MODEL,
            max_tokens=1024,
            system=sys,
            tools=TOOLS,
            messages=messages,
        ) as stream:
            current_tool = None
            current_input_str = ""

            for event in stream:
                etype = event.type

                if etype == "content_block_start":
                    blk = event.content_block
                    if blk.type == "tool_use":
                        current_tool = {"id": blk.id, "name": blk.name, "input": ""}
                        current_input_str = ""

                elif etype == "content_block_delta":
                    delta = event.delta
                    if delta.type == "text_delta":
                        text_chunks.append(delta.text)
                        yield f"data: {json.dumps({'type':'text','content':delta.text})}\n\n"
                    elif delta.type == "input_json_delta":
                        current_input_str += delta.partial_json

                elif etype == "content_block_stop":
                    if current_tool is not None:
                        try:
                            current_tool["input"] = json.loads(current_input_str) if current_input_str else {}
                        except Exception:
                            current_tool["input"] = {}
                        tool_calls.append(current_tool)
                        current_tool = None
                        current_input_str = ""

                elif etype == "message_delta":
                    stop_reason = getattr(event.delta, "stop_reason", None)

        # Build assistant message
        content_blocks = []
        if text_chunks:
            content_blocks.append({"type": "text", "text": "".join(text_chunks)})
        for tc in tool_calls:
            content_blocks.append({
                "type": "tool_use",
                "id":   tc["id"],
                "name": tc["name"],
                "input": tc["input"],
            })

        messages.append({"role": "assistant", "content": content_blocks})

        if stop_reason != "tool_use" or not tool_calls:
            break

        # Run tools, collect results
        tool_results = []
        for tc in tool_calls:
            notice = f"\n[Using tool: {tc['name']}…]"
            yield f"data: {json.dumps({'type':'text','content':notice})}\n\n"
            result = run_tool(tc["name"], tc["input"])
            tool_results.append({
                "type":        "tool_result",
                "tool_use_id": tc["id"],
                "content":     result,
            })

        messages.append({"role": "user", "content": tool_results})

    # Persist updated history
    sessions[session_id] = messages


# ── Routes ──────────────────────────────────────────────────────────────────
@app.route("/")
def index():
    return send_from_directory("static", "index.html")


@app.route("/session/new", methods=["POST"])
def new_session():
    sid = str(uuid.uuid4())[:12]
    opening = [{"role": "user", "content": "I just opened the SO-101 assistant. Please greet me, check the arm status, and let me know what I can do."}]
    sessions[sid] = opening

    def generate():
        yield from stream_claude(opening, sid)

    resp = Response(generate(), mimetype="text/event-stream")
    resp.headers["X-Session-Id"] = sid
    resp.headers["Cache-Control"] = "no-cache"
    return resp


@app.route("/chat", methods=["POST"])
def chat():
    data       = request.get_json(force=True)
    user_text  = data.get("message", "").strip()
    sid        = data.get("session_id") or str(uuid.uuid4())[:12]

    if not user_text:
        return Response("data: {}\n\n", mimetype="text/event-stream")

    history = sessions.get(sid, [])
    history.append({"role": "user", "content": user_text})
    sessions[sid] = history

    def generate():
        yield from stream_claude(history, sid)

    return Response(generate(), mimetype="text/event-stream",
                    headers={"Cache-Control": "no-cache", "X-Accel-Buffering": "no"})


@app.route("/status")
def status():
    """Proxy motor state from ui/app.py for the sidebar."""
    s = motor_get("/api/state")
    motors = s.get("motors", {})
    follower = motors.get("follower", {})
    positions = {name: d.get("position") for name, d in follower.items()}
    connected = any(info.get("connected") for info in s.get("connection", {}).values())
    return app.response_class(
        json.dumps({"connected": connected, "positions": positions}),
        mimetype="application/json"
    )


@app.route("/teleop/status")
def teleop_status():
    with teleop_lock:
        running = teleop_proc is not None and teleop_proc.poll() is None
    return app.response_class(
        json.dumps({"running": running}),
        mimetype="application/json"
    )


# ── Main ────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    print("SO-101 Chat Server  http://localhost:8888")
    print("  Model:  ", MODEL)
    print("  Motor API:", MOTOR_API)
    app.run(host="127.0.0.1", port=8888, debug=False, threaded=True)
