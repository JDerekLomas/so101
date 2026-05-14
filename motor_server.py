"""
SO-101 Motor Server — Claude-facing API daemon.
Reads follower motor positions, logs, tracks ranges.
Bridges to the UI (localhost:5833) via shared/messages/.

Endpoints:
  GET  /state              current positions + min/max ranges
  GET  /log                last 100 position entries
  POST /reset_ranges       reset min/max tracking
  POST /save_calibration   write current ranges to my_follower.json
  GET  /mailbox            read messages (from all sources)
  POST /mailbox            post a message
  DELETE /mailbox/<id>     mark message done
  GET  /handoff            full session context snapshot
  POST /notify             send a notification to the UI (shows in browser)
  GET  /from_user          poll for responses dropped by user in browser

Run: /Users/dereklomas/lerobot-env-312/bin/python /Users/dereklomas/so101/motor_server.py
"""
import time
import json
import os
import threading
import collections
import uuid
from http.server import HTTPServer, BaseHTTPRequestHandler
from pathlib import Path
from scservo_sdk import PortHandler, PacketHandler

PORT_PATH   = "/dev/tty.usbmodem5B141123331"
HTTP_PORT   = 7777
ADDR_POS    = 56
NAMES       = {1: "shoulder_pan", 2: "shoulder_lift", 3: "elbow_flex",
               4: "wrist_flex",   5: "wrist_roll",    6: "gripper"}
BASE        = Path.home() / "so101"
CAL_PATH    = Path.home() / ".cache/huggingface/lerobot/calibration/robots/so_follower/my_follower.json"
LOG_FILE    = BASE / "motor_log.jsonl"
SHARED      = BASE / "shared"
TO_USER     = SHARED / "messages" / "to_user"
FROM_USER   = SHARED / "messages" / "from_user"
MAILBOX_F   = BASE / "mailbox.json"

for d in (TO_USER, FROM_USER):
    d.mkdir(parents=True, exist_ok=True)

# Shared state
lock = threading.Lock()
state = {
    "connected": False,
    "positions": {},
    "mins": {NAMES[i]: 4095 for i in range(1, 7)},
    "maxs": {NAMES[i]: 0    for i in range(1, 7)},
    "last_update": None,
    "uptime_start": time.time(),
}
log_buf = collections.deque(maxlen=1000)


def reader_loop():
    port = PortHandler(PORT_PATH)
    handler = PacketHandler(0)
    while True:
        if not port.is_open:
            if not port.openPort():
                time.sleep(2)
                continue
            port.setBaudRate(1_000_000)
        positions = {}
        for mid in range(1, 7):
            val, comm, _ = handler.read2ByteTxRx(port, mid, ADDR_POS)
            if comm == 0:
                positions[NAMES[mid]] = val
        ts = time.time()
        with lock:
            state["connected"]   = bool(positions)
            state["positions"]   = positions
            state["last_update"] = ts
            for name, val in positions.items():
                if val < state["mins"][name]: state["mins"][name] = val
                if val > state["maxs"][name]: state["maxs"][name] = val
        log_buf.append({"ts": ts, "positions": positions})
        if int(ts * 20) % 20 == 0:
            with open(LOG_FILE, "a") as f:
                f.write(json.dumps({"ts": ts, "positions": positions}) + "\n")
        time.sleep(0.05)


def load_mailbox():
    try:
        return json.loads(MAILBOX_F.read_text())
    except Exception:
        return []

def save_mailbox(msgs):
    MAILBOX_F.write_text(json.dumps(msgs, indent=2))

mbox_lock = threading.Lock()


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a): pass

    def send_json(self, data, code=200):
        body = json.dumps(data, indent=2).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", len(body))
        self.end_headers()
        self.wfile.write(body)

    def body(self):
        n = int(self.headers.get("Content-Length", 0))
        return json.loads(self.rfile.read(n)) if n else {}

    def do_GET(self):
        p = self.path.split("?")[0]

        if p == "/state":
            with lock:
                snap = {k: (dict(v) if isinstance(v, dict) else v) for k, v in state.items()}
            self.send_json(snap)

        elif p == "/log":
            self.send_json({"entries": list(log_buf)[-100:]})

        elif p == "/mailbox":
            with mbox_lock:
                msgs = load_mailbox()
            self.send_json({"count": len(msgs), "messages": msgs})

        elif p == "/from_user":
            # Collect any responses the user dropped via the browser
            responses = []
            for f in sorted(FROM_USER.glob("*.json")):
                try:
                    responses.append(json.loads(f.read_text()))
                    f.unlink()
                except Exception:
                    pass
            self.send_json({"count": len(responses), "responses": responses})

        elif p == "/handoff":
            with lock:
                snap = {k: (dict(v) if isinstance(v, dict) else v) for k, v in state.items()}
            with mbox_lock:
                msgs = load_mailbox()
            try:
                cal = json.loads(CAL_PATH.read_text())
            except Exception:
                cal = None
            self.send_json({
                "generated_at": time.time(),
                "robot": {
                    "follower_port": PORT_PATH,
                    "leader_port":   "/dev/tty.usbmodem5B141116761",
                    "follower_id":   "my_follower",
                    "leader_id":     "my_leader",
                },
                "environment": {
                    "venv":    "/Users/dereklomas/lerobot-env-312",
                    "python":  "/Users/dereklomas/lerobot-env-312/bin/python",
                    "lerobot": "/Users/dereklomas/so101/lerobot",
                    "scripts": "/Users/dereklomas/so101/scripts/",
                    "ui":      "http://localhost:5833",
                    "api":     "http://localhost:7777",
                },
                "motor_state":  snap,
                "calibration":  cal,
                "pending_messages": msgs,
                "teleoperate_command": (
                    "lerobot-teleoperate "
                    "--robot.type=so101_follower "
                    "--robot.port=/dev/tty.usbmodem5B141123331 --robot.id=my_follower "
                    "--teleop.type=so101_leader "
                    "--teleop.port=/dev/tty.usbmodem5B141116761 --teleop.id=my_leader"
                ),
            })

        else:
            self.send_json({"error": "not found"}, 404)

    def do_POST(self):
        p = self.path.split("?")[0]

        if p == "/reset_ranges":
            with lock:
                state["mins"] = {NAMES[i]: 4095 for i in range(1, 7)}
                state["maxs"] = {NAMES[i]: 0    for i in range(1, 7)}
            self.send_json({"ok": True})

        elif p == "/save_calibration":
            with lock:
                mins, maxs = dict(state["mins"]), dict(state["maxs"])
            try:
                cal = json.loads(CAL_PATH.read_text())
                for name in NAMES.values():
                    if name in cal:
                        cal[name]["range_min"] = mins[name]
                        cal[name]["range_max"] = maxs[name]
                CAL_PATH.write_text(json.dumps(cal, indent=4))
                self.send_json({"ok": True, "mins": mins, "maxs": maxs})
            except Exception as e:
                self.send_json({"ok": False, "error": str(e)}, 500)

        elif p == "/mailbox":
            b = self.body()
            msg = {"id": str(uuid.uuid4())[:8], "ts": time.time(),
                   "from": b.get("from", "unknown"), "text": b.get("text", ""),
                   "tags": b.get("tags", []), "done": False}
            with mbox_lock:
                msgs = load_mailbox()
                msgs.append(msg)
                save_mailbox(msgs)
            self.send_json({"ok": True, "id": msg["id"]})

        elif p == "/notify":
            # Post a notification to the UI (shows in browser at localhost:5833)
            b = self.body()
            msg_id = f"{int(time.time())}_{b.get('type','notify')}"
            payload = {
                "id": msg_id, "from": "claude", "to": "user",
                "type": b.get("type", "notify"), "ts": time.time(),
                "text": b.get("text", ""), "payload": b.get("payload", {}),
            }
            (TO_USER / f"{msg_id}.json").write_text(json.dumps(payload, indent=2))
            self.send_json({"ok": True, "id": msg_id})

        else:
            self.send_json({"error": "not found"}, 404)

    def do_DELETE(self):
        if self.path.startswith("/mailbox/"):
            msg_id = self.path.split("/")[-1]
            with mbox_lock:
                msgs = load_mailbox()
                for m in msgs:
                    if m["id"] == msg_id:
                        m["done"] = True
                save_mailbox(msgs)
            self.send_json({"ok": True})
        else:
            self.send_json({"error": "not found"}, 404)


if __name__ == "__main__":
    threading.Thread(target=reader_loop, daemon=True).start()
    server = HTTPServer(("127.0.0.1", HTTP_PORT), Handler)
    print(f"SO-101 Motor Server  http://localhost:{HTTP_PORT}")
    print(f"  UI (human):  http://localhost:5833")
    print(f"  API (Claude): http://localhost:{HTTP_PORT}")
    print(f"  GET  /state  /log  /mailbox  /handoff  /from_user")
    print(f"  POST /reset_ranges  /save_calibration  /mailbox  /notify")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down.")
