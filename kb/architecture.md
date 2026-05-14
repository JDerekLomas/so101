# System Architecture

## Stack Overview

```
Human operator
    |
Claude Code (CLI agent)
    |
MCP Server (mcp_server.py) --- 14 tools
    |
UI Server (ui/app.py :5833) --- HTTP API, owns serial ports
    |
Serial bus (scservo_sdk, 1 Mbps)
    |
Feetech STS3215 servos (6 per arm)
```

## Components

### UI Server (`~/so101/ui/app.py` on :5833)

Primary hardware interface. Owns both serial ports.

Functions:
- Port detection and auto-assignment
- Live position streaming (2 Hz poll loop)
- Calibration recording
- Motor health diagnostics
- Ramped move control with safety guards
- Teleop state management
- Load/temperature monitoring

### MCP Server (`~/so101/mcp_server.py`)

Claude Code native tool interface. Wraps UI HTTP API.

Tools:
- `get_state()` — full robot state
- `get_events()` — event history
- `get_positions()` — motor positions
- `rescan()` — USB port detection
- `assign_arm()` — port-to-arm assignment
- `nudge_joint()` — small position adjustments (+-200 counts)
- `move_joint()` — direct joint targeting (ramped by default)
- `move_to_middle()` — reset to midpoints
- `start_teleop()` / `stop_teleop()` — teleop control
- `get_calibration()` — calibration data
- `start_calibration()` / `stop_calibration()` — range recording

Pre-flight checks (P2): refuses moves during teleop, checks temps, warns on large moves.

Config: `~/so101/.mcp.json`

### Chat Server (`~/so101/chat/server.py` on :8888)

Flask + Anthropic streaming. Secondary interface (demo/blog post).

8 tools: check_motors, scan_motors, start/stop_teleoperate, start/save_calibration, read_calibration, run_shell.

### LeRobot Fork (`~/so101/lerobot/`)

v0.5.2 with patches for bus signal integrity. See [hardware.md](hardware.md#patched-code--workarounds).

## Inter-Worker Communication

### Shared Files

| File | Writer | Reader | Purpose |
|---|---|---|---|
| `shared/robot_state.json` | UI server | Workers | Live positions, temps, loads, connection status (2 Hz) |
| `shared/kb_context.json` | KB worker | Robot tool | Hardware docs, config context |
| `shared/calibration.json` | Calibration tool | All workers | Shared calibration data |
| `shared/safety.json` | Manual / any worker | UI server, MCP server | Safety thresholds, single source of truth |

### Mailbox (`~/so101/mailbox.json`)

Flat JSON array. Read by `~/.claude/hooks/so101-mailbox-check.sh` on every UserPromptSubmit. Shows messages from last 10 minutes that aren't marked `done`.

Write helper: `~/so101/scripts/mailbox_write.sh <from> <text> [tags]`

Message format:
```json
{
  "id": "hex8",
  "ts": 1778760000.0,
  "from": "claude-session-11",
  "text": "What happened and what's next",
  "tags": ["handoff", "sync"],
  "done": false
}
```

### Directory Mailbox (`~/so101/shared/messages/`)

Structured inter-process messaging (fire-and-forget):

```
shared/messages/
  to_robot/    <- KB worker drops requests
  to_kb/       <- Robot tool sends updates
  from_user/   <- UI notifications the user responded to
```

Envelope format:
```json
{
  "id": "1747612800_scan_request",
  "from": "kb",
  "to": "robot",
  "type": "scan_request",
  "ts": 1747612800.0,
  "payload": {}
}
```

### Session Log (`~/so101/session_log.jsonl`)

Append-only. One JSON object per session:
```json
{
  "session": 10,
  "date": "2026-05-14",
  "learnings": ["..."],
  "artifacts": ["file1.py"],
  "open_questions": ["..."]
}
```

## Service Management

| Service | Port | Start Command |
|---|---|---|
| UI server | 5833 | `~/lerobot-env-312/bin/python ~/so101/ui/app.py` |
| MCP server | stdio | Auto-started by Claude Code via `.mcp.json` |
| Chat server | 8888 | `~/lerobot-env-312/bin/python ~/so101/chat/server.py` |
| Motor server | 7777 | **LEGACY** — conflicts with UI server. Do not run. |

## Hooks

### `~/.claude/hooks/so101-mailbox-check.sh`

Runs on UserPromptSubmit. Reads `mailbox.json`, prints messages from last 10 min that aren't done.

### `~/.claude/hooks/session-sync.sh`

Runs on SessionStart. Injects session_log, robot_state, mailbox, and service status.
