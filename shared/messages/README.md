# Mailbox

Two directories for inter-worker messages.

## Protocol

- Drop a JSON file in `to_robot/` or `to_kb/`
- Filename: `<timestamp>_<type>.json`  e.g. `1747612800_scan_request.json`
- Reader processes and deletes (or moves to an `archive/` subdir if you want history)
- Writer does not wait — fire and forget unless the message type implies a response

## Message envelope

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

## Message types

### to_robot/
| type | payload | description |
|------|---------|-------------|
| `scan_request` | `{}` | Ask robot tool to rescan ports and update robot_state.json |
| `calibrate_start` | `{}` | Start range recording |
| `calibrate_stop` | `{}` | Stop range recording and save |

### to_kb/
| type | payload | description |
|------|---------|-------------|
| `state_update` | snapshot of robot_state.json | Pushed on significant events |
| `calibration_saved` | `{ "path": "...", "ranges": {...} }` | Calibration was written to disk |
| `motor_error` | `{ "arm": "follower", "motor": "wrist_roll", "detail": "..." }` | Motor fault detected |
| `port_assigned` | `{ "follower": "/dev/...", "leader": "/dev/..." }` | Ports identified and saved |
