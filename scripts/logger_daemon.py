#!/usr/bin/env python3
"""
Continuous motor data logger daemon.
Polls /api/state at ~10 Hz and appends to shared/motor_data.jsonl.
Runs independently of app.py — survives app restarts.

Usage:
    python3 scripts/logger_daemon.py &
    tail -f shared/motor_data.jsonl | python3 -c "
        import sys,json
        for line in sys.stdin:
            r=json.loads(line)
            f=r['motors'].get('follower',{})
            print({k: v['temp'] for k,v in f.items()})
    "
"""

import json
import time
import urllib.request
from pathlib import Path

API = "http://localhost:5833/api/state"
LOG = Path.home() / "so101/shared/motor_data.jsonl"
INTERVAL = 0.1   # 10 Hz
RECONNECT_WAIT = 2.0

ALERT_THRESHOLDS = {
    "load": 400,
    "temp": 55,
}


def fetch_state():
    with urllib.request.urlopen(API, timeout=2) as r:
        return json.loads(r.read())


def check_alerts(record):
    alerts = []
    for role, motors in record.get("motors", {}).items():
        for name, v in motors.items():
            if v.get("load") and v["load"] > ALERT_THRESHOLDS["load"]:
                alerts.append(f"{role}/{name} HIGH LOAD={v['load']}")
            if v.get("temp") and v["temp"] > ALERT_THRESHOLDS["temp"]:
                alerts.append(f"{role}/{name} HOT temp={v['temp']}°C")
    return alerts


def main():
    LOG.parent.mkdir(parents=True, exist_ok=True)
    print(f"[logger] Writing to {LOG}")
    print(f"[logger] Polling {API} at {int(1/INTERVAL)} Hz")

    consecutive_errors = 0

    with open(LOG, "a", buffering=1) as f:
        while True:
            try:
                state = fetch_state()
                now = time.time()

                motors = {}
                for role in ("leader", "follower"):
                    m = state.get("motors", {}).get(role, {})
                    if m:
                        motors[role] = {
                            name: {
                                "pos":  v.get("position"),
                                "load": v.get("load"),
                                "temp": v.get("temperature"),
                            }
                            for name, v in m.items()
                        }

                record = {
                    "ts":     round(now, 3),
                    "teleop": state.get("teleop_active", False),
                    "eq":     state.get("teleop_equalizing", False),
                    "motors": motors,
                }

                # Include any new events
                events = state.get("events", [])
                if events:
                    record["last_event"] = events[-1].get("kind")

                f.write(json.dumps(record) + "\n")

                alerts = check_alerts(record)
                for a in alerts:
                    print(f"[logger] ALERT: {a}")

                consecutive_errors = 0

            except Exception as e:
                consecutive_errors += 1
                if consecutive_errors % 10 == 1:
                    print(f"[logger] Connection error ({consecutive_errors}): {e}")
                time.sleep(RECONNECT_WAIT)
                continue

            time.sleep(INTERVAL)


if __name__ == "__main__":
    main()
