#!/usr/bin/env python3
"""
SO-101 Robot Assistant — start everything with one command.

Usage:
    python ~/so101/start.py
    so101              (if alias is set — run once: echo 'alias so101="python ~/so101/start.py"' >> ~/.zshrc)
"""
import os
import signal
import socket
import subprocess
import sys
import time
import webbrowser
from pathlib import Path

PYTHON  = "/Users/dereklomas/lerobot-env-312/bin/python3.12"
BASE    = Path.home() / "so101"

SERVERS = [
    {"name": "Motor Server", "port": 7777, "script": BASE / "motor_server.py"},
    {"name": "Web UI",       "port": 5833, "script": BASE / "ui" / "app.py"},
    {"name": "Chat",         "port": 8888, "script": BASE / "chat" / "server.py"},
]

started_procs = []


def port_open(port: int) -> bool:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(0.5)
        return s.connect_ex(("127.0.0.1", port)) == 0


def wait_for_port(port: int, timeout: float = 12.0) -> bool:
    deadline = time.time() + timeout
    while time.time() < deadline:
        if port_open(port):
            return True
        time.sleep(0.3)
    return False


def check_api_key():
    key = os.environ.get("ANTHROPIC_API_KEY", "")
    if not key:
        print("\n  ⚠  ANTHROPIC_API_KEY is not set.")
        print("     The chat assistant needs it to talk to Claude.")
        print()
        print("     Add it to your shell config:")
        print("       echo 'export ANTHROPIC_API_KEY=sk-ant-...' >> ~/.zshrc")
        print("       source ~/.zshrc")
        print()
        print("     Or set it just for this session:")
        print("       export ANTHROPIC_API_KEY=sk-ant-... && python ~/so101/start.py")
        print()
        ans = input("     Start anyway without the chat assistant? [y/N]: ").strip().lower()
        if ans != "y":
            sys.exit(1)
        return False
    return True


def start_server(info: dict) -> subprocess.Popen | None:
    port   = info["port"]
    name   = info["name"]
    script = info["script"]

    if port_open(port):
        print(f"  ✓  {name} already running on :{port}")
        return None

    print(f"  →  Starting {name}...", end=" ", flush=True)
    proc = subprocess.Popen(
        [PYTHON, str(script)],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    if wait_for_port(port):
        print(f"ready  :{port}")
        return proc
    else:
        print(f"FAILED (port {port} not responding)")
        proc.terminate()
        return None


def shutdown(sig, frame):
    print("\n\nShutting down...")
    for proc in started_procs:
        try:
            proc.terminate()
        except Exception:
            pass
    sys.exit(0)


def main():
    print()
    print("  SO-101 Robot Assistant")
    print("  " + "─" * 36)
    print()

    has_key = check_api_key()

    servers_to_start = SERVERS if has_key else SERVERS[:2]

    for info in servers_to_start:
        proc = start_server(info)
        if proc:
            started_procs.append(proc)

    print()

    chat_up = port_open(8888)
    ui_up   = port_open(5833)
    url     = "http://localhost:8888" if chat_up else "http://localhost:5833"

    print(f"  Chat assistant:  {'http://localhost:8888' if chat_up else '(not started)'}")
    print(f"  Motor UI:        {'http://localhost:5833' if ui_up else '(not started)'}")
    print(f"  Motor API:       {'http://localhost:7777' if port_open(7777) else '(not started)'}")
    print()

    if not (os.environ.get("NO_BROWSER") or "--no-browser" in sys.argv):
        print(f"  Opening {url} ...")
        time.sleep(0.5)
        webbrowser.open(url)

    print()
    print("  Press Ctrl+C to stop all servers.")
    print()

    # Add alias hint once
    zshrc = Path.home() / ".zshrc"
    if zshrc.exists() and "alias so101=" not in zshrc.read_text():
        print('  Tip: add this alias so you can just type "so101":')
        print("    echo \'alias so101=\"python ~/so101/start.py\"\' >> ~/.zshrc")
        print()

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    # Keep alive — monitor procs
    while True:
        for proc in started_procs:
            if proc.poll() is not None:
                print(f"  ⚠  A server exited unexpectedly (PID {proc.pid})")
                started_procs.remove(proc)
                break
        time.sleep(2)


if __name__ == "__main__":
    main()
