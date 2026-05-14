#!/bin/bash
# Injected into Claude's context at session start.
# Reads session log, shared state, and checks running services.

DIR="$(cd "$(dirname "$0")/../.." && pwd)"

echo "=== SESSION LOG (accumulated learnings) ==="
cat "$DIR/session_log.jsonl" 2>/dev/null || echo "No session log yet."
echo ""

echo "=== ROBOT STATE ==="
cat "$DIR/shared/robot_state.json" 2>/dev/null || echo "No robot state file."
echo ""

echo "=== MAILBOX (pending handoff notes) ==="
cat "$DIR/mailbox.json" 2>/dev/null || echo "No mailbox."
echo ""

echo "=== SERVICES ==="
curl -s --connect-timeout 1 http://localhost:7777/state > /dev/null 2>&1 && echo "Motor server (:7777): RUNNING" || echo "Motor server (:7777): DOWN"
curl -s --connect-timeout 1 http://localhost:5833 > /dev/null 2>&1 && echo "Web UI (:5833): RUNNING" || echo "Web UI (:5833): DOWN"
curl -s --connect-timeout 1 http://localhost:8888 > /dev/null 2>&1 && echo "Chat server (:8888): RUNNING" || echo "Chat server (:8888): DOWN"
echo ""

echo "=== INSTRUCTIONS ==="
echo "You are working on the SO-101 robot arm project. Read the session log above for accumulated learnings."
echo "Before ending a session, append a new line to session_log.jsonl with your learnings, artifacts, and open questions."
echo "Knowledge base: ~/so101/so101_knowledge_base.md"
echo "Paper outline: ~/so101/paper/outline.md"

exit 0
