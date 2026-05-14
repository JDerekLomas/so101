#!/bin/bash
# Write a message to the so101 mailbox.
# Usage: mailbox_write.sh <from> <text> [tag1,tag2,...]
# Example: mailbox_write.sh "claude-session-11" "Restructuring KB" "kb,restructure"

FROM="${1:?Usage: mailbox_write.sh <from> <text> [tags]}"
TEXT="${2:?Usage: mailbox_write.sh <from> <text> [tags]}"
TAGS="${3:-sync}"

MAILBOX="$HOME/so101/mailbox.json"

python3 - "$FROM" "$TEXT" "$TAGS" "$MAILBOX" <<'PYEOF'
import json, time, sys, os, uuid

from_id = sys.argv[1]
text = sys.argv[2]
tags = [t.strip() for t in sys.argv[3].split(",")]
mailbox_path = sys.argv[4]

msg = {
    "id": uuid.uuid4().hex[:8],
    "ts": time.time(),
    "from": from_id,
    "text": text,
    "tags": tags,
    "done": False
}

try:
    msgs = json.loads(open(mailbox_path).read())
except Exception:
    msgs = []

msgs.append(msg)

with open(mailbox_path, "w") as f:
    json.dump(msgs, f, indent=2)

print(f"Mailbox: wrote message {msg['id']} from {from_id}")
PYEOF
