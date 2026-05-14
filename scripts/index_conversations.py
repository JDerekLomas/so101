#!/usr/bin/env python3
"""Index Claude Code conversation transcripts for search.

Reads raw JSONL transcripts from ~/.claude/projects/-Users-dereklomas-so101/
and builds a searchable index at shared/conversation_history/index.jsonl.

Each index entry has: session_id, timestamp, role, text (user prompts and
assistant text responses only — no tool calls or thinking).

Run: python scripts/index_conversations.py
"""

import json
import re
import sys
from pathlib import Path

TRANSCRIPT_DIR = Path.home() / ".claude/projects/-Users-dereklomas-so101"
INDEX_DIR = Path(__file__).parent.parent / "shared" / "conversation_history"


def extract_text(content):
    """Extract readable text from message content."""
    if isinstance(content, str):
        return content
    if isinstance(content, list):
        texts = []
        for block in content:
            if isinstance(block, dict):
                if block.get("type") == "text":
                    texts.append(block.get("text", ""))
                elif block.get("type") == "tool_result":
                    # Skip tool results
                    pass
            elif isinstance(block, str):
                texts.append(block)
        return "\n".join(texts)
    return ""


def index_transcript(jsonl_path):
    """Extract user and assistant messages from a transcript."""
    entries = []
    session_id = jsonl_path.stem

    with open(jsonl_path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                record = json.loads(line)
            except json.JSONDecodeError:
                continue

            msg_type = record.get("type")
            if msg_type not in ("user", "assistant"):
                continue

            message = record.get("message", {})
            role = message.get("role", msg_type)
            content = message.get("content", "")
            text = extract_text(content)

            # Skip empty text, tool calls, and thinking blocks
            if not text.strip():
                continue

            # Skip system-reminder-only content
            if text.strip().startswith("<system-reminder>") and text.strip().endswith("</system-reminder>"):
                continue

            entries.append({
                "session_id": session_id,
                "ts": record.get("timestamp", ""),
                "role": role,
                "text": text[:2000],  # cap at 2000 chars per entry
            })

    return entries


def main():
    INDEX_DIR.mkdir(parents=True, exist_ok=True)

    # Find all session transcripts (top-level only, not subagents)
    transcripts = sorted(TRANSCRIPT_DIR.glob("*.jsonl"))
    if not transcripts:
        print("No transcripts found.")
        return

    all_entries = []
    for t in transcripts:
        print(f"Indexing {t.name}...")
        entries = index_transcript(t)
        all_entries.extend(entries)
        print(f"  -> {len(entries)} messages")

    # Write index
    index_path = INDEX_DIR / "index.jsonl"
    with open(index_path, "w") as f:
        for entry in all_entries:
            f.write(json.dumps(entry) + "\n")

    print(f"\nIndexed {len(all_entries)} messages from {len(transcripts)} sessions")
    print(f"Index: {index_path}")

    # Also write a summary
    summary = {
        "indexed_at": __import__("time").strftime("%Y-%m-%dT%H:%M:%S"),
        "sessions": len(transcripts),
        "total_messages": len(all_entries),
        "user_messages": sum(1 for e in all_entries if e["role"] == "user"),
        "assistant_messages": sum(1 for e in all_entries if e["role"] == "assistant"),
        "session_ids": [t.stem for t in transcripts],
    }
    (INDEX_DIR / "summary.json").write_text(json.dumps(summary, indent=2))
    print(f"Summary: {INDEX_DIR / 'summary.json'}")


if __name__ == "__main__":
    main()
