# SO-101 Knowledge Base

Structured reference for the SO-101 robot arm project. Each file is self-contained for the concern it covers.

## Quick Navigation

| File | Covers | When to read |
|------|--------|-------------|
| [hardware.md](hardware.md) | Motors, registers, ports, calibration, serial protocol | Setting up, debugging hardware, writing to servos |
| [safety.md](safety.md) | Safety principles (P1-P7), thresholds, incident log | Before any motor actuation, after any incident |
| [control.md](control.md) | Teleop loop, cybernetic control, speed formula, equalization | Working on teleoperation or recording |
| [troubleshooting.md](troubleshooting.md) | Footguns, motor health, diagnostics, failure modes | When something goes wrong |
| [ecosystem.md](ecosystem.md) | Models, datasets, community projects, SmolVLA/ACT/GR00T/pi0 | Planning training, choosing a policy |
| [training.md](training.md) | Community training lessons, recipes, dataset collection tips | Before collecting demos or training a policy |
| [architecture.md](architecture.md) | System architecture, inter-worker state, mailbox, MCP, UI server | Understanding the software stack |
| [scripts.md](scripts.md) | Diagnostic scripts, raw control examples, quick-start snippets | Debugging or writing new motor scripts |
| [prior-art.md](prior-art.md) | Related work, paper framing, what's been done before | Paper writing, positioning our contribution |
| [links.md](links.md) | URLs to docs, tutorials, community resources | Quick reference |

## Project Status

- **Hardware**: Dual-arm SO-101 (leader + follower), both calibrated, both online
- **Software**: UI server (:5833), MCP server (Claude Code tools), chat server (:8888)
- **Control**: Cybernetic teleop with proportional speed + velocity feedforward
- **Safety**: 7-principle framework in shared/safety.json
- **Next milestone**: First dataset recording episode

## File Locations

| What | Where |
|------|-------|
| This KB | `~/so101/kb/` |
| Legacy monolithic KB | `~/so101/so101_knowledge_base.md` |
| Session log | `~/so101/session_log.jsonl` |
| Mailbox | `~/so101/mailbox.json` |
| Shared state | `~/so101/shared/` |
| Safety config | `~/so101/shared/safety.json` |
| Paper | `~/so101/paper/` |
| Scripts | `~/so101/scripts/` |
| UI server | `~/so101/ui/app.py` |
| MCP server | `~/so101/mcp_server.py` |
| LeRobot fork | `~/so101/lerobot/` |
| Python env | `~/lerobot-env-312/` |
