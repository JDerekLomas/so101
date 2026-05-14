# Testing Plan: SO-101 + Claude Code Integration

## Test Results (Session 7, 2026-05-14)

### Passed
- [x] UI server (:5833) responding
- [x] `/api/state` returns valid JSON with connection info
- [x] `/api/events` returns 17 events (moves, calibration)
- [x] MCP SDK imports correctly in lerobot-env-312
- [x] MCP->UI HTTP path works (urllib from Python)
- [x] Both boards detected (2 USB ports, 5+6 motors)
- [x] Session sync hook runs and outputs all context files
- [x] Git repo initialized, initial commit successful
- [x] `nudge_gripper.py` moves motor 6 (tested earlier this session)
- [x] Notification POST to UI works

### Not tested (need next session with MCP active)
- [ ] MCP server starts via `.mcp.json` on Claude Code launch
- [ ] `mcp__so101__get_state` tool appears in Claude's tool list
- [ ] `mcp__so101__move_joint` actually moves a motor
- [ ] `mcp__so101__nudge_joint` reads current pos and offsets correctly
- [ ] `mcp__so101__start_calibration` / `save_calibration` round-trip
- [ ] `mcp__so101__notify_user` shows notification in web UI
- [ ] SessionStart hook injects context into new session automatically

### Not tested (need hardware action)
- [ ] Calibration full round-trip: start -> move arms -> save -> verify JSON
- [ ] Teleoperation via lerobot-teleoperate (both arms must be assigned)
- [ ] Leader arm motor response (currently 5/6 responding)
- [ ] Safety: stall detection trips and disables torque
- [ ] Safety: temperature cutoff at 65C

---

## Gaps Between Paper Claims and Implementation

### Gap 1: Chat server cannot move motors
**Paper says**: "LLM writes goal positions"
**Reality**: chat/server.py has 8 tools, none write to motors.
**Fix**: Either (a) add move tools to chat server, or (b) deprecate chat server in favor of Claude Code + MCP. Option (b) is cleaner — the MCP server already has all the tools.
**Recommendation**: Option (b). The paper's story is about Claude Code, not a custom chat UI.

### Gap 2: Motors not assigned to roles
**Current state**: Both boards connected but `motors.follower` and `motors.leader` are empty `{}`.
**Why**: No one has called `/api/assign` to label which port is which.
**Fix**: Need wiggle-test or manual assignment via UI before move tools work.
**Test**: After assigning, verify `/api/move` routes to correct arm.

### Gap 3: No calibration data exists
**Current state**: `calibration.last_saved: null`, JSON files missing.
**Impact**: `move_to_middle` will fail (needs calibration). Position clamping has no limits to clamp to.
**Test**: Full calibration round-trip needed.

### Gap 4: Session log not auto-populated
**Paper says**: "Append-only lab notebook"
**Reality**: Claude must manually append to `session_log.jsonl`. No enforcement.
**Fix**: Could add a SessionEnd hook that prompts Claude to write learnings. Or accept it as a manual discipline and document that in the paper.

### Gap 5: Three servers vs. one
**Current state**: motor_server.py (:7777), ui/app.py (:5833), chat/server.py (:8888)
**Problem**: motor_server.py and ui/app.py both try to own the serial port. They can't run simultaneously.
**Reality**: Only ui/app.py runs in practice; motor_server.py is legacy.
**Fix**: Remove motor_server.py from start.py, or document it as deprecated.
**Impact on MCP**: MCP server talks to :5833 (correct). Chat server talks to :7777 (may be down).

---

## Testing Protocol for Next Session

### Phase 1: MCP Smoke Test (first thing)
1. Start new Claude Code session in ~/so101/
2. Verify hook fires ("Syncing session context..." spinner)
3. Verify MCP server connects (check for `mcp__so101__*` tools)
4. Call `mcp__so101__get_state` — should return connection info
5. Call `mcp__so101__get_events` — should return event history

### Phase 2: Arm Assignment
1. Call `mcp__so101__rescan`
2. Identify which port has 6 motors (follower) vs 5 (leader, missing one)
3. Call `mcp__so101__assign_arm` for both ports
4. Verify `get_state` now shows motors under follower/leader

### Phase 3: Motor Control
1. Call `mcp__so101__get_positions("follower")` — verify positions
2. Call `mcp__so101__nudge_joint("gripper", 50, "follower")` — small open
3. Confirm physical movement
4. Call `mcp__so101__nudge_joint("gripper", -50, "follower")` — close back
5. Try `move_joint("shoulder_pan", 2048, "follower")` — move to center

### Phase 4: Calibration
1. Call `mcp__so101__start_calibration`
2. Physically move all joints through full range (human action)
3. Call `mcp__so101__save_calibration`
4. Verify JSON files created at expected paths
5. Call `mcp__so101__move_to_middle` — all joints go to midpoint

### Phase 5: Paper Validation
1. Record the tool call sequence from phases 1-4
2. Note every human physical action required
3. Note every failure and recovery
4. This becomes Section 4 of the paper ("Development Narrative")

---

## Architecture Decision: Chat Server vs. MCP

The chat server (chat/server.py on :8888) and the MCP server (mcp_server.py) overlap significantly. For the paper, the cleanest story is:

**Claude Code + MCP = the primary interface**
- Native tool access (no HTTP wrapper in the loop)
- Session hooks for context persistence
- Git-tracked artifacts
- Terminal-based, reproducible

**Chat UI = secondary/demo interface**
- Good for showing non-technical users
- Good for screenshots in the blog post
- But not the core contribution

**Web UI (:5833) = hardware layer**
- Always running
- Owns the serial port
- Serves both MCP and chat server
- This is the "robot operating system"

This three-layer architecture (Claude Code -> MCP -> UI -> Hardware) is the paper's contribution. Document it as such.
