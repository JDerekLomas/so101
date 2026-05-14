# Testing Plan: SO-101 + Claude Code Integration

## Test Results (Sessions 7-10, 2026-05-14)

### Phase 1: MCP Smoke Test — PASS (Session 10)
- [x] MCP server starts via `.mcp.json` on Claude Code launch
- [x] `mcp__so101__get_state` tool appears in Claude's tool list
- [x] `mcp__so101__get_state` returns full state (connection, motors, calibration, teleop_active)
- [x] `mcp__so101__get_events` returns event history
- [x] SessionStart hook injects session_log, robot_state, mailbox, service status
- [x] UI server (:5833) responding
- [x] Both boards detected (2 USB ports, 6+6 motors after rescan)

### Phase 2: Arm Assignment — PASS (Session 10)
- [x] `mcp__so101__rescan` finds both boards
- [x] `mcp__so101__assign_arm` assigns both ports to follower/leader
- [x] `mcp__so101__get_positions` returns data for both arms
- [x] Temperature pattern confirms correct assignment (follower: 27-33C real, leader: 0C passive)

### Phase 3: Motor Control — PASS with findings (Session 10)
- [x] `mcp__so101__nudge_joint("gripper", 50, "follower")` — physical movement confirmed
- [x] `mcp__so101__nudge_joint("gripper", -50, "follower")` — returned to original
- [x] `mcp__so101__move_joint("shoulder_pan", 2048, "follower")` — motor responded
- [x] `mcp__so101__move_to_middle` — 5/6 joints moved, elbow_flex write_error
- [x] Ramped move via `/api/move` with `ramped: true` — smooth motion confirmed
- [x] Multi-joint ramped move (all 6 joints to midpoints) — smooth

#### Findings
- **Teleop blocks external moves**: moves return "ok" but teleop overwrites at 20Hz. Fixed: added `start_teleop`/`stop_teleop` MCP tools, pre-flight check in MCP refuses moves during teleop.
- **Elbow_flex stuck at desk**: position 102 is into the desk surface. Stall detection killed torque before motor could escape. Fixed: 2s grace period on stall detection after commanded moves.
- **Elbow_flex calibration too wide**: range_min was 24, allowing desk collision. Fixed: raised to 300.
- **MCP server hot-reload**: MCP tools loaded at Claude Code startup; changes require restart. The old (pre-safety) MCP server ran during this session. New MCP server with pre-flight checks will activate next session.

### Phase 4: Calibration — PASS (pre-existing, verified Session 10)
- [x] Follower calibration file exists with proper ranges (all 6 joints)
- [x] Leader calibration file exists with proper ranges (all 6 joints)
- [x] No joints left at 0-4095 (the uncalibrated default)
- [x] elbow_flex range_min tightened 24→300 to prevent desk collision
- [ ] Full round-trip via MCP (start_calibration → move → save_calibration) — not tested this session

### Phase 5: Paper Validation — IN PROGRESS
Tool call sequence from this session documents the development narrative:
1. Hook fires → context injected (session log, robot state, mailbox)
2. MCP tools appear → get_state works
3. Rescan → both boards found, 6 motors each
4. Assign arms → follower/leader labeled
5. Nudge gripper → **no movement** (teleop was active, overriding commands)
6. Discovery: teleop_active not visible in state, no stop_teleop tool
7. Fix: add teleop tools to MCP, add teleop_active to state JSON
8. Stop teleop → nudge works → movement confirmed
9. Move_to_middle → elbow_flex stuck at desk
10. Discovery: stall detection prevents recovery from stalled position
11. Fix: 2s grace period, raise elbow_flex range_min
12. Ramped moves working → smooth, safe motion

**Human physical actions required**: zero (all via Claude Code + MCP tools)
**Failures encountered**: 2 (teleop override, stall-prevents-recovery)
**Recoveries**: 2 (both fixed programmatically in same session)

---

## Safety Framework (Session 10)

### Principles (shared/safety.json)
| ID | Principle | Implementation |
|----|-----------|----------------|
| P1 | No teleport | Ramped moves (60 counts/cycle max), nudges capped ±200 |
| P2 | Know before move | MCP pre-flight: check teleop, temps, arm assignment |
| P3 | Margin of safety | 5% inside calibrated range, desk collision guard |
| P4 | Torque is temporary | Auto-disable after ramp completes + settle time |
| P5 | Detect and halt | Stall (800 load, 6 cycles), temp (65C), with 2s move grace |
| P6 | Equalize before track | Teleop startup alignment to leader position |
| P7 | Human in the loop | Large-move warnings (>500 counts) in MCP response |

### Single Source of Truth
All thresholds in `shared/safety.json`. Both `ui/app.py` and `mcp_server.py` import from it.

---

## Gaps Resolved

### Gap 1: Chat server cannot move motors — RESOLVED
MCP server is the primary interface. Chat server updated to talk to :5833 (session 9).

### Gap 2: Motors not assigned to roles — RESOLVED
`assign_arm` tool works. Port-swap-on-replug documented as known behavior.

### Gap 3: No calibration data — RESOLVED
Both arms calibrated. Files at expected HuggingFace cache paths.

### Gap 4: Session log not auto-populated — PARTIALLY RESOLVED
Backfilled sessions 1-9. Hook instructions strengthened with mandatory checklist. No SessionEnd hook (not supported by Claude Code hooks API).

### Gap 5: Three servers — RESOLVED
Only ui/app.py (:5833) runs. motor_server.py is legacy. MCP and chat server both talk to :5833.

### New Gap: MCP hot-reload
MCP server process loaded at Claude Code startup. Code changes to mcp_server.py require Claude Code restart to take effect. The safety pre-flight checks added this session won't be active until next restart.

---

## Architecture (confirmed)

```
Claude Code ─── MCP server (mcp_server.py, stdio) ─── UI server (:5833) ─── Serial ─── Robot
                 │ pre-flight checks                    │ ramped moves
                 │ large-move warnings                  │ stall detection
                 │ teleop guard                         │ temp cutoff
                 │ nudge cap ±200                       │ position clamping
                                                        │ move grace period
                                                        │ teleop mirror (20Hz)
```

Safety is layered: MCP catches intent-level issues (wrong mode, dangerous targets), UI catches hardware-level issues (stall, temp, limits).
