# Conversational Robot Bringup: An LLM Agent in the SO-101 Development Loop

## Target venues
- arXiv preprint (cs.RO, cs.HC, or cs.AI)
- Companion blog post (HF blog, personal site, or Anthropic dev blog)

---

## Abstract sketch

We document the integration of Claude Code — an LLM-based CLI agent — into the full development lifecycle of an SO-101 robot arm: from hardware bringup and motor debugging through calibration, teleoperation infrastructure, and iterative control. Unlike prior work that uses LLMs to generate robot code offline, our setup places the LLM in the runtime loop: it reads sensor data, writes and executes motor commands, builds persistent knowledge across sessions, and negotiates physical tasks with the human operator through natural language. We describe the architecture, the emergent protocols that developed over multiple sessions, and analyze what this interaction pattern reveals about human-LLM collaboration on physical systems.

---

## 1. Introduction

- The SO-101 as a case study: cheap, open-source, well-documented, large community
- The problem: hardware bringup is still a manual, iterative, under-documented process
- The claim: an LLM agent with tool use (file I/O, shell, serial) can participate as a collaborator in this process, not just a code generator
- Distinction from code generation, MCP robot control, and autonomous agents

## 2. Related Work

- LLMs for robot code generation (Code as Policies, SayCan, VoxPoser, etc.)
- MCP-based robot control (robot_MCP, phospho-mcp, ROS + LLM bridges)
- LLM agents with tool use (Claude Code, Devin, Cursor, OpenHands)
- Human-robot interaction via natural language (but here it's human-LLM-robot)
- Persistent memory for LLM agents (MemGPT, generative agents, etc.)

## 3. System Architecture

- SO-101 hardware: STS3215 servos, serial bus, dual-arm leader/follower
- Claude Code as the development interface: tool use (bash, file read/write, web)
- Motor server (read-only HTTP bridge to serial bus)
- Web UI for human-side monitoring
- Shared state files: robot_state.json, kb_context.json, mailbox.json
- Session persistence: session_log.jsonl, knowledge base markdown

### 3.1 The Tool-Use Stack
- What the LLM can do: read files, execute scripts, curl endpoints, write code
- What it cannot do: physical manipulation, visual inspection, feel resistance
- The serial port as the boundary: LLM writes goal positions, physics does the rest

### 3.2 Persistent Context Architecture
- Knowledge base (curated reference — the "textbook")
- Session log (append-only learnings — the "lab notebook")
- Shared state (live telemetry — the "dashboard")
- Mailbox (inter-session handoff — the "shift change notes")

## 4. Development Narrative

Chronological account of sessions, structured as episodes. Each episode:
- What was attempted
- What the LLM did (tool calls, scripts written, commands run)
- What the human did (physical actions, corrections, decisions)
- What was learned (added to session log)

### Key episodes to document:
- Session 1: Initial bringup, motor scanning, bus debugging
- Session 2: Reorganization, shared state architecture
- Session 3: UI notification system, inter-process communication
- Session 4: First direct motor actuation from LLM ("open the gripper")
- Sessions 5+: Calibration, teleoperation, recording (TBD)

## 5. Analysis

### 5.1 Division of Labor
- Table: tasks performed by LLM vs human vs jointly
- Quantitative: tool calls per session, scripts written, files modified
- What the human did that the LLM could not (and vice versa)

### 5.2 Knowledge Accumulation
- How the knowledge base and session log grew over time
- What information persisted vs. was re-derived
- Comparison to traditional documentation (README, wiki, comments)

### 5.3 Emergent Protocols
- The "nudge and confirm" interaction pattern for motor control
- Safety conventions that developed (read before write, small steps, confirm direction)
- How the mailbox/handoff system evolved from ad-hoc to structured

### 5.4 Failure Modes
- Hallucinated register addresses or protocol details
- Sandbox restrictions blocking hardware access
- Serial port contention (server holds port)
- Context loss across sessions (what the log didn't capture)

## 6. Discussion

- Is this teleoperation of the development process?
- The LLM as junior engineer with perfect memory but no hands
- Safety implications: who authorizes motor writes?
- Comparison to MCP approach: flexibility vs. formalism
- Generalizability beyond SO-101

## 7. Conclusion

- What worked, what didn't, what's next
- Open questions for the community

---

## Appendices (arXiv)
- A: Full tool-use trace for a representative session
- B: Knowledge base snapshot
- C: Session log entries

---

## Blog post structure (shorter, narrative-driven)

1. Hook: "I asked Claude to open my robot's gripper. It did."
2. What is the SO-101? (photo, 2 paragraphs)
3. What is Claude Code? (brief, link to docs)
4. The setup: how they're wired together (architecture diagram)
5. The story: 4-5 key moments from the development process
6. What surprised me (emergent behaviors, failure modes)
7. What's next (calibration, training, autonomous operation?)
8. Links: repo, arXiv paper, knowledge base

---

## Data to collect going forward

To strengthen the paper, capture these during future sessions:
- [ ] Tool call counts per session (available from Claude Code logs)
- [ ] Time-to-resolution for specific tasks (bringup, calibration, etc.)
- [ ] Screenshots/photos of physical setup at each stage
- [ ] Video of the "nudge gripper" interaction
- [ ] Diff of knowledge base over time (git track it)
- [ ] Any cases where the LLM was wrong and how it was corrected
- [ ] User experience notes (your subjective observations)
