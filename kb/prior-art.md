# Prior Art & Paper Framing

## Our Contribution

An LLM agent (Claude Code) participates in the full development lifecycle of a physical robot — not code generation, not task planning, but the messy iterative process of bringup, debugging, calibration, and infrastructure building. The agent has tool access, persistent memory, and negotiates physical tasks with the human through natural language.

## Gap in the Literature

Existing work uses LLMs to:
- **Generate robot code offline** (Code as Policies, VoxPoser) — one-shot, no iteration
- **Plan high-level tasks** (SayCan, Inner Monologue) — abstract reasoning, not hardware debugging
- **Control robots via chat** (robot_MCP) — imperative control, not collaborative development

Nobody has documented the LLM as a **development collaborator** that builds knowledge over sessions, discovers hardware quirks, writes safety systems, and hands off to other workers.

## Related Work Categories

### LLMs for Robot Code Generation
| Paper | Key Idea | Difference from ours |
|---|---|---|
| Code as Policies (Liang et al., 2023) | LLM writes Python for robot manipulation | One-shot code gen, no iterative debugging |
| SayCan (Ahn et al., 2022) | LLM scores affordances for task planning | High-level planning, no hardware access |
| VoxPoser (Huang et al., 2023) | LLM generates 3D value maps for manipulation | Spatial reasoning, not bringup |
| Inner Monologue (Huang et al., 2022) | Feedback loop between LLM and environment | Closest precedent — but task execution, not development |

### LLM Agents with Tool Use
| System | Key Idea | Difference |
|---|---|---|
| Claude Code | CLI agent with bash, file I/O, web | Our platform |
| Devin / SWE-bench | Software engineering agents | Code only, no hardware |
| OpenHands | Multi-modal development agent | Software focused |

### Persistent Memory for LLM Agents
| System | Key Idea | Difference |
|---|---|---|
| MemGPT (Packer et al., 2023) | Virtual context management for long conversations | Memory architecture, not domain-specific |
| Generative Agents (Park et al., 2023) | Simulated agents with memory and reflection | Simulated world, not physical |

### MCP-Based Robot Control
| Project | Key Idea | Difference |
|---|---|---|
| robot_MCP | Direct LLM motor control via MCP | Imperative control, no development narrative |
| phosphobot | Browser UI + recording + training | Closest system to ours, but no LLM in the loop |

## Paper Structure

See `~/so101/paper/outline.md` for full outline.

**Core thesis**: The development process itself is a human-robot interaction problem, and LLM agents are a new participant. Not code generation, not task planning — the whole messy bringup.

**Why SO-101**: Cheap, open-source, 8,500+ community datasets. Thousands going through same bringup pain. If LLM agent reduces that pain and produces reusable artifacts, that's a real contribution.

**Where the paper lives or dies**: Section 5 (Analysis)
- Division of labor table (LLM vs human per session)
- Knowledge accumulation curve (git diff of KB over time)
- Failure taxonomy
- Emergent protocols ("nudge and confirm", read-before-write, mailbox evolution)

## Data Still Needed

- [ ] Tool call counts per session
- [ ] Time-to-resolution for specific tasks
- [ ] Photos/videos at each development stage
- [ ] Git-tracked KB diffs
- [ ] Cases where LLM was wrong and how corrected
- [ ] User experience notes (subjective observations)
