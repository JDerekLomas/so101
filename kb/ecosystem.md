# Models, Datasets & Community Ecosystem

## Model Recommendation Ladder

| Tier | Model | Params | VRAM | Train Time | Success Rate | Notes |
|---|---|---|---|---|---|---|
| 1 (start here) | ACT | 52M | Low | ~4h (RTX 3080 12GB) | 60-90% with 50-150 demos | Fast iteration, low resource |
| 2 | SmolVLA | 450M | ~24GB | ~10h (RTX 3090) | 78%+ pick-place | Better generalization, language conditioning |
| 3 | GR00T N1.5 | 3B | 25GB+ | — | — | Cross-embodiment, language-steered. SO-101 NOT in pretraining |
| 4 | pi0 / pi0-fast | 3B+ | High | — | — | Community fine-tuned on SO-101. Base: `lerobot/pi0_base` |

## SmolVLA Details

| Property | Value |
|---|---|
| Parameters | 450M (350M backbone + 100M action expert) |
| Backbone | SmolVLM2-500M-Video-Instruct |
| Pretraining | 487 community datasets / ~10M frames / ~30k episodes from SO-100 community |
| Performance | 78.3% pick-place (vs 51.7% without pretraining) |
| Fine-tune | 20k steps, batch 64, lr 1e-4 cosine, ~10h on 1x A100 |
| Async inference | 30% faster, 2x task throughput |
| Community fine-tunes | 5,478 variants on HF Hub |
| Base model | `lerobot/smolvla_base` |

Key techniques: reduced visual tokens, skip upper VLM layers, interleaved cross/self-attention in action expert.

## Official LeRobot Datasets

| Dataset | Episodes | Frames | Task |
|---|---|---|---|
| `lerobot/svla_so101_pickplace` | 50 | — | Pick object, place in container |
| `lerobot/svla_so100_pickplace` | 50 | 19,631 | Pick and place |
| `lerobot/svla_so100_stacking` | 56 | 22,956 | Block stacking |
| `lerobot/svla_so100_sorting` | 52 | 35,713 | Object sorting |

All: 30 FPS, top + wrist cameras, 480x640.

## Community Scale

| Metric | Value |
|---|---|
| SO-100 + SO-101 datasets on HF Hub | 8,500+ |
| Median episodes per dataset | 10 |
| 43% of datasets | 1-5 episodes |
| Pretrained SO-101 policies on Hub | 72+ |

## Notable Community Models

| Model | Description |
|---|---|
| `felixmayor/pi05_so101_orange_cube` | Pi0.5, 154 demos / 68.5k frames |
| `mason-mcgough/act-lerobot-so101-lego-cube-50-default` | ACT, lego cube, 50 demos |
| `kogeek/lerobot_so101_gr00t_n1.6` | GR00T 3B fine-tune |
| `youliangtan/so101-table-cleanup` | Table cleanup, used in NVIDIA tutorial |

## Alternative Control Software

| Project | Description |
|---|---|
| phosphobot | Browser UI, one-click recording/training, ACT/SmolVLA/pi0.5/GR00T N1.5 |
| kedikala/soarm101 | Simpler middleware alternative to lerobot |
| viam-devrel/so-101 | Viam platform, web calibration, remote teleop |
| msf4-0/so101_ros2 | Pure ROS2, no lerobot dependency |
| MuammerBay/isaac_so_arm101 | Isaac Lab sim-to-real pipeline |

## MCP Servers (LLM-Native Control)

| Project | Description |
|---|---|
| IliaLarchenko/robot_MCP | MCP for Claude/GPT/Gemini, lerobot normalized states |
| phospho-app/phospho-mcp-server | phosphobot MCP bridge |
| titansage02/so101-mcp | SO-101 specific MCP |

## Simulation

| Platform | Use Case |
|---|---|
| Isaac Sim | Real-to-sim position mirroring (digital twin) |
| Isaac Lab RL | Sim-to-real policy transfer via RL |

## SO-100 vs SO-101

| Property | SO-100 | SO-101 |
|---|---|---|
| Follower torque | 19.5 kg.cm (7.4V) | 30 kg.cm (12V) |
| Assembly | Requires gear removal | No gear removal needed |
| Wiring | Disconnection issues at joint 3 | Improved wiring |
| Leader follows follower | No | Yes (for RL intervention) |
| Community data | Majority of 8,500+ datasets | Growing |
