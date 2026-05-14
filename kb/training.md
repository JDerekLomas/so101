# Training Recipes & Lessons

Community-sourced lessons from real SO-101/SO-100 training runs.

## ACT Training — Sherry Chen (3 attempts, RTX 3080)

| Attempt | Demos | In-Distribution | Out-of-Distribution | Key Issue |
|---|---|---|---|---|
| 1 | 50 | ~0% | — | Camera POV changed between train/eval; overfit to fixed locations |
| 2 | 72 (50 train + eval) | 60% | 10% | Fixed cameras, spatial bins — limited generalization |
| 3 | ~150 (25/bin x 6 bins + yaw variation) | 90% | 75% | Systematic spatial coverage with orientation variation |

### Key ACT Lessons

1. **Consistent camera placement is critical** — mark positions with tape/rulers; any shift degrades performance
2. **Spatial diversity beats quantity** — cover full workspace and vary object orientation; 6 bins x 25 demos > 150 random demos
3. **Always maintain an eval set** — needed to detect overfitting early
4. **Gripper friction matters** — add tape to gripper tips to prevent slipping
5. **Camera USB randomization** — use udev rules to lock camera identities across reboots
6. **Do not over-grasp** — motor wear accumulates; consider velocity limits

### ACT Quick Recipe

- **Demos needed**: 50 minimum, 150 for good generalization
- **Model size**: 52M parameters
- **Train time**: ~4h on RTX 3080 (12GB)
- **Training config**: use LeRobot defaults

## SmolVLA Fine-Tuning — ggando.com (RTX 3090)

Setup: 75 episodes in constrained 10cm workspace.

| Model | Cameras | Success Rate | Conditions |
|---|---|---|---|
| SmolVLA | Dual (wrist + overhead) | 100% (5/5) | Night |
| SmolVLA | Dual (wrist + overhead) | 60-80% | Afternoon |
| ACT | Dual (wrist + overhead) | 80% (4/5) | — |

### Key SmolVLA Lessons

- "One clean strategy beats a mix of tricks" — consistency matters more than volume
- Teleoperation lag degrades demo quality — record slower and more deliberately
- Dual camera (wrist + overhead) significantly improves trajectory smoothness
- **Lighting affects performance** — train in conditions you'll deploy in

### SmolVLA Quick Recipe

- **Demos needed**: 50-75 (pretrained model helps)
- **Fine-tune**: 20k steps, batch 64, lr 1e-4 cosine schedule
- **Hardware**: 1x A100 or equivalent (~24GB VRAM)
- **Cameras**: dual (wrist + overhead) recommended
- **Base model**: `lerobot/smolvla_base`

## GR00T N1.5 Notes

- Default motion is jerky — increase denoising steps to 16, action horizon to 16
- Minimum training: 5k-6k steps; overtraining makes model ignore language instructions
- SO-101 NOT in pretraining — always tag datasets as `new_embodiment`
- 3B parameters, needs 25GB+ VRAM

## Dataset Collection Best Practices

### Camera Setup
- Mark camera positions physically (tape on desk)
- Lock USB camera identities with udev rules
- Use dual cameras: overhead for spatial context, wrist for precision
- 480x640 resolution, 30 FPS
- Train and deploy under same lighting conditions

### Demo Quality
- Move slowly and deliberately — lag in teleop = noise in demos
- Cover full workspace systematically (spatial bins)
- Vary object starting position AND orientation
- 50 demos minimum, 150 for robust policies
- Each dataset: one task, consistent strategy

### Recording Pipeline

```bash
# LeRobot native recording
lerobot-record \
  --robot.type=so101_follower \
  --robot.port=/dev/tty.usbmodem5B141123331 \
  --robot.id=my_follower \
  --teleop.type=so101_leader \
  --teleop.port=/dev/tty.usbmodem5B141116761 \
  --teleop.id=my_leader \
  --dataset.repo_id=<your-hf-id>/<dataset-name> \
  --dataset.num_episodes=50
```

### Dataset Format

LeRobot v2.1 standardized format:
- **Parquet**: motor positions, actions, timestamps
- **MP4**: camera frames
- Hosted on HuggingFace Hub
- Compatible with all LeRobot policy architectures
