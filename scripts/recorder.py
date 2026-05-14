#!/usr/bin/env python3
"""SO-101 dataset recorder — runs alongside UI server (ui/app.py).

Reads motor positions from http://localhost:5833/api/positions at target FPS.
Does NOT need serial port access — app.py handles that.

Modes:
  --auto    (default) Auto-record during teleop. Teleop start/stop = episode boundaries.
  --manual  Press Enter to start each episode, Ctrl+C or Enter again to stop.

Usage:
  python scripts/recorder.py --task "pick and place" --episodes 10
  python scripts/recorder.py --task "pick and place" --no-camera --episodes 5
  python scripts/recorder.py --task "stack blocks" --mode manual --fps 20
"""

import argparse
import json
import os
import sys
import time
from pathlib import Path

try:
    import numpy as np
    import requests
except ImportError as e:
    print(f"Missing dependency: {e}")
    print("Run: pip install numpy requests")
    sys.exit(1)

JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex",
               "wrist_flex", "wrist_roll", "gripper"]
NUM_JOINTS = len(JOINT_NAMES)

API_BASE = "http://localhost:5833"
POSITIONS_URL = f"{API_BASE}/api/positions"

CAL_PATHS = {
    "follower": Path.home() / ".cache/huggingface/lerobot/calibration/robots/so_follower/my_follower.json",
    "leader":   Path.home() / ".cache/huggingface/lerobot/calibration/teleoperators/so_leader/my_leader.json",
}


# ── Calibration ────────────────────────────────────────────────────────────────

def load_calibration():
    cal = {}
    for role, path in CAL_PATHS.items():
        if path.exists():
            cal[role] = json.loads(path.read_text())
        else:
            print(f"WARNING: No calibration file for {role} at {path}")
            cal[role] = None
    return cal


def normalize(raw_positions, cal_role):
    """Normalize raw encoder counts to [-1, 1] using calibration ranges."""
    normed = []
    for i, name in enumerate(JOINT_NAMES):
        rmin = cal_role[name]["range_min"]
        rmax = cal_role[name]["range_max"]
        span = rmax - rmin
        if span < 1:
            normed.append(0.0)
        else:
            normed.append(2.0 * (raw_positions[i] - rmin) / span - 1.0)
    return np.array(normed, dtype=np.float32)


# ── API ────────────────────────────────────────────────────────────────────────

def get_positions():
    """Fetch current positions from UI server.
    Returns (follower_list, leader_list, teleop_active) or raises."""
    r = requests.get(POSITIONS_URL, timeout=1)
    r.raise_for_status()
    d = r.json()
    follower = [d["follower"][j] for j in JOINT_NAMES] if "follower" in d else None
    leader   = [d["leader"][j]   for j in JOINT_NAMES] if "leader"   in d else None
    return follower, leader, d.get("teleop", False)


# ── Camera ─────────────────────────────────────────────────────────────────────

def open_camera(index, width, height):
    try:
        import cv2
        cap = cv2.VideoCapture(index, cv2.CAP_AVFOUNDATION)
        if not cap.isOpened():
            cap = cv2.VideoCapture(index)
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            cap.set(cv2.CAP_PROP_FPS, 30)
            w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            print(f"  Camera {index}: {w}x{h}")
            return cap
        print(f"  Camera {index}: not available")
        return None
    except ImportError:
        print("  cv2 not installed — skipping camera")
        return None


def read_frame(cap):
    try:
        import cv2
        ret, img = cap.read()
        if ret:
            return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    except Exception:
        pass
    return None


# ── Dataset backends ───────────────────────────────────────────────────────────

def try_lerobot():
    try:
        from lerobot.datasets import LeRobotDataset
        return LeRobotDataset
    except ImportError:
        return None


class FallbackDataset:
    """JSON + PNG fallback when lerobot is not available."""

    def __init__(self, repo_id, fps, root):
        self.root = Path(root)
        self.root.mkdir(parents=True, exist_ok=True)
        self.fps = fps
        self.buf = []
        self.ep_idx = 0
        self.total = 0
        meta = {"repo_id": repo_id, "fps": fps, "joints": JOINT_NAMES}
        (self.root / "meta.json").write_text(json.dumps(meta, indent=2))

    def add_frame(self, frame):
        self.buf.append(frame)

    def save_episode(self):
        ep_dir = self.root / f"episode_{self.ep_idx:06d}"
        ep_dir.mkdir(exist_ok=True)
        records = []
        for i, f in enumerate(self.buf):
            rec = {"frame_index": i, "ts": i / self.fps, "episode_index": self.ep_idx}
            for k in ("action", "observation.state"):
                if k in f:
                    v = f[k]
                    rec[k] = v.tolist() if hasattr(v, "tolist") else v
            if "task" in f:
                rec["task"] = f["task"]
            records.append(rec)
            if "observation.images.laptop" in f:
                try:
                    import cv2
                    img = f["observation.images.laptop"]
                    if isinstance(img, np.ndarray):
                        cv2.imwrite(str(ep_dir / f"frame_{i:06d}.jpg"),
                                    cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
                except Exception:
                    pass
        (ep_dir / "data.json").write_text(json.dumps(records))
        n = len(self.buf)
        self.total += n
        self.ep_idx += 1
        self.buf = []
        print(f"  Saved episode {self.ep_idx - 1}: {n} frames → {ep_dir}")

    @property
    def num_episodes(self): return self.ep_idx

    @property
    def num_frames(self): return self.total

    def finalize(self):
        print(f"Dataset: {self.ep_idx} episodes, {self.total} frames at {self.root}")


def build_features(use_camera, cam_h, cam_w):
    feats = {
        "action":            {"dtype": "float32", "shape": (NUM_JOINTS,), "names": JOINT_NAMES},
        "observation.state": {"dtype": "float32", "shape": (NUM_JOINTS,),
                              "names": [f"{j}_fb" for j in JOINT_NAMES]},
    }
    if use_camera:
        feats["observation.images.laptop"] = {
            "dtype": "video", "shape": (cam_h, cam_w, 3),
            "names": ["height", "width", "channels"],
        }
    return feats


def create_dataset(repo_id, fps, use_camera, cam_h, cam_w, root):
    LeRobotDataset = try_lerobot()
    root_path = Path(root)
    if LeRobotDataset:
        features = build_features(use_camera, cam_h, cam_w)
        if root_path.exists() and (root_path / "meta").exists():
            print("Resuming existing lerobot dataset")
            return LeRobotDataset.resume(repo_id=repo_id, root=str(root_path))
        print("Creating new lerobot dataset")
        return LeRobotDataset.create(
            repo_id=repo_id, fps=fps, features=features, root=str(root_path),
            robot_type="so101_follower", use_videos=use_camera,
            image_writer_processes=0, image_writer_threads=2,
        )
    else:
        print("lerobot not available — using JSON fallback")
        return FallbackDataset(repo_id, fps, str(root_path))


# ── Recording loops ────────────────────────────────────────────────────────────

def record_auto(args, dataset, cal, cam, use_camera):
    """Auto mode: record whenever teleop is active."""
    print(f"\nAuto mode: recording starts/stops with teleop")
    print(f"Task: {args.task}  |  FPS: {args.fps}  |  Max episodes: {args.episodes}")
    print("Press Ctrl+C to finish.\n")

    episodes = 0
    was_teleop = False
    frame_count = 0
    interval = 1.0 / args.fps
    last_frame_t = 0.0

    try:
        while episodes < args.episodes:
            try:
                follower, leader, teleop = get_positions()
            except Exception as e:
                print(f"  API error: {e} — retrying...")
                time.sleep(1.0)
                continue

            if teleop and not was_teleop:
                print(f"\n--- Episode {episodes}: recording started ---")
                was_teleop = True
                frame_count = 0
                last_frame_t = 0.0

            elif not teleop and was_teleop:
                if frame_count > 0:
                    dataset.save_episode()
                    episodes += 1
                    print(f"--- Episode {episodes - 1} done: {frame_count} frames ---")
                else:
                    print("--- Teleop ended with 0 frames, skipping ---")
                was_teleop = False
                frame_count = 0

            if teleop and follower and leader:
                now = time.time()
                if now - last_frame_t >= interval:
                    last_frame_t = now
                    obs = normalize(follower, cal["follower"])
                    act = normalize(leader,   cal["leader"])
                    frame = {"action": act, "observation.state": obs, "task": args.task}
                    if use_camera and cam:
                        img = read_frame(cam)
                        if img is not None:
                            frame["observation.images.laptop"] = img
                    dataset.add_frame(frame)
                    frame_count += 1
                    if frame_count % args.fps == 0:
                        print(f"  {frame_count // args.fps}s ({frame_count} frames)", end="\r")

            time.sleep(0.5 if not teleop else max(0, interval - (time.time() - last_frame_t)))

    except KeyboardInterrupt:
        print("\n\nCtrl-C — stopping.")
        if was_teleop and frame_count > 0:
            dataset.save_episode()
            episodes += 1
            print(f"Saved in-progress episode: {frame_count} frames")

    return episodes


def record_manual(args, dataset, cal, cam, use_camera):
    """Manual mode: Enter to start each episode, Enter again to stop."""
    print(f"\nManual mode: Enter to start/stop episodes")
    print(f"Task: {args.task}  |  FPS: {args.fps}  |  Max episodes: {args.episodes}")
    print("Ctrl+C to finish.\n")

    import select

    episodes = 0
    try:
        while episodes < args.episodes:
            input(f"Press Enter to START episode {episodes}...")

            frame_count = 0
            interval = 1.0 / args.fps
            last_frame_t = 0.0
            recording = True
            print(f"Recording... (press Enter to stop)")

            try:
                while recording:
                    try:
                        follower, leader, _ = get_positions()
                    except Exception as e:
                        print(f"  API error: {e}")
                        time.sleep(0.5)
                        continue

                    if follower and leader:
                        now = time.time()
                        if now - last_frame_t >= interval:
                            last_frame_t = now
                            obs = normalize(follower, cal["follower"])
                            act = normalize(leader,   cal["leader"])
                            frame = {"action": act, "observation.state": obs, "task": args.task}
                            if use_camera and cam:
                                img = read_frame(cam)
                                if img is not None:
                                    frame["observation.images.laptop"] = img
                            dataset.add_frame(frame)
                            frame_count += 1
                            if frame_count % args.fps == 0:
                                print(f"  {frame_count // args.fps}s ({frame_count} frames)", end="\r")

                    if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                        sys.stdin.readline()
                        recording = False

                    time.sleep(max(0, interval - (time.time() - last_frame_t)))

            except KeyboardInterrupt:
                recording = False

            if frame_count > 0:
                dataset.save_episode()
                episodes += 1
                print(f"\nEpisode {episodes - 1} saved: {frame_count} frames")
            else:
                print("\nNo frames, skipping.")

    except (KeyboardInterrupt, EOFError):
        pass

    return episodes


# ── Main ───────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="SO-101 dataset recorder (requires ui/app.py)")
    parser.add_argument("--task",      default="pick and place", help="Task description")
    parser.add_argument("--episodes",  type=int, default=10,     help="Number of episodes")
    parser.add_argument("--fps",       type=int, default=30,     help="Recording FPS")
    parser.add_argument("--repo-id",   default=None,             help="Dataset repo ID")
    parser.add_argument("--output",    default=None,             help="Output directory")
    parser.add_argument("--mode",      choices=["auto", "manual"], default="auto")
    parser.add_argument("--no-camera", action="store_true",      help="Skip camera")
    parser.add_argument("--cam-index", type=int, default=0)
    parser.add_argument("--cam-width", type=int, default=640)
    parser.add_argument("--cam-height",type=int, default=480)
    args = parser.parse_args()

    if args.repo_id is None:
        slug = args.task.lower().replace(" ", "-").replace("/", "-")
        args.repo_id = f"local/so101-{slug}"

    # Verify API
    try:
        r = requests.get(POSITIONS_URL, timeout=2)
        r.raise_for_status()
        d = r.json()
        missing = [arm for arm in ("follower", "leader") if arm not in d]
        if missing:
            print(f"WARNING: Arms not yet assigned in UI: {missing}")
            print("  POST /api/assign to assign arms, or use the UI.")
    except Exception as e:
        print(f"ERROR: Cannot reach UI server at {API_BASE}: {e}")
        print("Start the UI server first: python ui/app.py")
        sys.exit(1)

    # Calibration
    cal = load_calibration()
    if not cal["follower"] or not cal["leader"]:
        print("ERROR: Both arms must be calibrated.")
        sys.exit(1)

    # Camera
    use_camera = not args.no_camera
    cam = None
    if use_camera:
        cam = open_camera(args.cam_index, args.cam_width, args.cam_height)
        use_camera = cam is not None

    # Dataset
    root = args.output or str(Path.home() / f".cache/huggingface/lerobot/{args.repo_id}")
    dataset = create_dataset(args.repo_id, args.fps, use_camera,
                             args.cam_height, args.cam_width, root)
    print(f"Output: {root}\n")

    # Record
    if args.mode == "auto":
        n = record_auto(args, dataset, cal, cam, use_camera)
    else:
        n = record_manual(args, dataset, cal, cam, use_camera)

    dataset.finalize()
    print(f"\nDone: {n} episodes, {dataset.num_frames} frames")
    print(f"Dataset: {root}")

    if cam:
        try:
            cam.release()
        except Exception:
            pass


if __name__ == "__main__":
    main()
