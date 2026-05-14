#!/bin/bash
# SO-101 dataset recording script
# Usage: ./record.sh <task_description> [num_episodes]
#
# Cameras:
#   laptop (index 0) — Mac built-in FaceTime camera, 640x480 @ 30fps
#   phone  (index 1) — iPhone via Continuity Camera (connect iPhone via Bluetooth/WiFi)
#                      Or set PHONE_INDEX env var if it appears on a different index
#
# To add phone camera: ensure iPhone is near Mac with both on same WiFi,
# check System Settings → Privacy & Security → Camera, then run this script.

set -e

TASK="${1:-pick and place}"
NUM_EPISODES="${2:-10}"
PHONE_INDEX="${PHONE_INDEX:-1}"
HF_USER="${HF_USER:-$(python3 -c 'from huggingface_hub import whoami; print(whoami()["name"])' 2>/dev/null || echo "local")}"

# Sanitize task name for repo id
TASK_SLUG=$(echo "$TASK" | tr '[:upper:]' '[:lower:]' | sed 's/[^a-z0-9]/-/g' | sed 's/--*/-/g' | sed 's/^-\|-$//g')
REPO_ID="${HF_USER}/so101-${TASK_SLUG}"

echo "Task:     $TASK"
echo "Episodes: $NUM_EPISODES"
echo "Repo:     $REPO_ID"
echo ""

# Check for phone camera
PHONE_AVAILABLE=false
python3 -c "
import cv2, sys
cap = cv2.VideoCapture($PHONE_INDEX, cv2.CAP_AVFOUNDATION)
sys.exit(0 if cap.isOpened() else 1)
" 2>/dev/null && PHONE_AVAILABLE=true

if [ "$PHONE_AVAILABLE" = true ]; then
    echo "Cameras: laptop (0) + phone ($PHONE_INDEX)"
    CAMERAS="{laptop: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}, phone: {type: opencv, index_or_path: $PHONE_INDEX, width: 640, height: 480, fps: 30}}"
else
    echo "Cameras: laptop only (phone not detected at index $PHONE_INDEX)"
    CAMERAS="{laptop: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}}"
fi

echo ""
echo "Starting recording in 3 seconds... (Ctrl+C to cancel)"
sleep 3

source ~/lerobot-env-312/bin/activate
cd ~/so101/lerobot

python -m lerobot.scripts.lerobot_record \
    --robot.type=so101_follower \
    --robot.port=/dev/tty.usbmodem5B141123331 \
    --robot.id=my_follower \
    --robot.cameras="$CAMERAS" \
    --teleop.type=so101_leader \
    --teleop.port=/dev/tty.usbmodem5B141116761 \
    --teleop.id=my_leader \
    --dataset.repo_id="$REPO_ID" \
    --dataset.num_episodes="$NUM_EPISODES" \
    --dataset.single_task="$TASK" \
    --dataset.streaming_encoding=true \
    --dataset.encoder_threads=2 \
    --display_data=true
