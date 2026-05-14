#!/usr/bin/env python3
"""
Non-interactive SO-101 follower motor setup script.
Run with: source ~/lerobot-env-312/bin/activate && python ~/setup_so101.py <motor_name>

Motor order (do them in this sequence):
  1. gripper
  2. wrist_roll
  3. wrist_flex
  4. elbow_flex
  5. shoulder_lift
  6. shoulder_pan

For each motor:
  - Connect ONLY that motor to the controller board (3-pin cable)
  - Run this script with the motor name
  - Then move to the next motor
"""

import sys
from lerobot.robots.so_follower.config_so_follower import SOFollowerRobotConfig
from lerobot.robots.so_follower.so_follower import SOFollower

PORT = "/dev/tty.usbmodem5B141116761"

MOTOR_ORDER = ["gripper", "wrist_roll", "wrist_flex", "elbow_flex", "shoulder_lift", "shoulder_pan"]

def setup_single_motor(motor_name: str):
    if motor_name not in MOTOR_ORDER:
        print(f"Error: '{motor_name}' not valid. Choose from: {MOTOR_ORDER}")
        sys.exit(1)

    config = SOFollowerRobotConfig(port=PORT)
    robot = SOFollower(config)

    print(f"Setting up '{motor_name}' motor...")
    robot.bus.setup_motor(motor_name)
    motor_id = robot.bus.motors[motor_name].id
    print(f"Done! '{motor_name}' motor id set to {motor_id}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python setup_so101.py <motor_name>")
        print(f"Motor order: {MOTOR_ORDER}")
        print("\nConnect only ONE motor at a time to the controller board.")
        sys.exit(0)

    setup_single_motor(sys.argv[1])
