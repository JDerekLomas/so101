"""Nudge the gripper open by a small amount."""
import sys
import time
from scservo_sdk import PortHandler, PacketHandler

PORT = "/dev/tty.usbmodem5B141123331"
GRIPPER_ID = 6
ADDR_PRESENT_POS = 56
ADDR_GOAL_POS = 42
ADDR_TORQUE_ENABLE = 40

# How many steps to nudge (out of 0-4095 range). Positive = higher position value.
NUDGE = int(sys.argv[1]) if len(sys.argv) > 1 else 80

port = PortHandler(PORT)
handler = PacketHandler(0)  # protocol 0 for STS3215

if not port.openPort():
    print(f"Failed to open {PORT}")
    sys.exit(1)
port.setBaudRate(1_000_000)

# Read current position
pos, comm, _ = handler.read2ByteTxRx(port, GRIPPER_ID, ADDR_PRESENT_POS)
if comm != 0:
    print(f"Failed to read gripper position (comm error {comm})")
    port.closePort()
    sys.exit(1)

target = max(0, min(4095, pos + NUDGE))
print(f"Gripper current position: {pos}")
print(f"Moving to: {target} (nudge {NUDGE:+d})")

# Enable torque
handler.write1ByteTxRx(port, GRIPPER_ID, ADDR_TORQUE_ENABLE, 1)
time.sleep(0.05)

# Write goal position
handler.write2ByteTxRx(port, GRIPPER_ID, ADDR_GOAL_POS, target)
time.sleep(0.5)

# Read final position
final, comm, _ = handler.read2ByteTxRx(port, GRIPPER_ID, ADDR_PRESENT_POS)
if comm == 0:
    print(f"Final position: {final}")

port.closePort()
