"""Read current position from each motor on the follower arm."""
from scservo_sdk import PortHandler, PacketHandler

PORT = "/dev/tty.usbmodem5B141123331"
MOTORS = {
    1: "shoulder_pan",
    2: "shoulder_lift",
    3: "elbow_flex",
    4: "wrist_flex",
    5: "wrist_roll",
    6: "gripper",
}
PRESENT_POSITION_ADDR = 56  # STS3215 present position register

port = PortHandler(PORT)
handler = PacketHandler(1)

if not port.openPort():
    print(f"Failed to open {PORT}")
    exit(1)
port.setBaudRate(1_000_000)

print(f"Reading positions from follower arm ({PORT})...\n")
for id_, name in MOTORS.items():
    responded = False
    for _ in range(10):
        val, comm, err = handler.read2ByteTxRx(port, id_, PRESENT_POSITION_ADDR)
        if comm == 0:
            print(f"  Motor {id_} ({name}): position = {val}")
            responded = True
            break
    if not responded:
        print(f"  Motor {id_} ({name}): NO RESPONSE")

port.closePort()
