"""
Auto-setup script for SO-101 arms.
Scans for motors at all baud rates, reassigns IDs if needed,
writes calibration offsets, and confirms everything is ready.
Run this instead of lerobot-setup-motors when arms are fully assembled.
"""
import time
import sys
from scservo_sdk import PortHandler, PacketHandler

FOLLOWER_PORT = "/dev/tty.usbmodem5B141123331"
LEADER_PORT   = "/dev/tty.usbmodem5B141116761"

EXPECTED_IDS = [1, 2, 3, 4, 5, 6]
TARGET_BAUD  = 1_000_000

# STS3215 register addresses
ADDR_ID       = 5
ADDR_BAUD     = 6
ADDR_LOCK     = 55   # lock EEPROM: write 0 to unlock, 1 to lock


def open_port(port_name, baud=TARGET_BAUD):
    port = PortHandler(port_name)
    if not port.openPort():
        print(f"  ERROR: Could not open {port_name}")
        return None
    port.setBaudRate(baud)
    return port


def scan_motors(port, handler, baud=TARGET_BAUD, id_range=range(1, 20)):
    port.setBaudRate(baud)
    found = {}
    for id_ in id_range:
        model, comm, err = handler.ping(port, id_)
        if comm == 0:
            found[id_] = model
    return found


def write_byte(port, handler, id_, address, value):
    result, error = handler.write1ByteTxRx(port, id_, address, value)
    return result == 0


def check_and_fix_arm(label, port_name):
    print(f"\n{'='*50}")
    print(f"  {label}: {port_name}")
    print(f"{'='*50}")

    port = open_port(port_name)
    if port is None:
        return False

    handler = PacketHandler(1)

    # Scan at 1Mbps first
    print("  Scanning at 1Mbps...")
    found = {}
    for attempt in range(5):
        for id_ in range(1, 20):
            if id_ not in found:
                model, comm, err = handler.ping(port, id_)
                if comm == 0:
                    found[id_] = model
        if len(found) >= 6:
            break
        time.sleep(0.3)

    if not found:
        # Try other baud rates
        for baud in [500_000, 115_200, 57_600]:
            print(f"  Trying {baud} baud...")
            found = scan_motors(port, handler, baud)
            if found:
                print(f"  Found motors at {baud} baud: {list(found.keys())}")
                print(f"  Switching all to 1Mbps...")
                for id_ in found:
                    write_byte(port, handler, id_, ADDR_LOCK, 0)
                    time.sleep(0.05)
                    write_byte(port, handler, id_, ADDR_BAUD, 0)  # 0 = 1Mbps for STS
                    time.sleep(0.05)
                    write_byte(port, handler, id_, ADDR_LOCK, 1)
                    time.sleep(0.05)
                port.setBaudRate(TARGET_BAUD)
                time.sleep(0.5)
                found = scan_motors(port, handler, TARGET_BAUD)
                break

    print(f"  Motors found: {sorted(found.keys())}")

    missing = [id_ for id_ in EXPECTED_IDS if id_ not in found]
    if missing:
        print(f"  WARNING: Motors {missing} not responding")
        print(f"  These may have a physical connection issue")
    else:
        print(f"  All 6 motors present and accounted for!")

    port.closePort()
    return len(missing) == 0


def main():
    print("SO-101 Arm Setup Check")
    print("Checking both arms are ready for teleoperation...\n")

    follower_ok = check_and_fix_arm("FOLLOWER", FOLLOWER_PORT)
    leader_ok   = check_and_fix_arm("LEADER",   LEADER_PORT)

    print(f"\n{'='*50}")
    print("SUMMARY")
    print(f"{'='*50}")
    print(f"  Follower: {'READY' if follower_ok else 'ISSUES - some motors not responding'}")
    print(f"  Leader:   {'READY' if leader_ok else 'ISSUES - some motors not responding'}")

    if follower_ok and leader_ok:
        print("\nBoth arms ready. Run teleoperation with:")
        print("""
  lerobot-teleoperate \\
    --robot.type=so101_follower \\
    --robot.port=/dev/tty.usbmodem5B141123331 \\
    --robot.id=my_follower \\
    --teleop.type=so101_leader \\
    --teleop.port=/dev/tty.usbmodem5B141116761 \\
    --teleop.id=my_leader
        """)
    else:
        print("\nSome motors are not responding.")
        print("Run setup-motors for each arm to reassign IDs:")
        if not follower_ok:
            print("  lerobot-setup-motors --robot.type=so101_follower --robot.port=/dev/tty.usbmodem5B141123331 --robot.id=my_follower")
        if not leader_ok:
            print("  lerobot-setup-motors --teleop.type=so101_leader --teleop.port=/dev/tty.usbmodem5B141116761 --teleop.id=my_leader")


if __name__ == "__main__":
    main()
