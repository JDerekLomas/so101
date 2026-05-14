"""Scan all connected controller boards for motors."""
import glob
from scservo_sdk import PortHandler, PacketHandler

EXPECTED_IDS = [1, 2, 3, 4, 5, 6]
ID_NAMES = {1: "shoulder_pan", 2: "shoulder_lift", 3: "elbow_flex",
            4: "wrist_flex", 5: "wrist_roll", 6: "gripper"}

ports = sorted(glob.glob("/dev/tty.usbmodem*"))
if not ports:
    print("No controller boards found.")
else:
    for port_path in ports:
        try:
            port = PortHandler(port_path)
            if not port.openPort():
                print(f"{port_path}: failed to open")
                continue
            handler = PacketHandler(1)
            port.setBaudRate(1_000_000)
            found = {}
            for motor_id in range(1, 20):
                model_nb, result, error = handler.ping(port, motor_id)
                if result == 0:
                    found[motor_id] = model_nb
            port.closePort()
            missing = [i for i in EXPECTED_IDS if i not in found]
            status = "OK" if not missing else f"MISSING {[ID_NAMES[i] for i in missing]}"
            print(f"{port_path}: {status}")
            for mid in sorted(found):
                print(f"  ID {mid}: {ID_NAMES.get(mid, '?')}")
        except Exception:
            print(f"{port_path}: NOT CONNECTED")
