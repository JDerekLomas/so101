#!/usr/bin/env python3
"""Test which direction elbow_flex needs to go to lift off desk."""
import sys
sys.path.insert(0, "/Users/dereklomas/lerobot-env-312/lib/python3.12/site-packages")
from scservo_sdk import PortHandler, PacketHandler
import time

PORT = "/dev/tty.usbmodem5B141123331"
ph = PortHandler(PORT)
if not ph.openPort():
    print("Port busy — wait a few seconds and retry"); exit(1)
ph.setBaudRate(1000000)
h = PacketHandler(0)

MID_SL = 2  # shoulder_lift
MID_EF = 3  # elbow_flex

def w2(mid, addr, val):
    ph.is_using = False; h.write2ByteTxRx(ph, mid, addr, val); time.sleep(0.005)
def w1(mid, addr, val):
    ph.is_using = False; h.write1ByteTxRx(ph, mid, addr, val); time.sleep(0.005)
def r2(mid, addr):
    ph.is_using = False
    try:
        v, _, _ = h.read2ByteTxRx(ph, mid, addr)
        time.sleep(0.005)
        return v
    except Exception:
        time.sleep(0.05)
        return None

# 1. Raise shoulder very high first
print("Step 1: Raising shoulder to 2300 (very high)...")
w1(MID_SL, 40, 1)  # torque on
w2(MID_SL, 46, 500)  # speed
w2(MID_SL, 42, 2300)  # high target

for i in range(20):
    time.sleep(0.5)
    sl = r2(MID_SL, 56)
    ef = r2(MID_EF, 56)
    if sl: print(f"  shoulder={sl}  elbow={ef}")
    if sl and abs(sl - 2300) < 40:
        print("  Shoulder arrived!")
        break

# 2. Check where elbow settled after shoulder raise
ef_now = r2(MID_EF, 56)
print(f"\nElbow position after shoulder raise: {ef_now}")
print(f"(If elbow swung off desk, it should have changed from ~165)")

# 3. Test elbow in BOTH directions
print("\n--- Test A: elbow INCREASE (+400, toward 4095) ---")
w1(MID_EF, 40, 1)
w2(MID_EF, 46, 200)
target_a = min(4095, ef_now + 400) if ef_now else 600
w2(MID_EF, 42, target_a)
print(f"Goal: {target_a}")
for i in range(8):
    time.sleep(0.4)
    pos = r2(MID_EF, 56)
    ld_raw = r2(MID_EF, 60)
    ld = (ld_raw & 0x3FF) if ld_raw else 0
    print(f"  pos={pos}  load={ld}  moved={pos - ef_now if pos and ef_now else '?'}")

# Stop, return
w2(MID_EF, 42, r2(MID_EF, 56) or ef_now)
time.sleep(1)

print("\n--- Test B: elbow DECREASE (-100, toward 0) ---")
ef_now2 = r2(MID_EF, 56) or ef_now
target_b = max(0, ef_now2 - 100)
w2(MID_EF, 42, target_b)
print(f"Goal: {target_b} (from {ef_now2})")
for i in range(8):
    time.sleep(0.4)
    pos = r2(MID_EF, 56)
    ld_raw = r2(MID_EF, 60)
    ld = (ld_raw & 0x3FF) if ld_raw else 0
    print(f"  pos={pos}  load={ld}  moved={pos - ef_now2 if pos and ef_now2 else '?'}")

# Torque off
print("\nTorque off.")
w1(MID_SL, 40, 0)
w1(MID_EF, 40, 0)
ph.closePort()
