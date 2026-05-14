"""
Wrist follow — leader wrist_flex controls follower wrist_flex.
Safety: 5% margin inside calibrated range, speed limited to 200.
Run for N seconds (default 30), then stops and disables torque.
"""
import os, sys, time, signal
sys.stderr = open(os.devnull, "w")
from scservo_sdk import PortHandler, PacketHandler

FOLLOWER_PORT = "/dev/tty.usbmodem5B141123331"
LEADER_PORT   = "/dev/tty.usbmodem5B141116761"
MOTOR_ID = 4   # wrist_flex
DURATION = 30  # seconds

# Calibrated ranges (from my_follower.json / my_leader.json)
L_MIN, L_MAX = 875, 3230
F_MIN, F_MAX = 945, 3339
MARGIN = 0.05
L_LO = int(L_MIN + (L_MAX - L_MIN) * MARGIN)
L_HI = int(L_MAX - (L_MAX - L_MIN) * MARGIN)
F_LO = int(F_MIN + (F_MAX - F_MIN) * MARGIN)
F_HI = int(F_MAX - (F_MAX - F_MIN) * MARGIN)


def map_val(v, in_lo, in_hi, out_lo, out_hi):
    v = max(in_lo, min(in_hi, v))
    return int(out_lo + (v - in_lo) / (in_hi - in_lo) * (out_hi - out_lo))


lp = PortHandler(LEADER_PORT);   lh = PacketHandler(0)
fp = PortHandler(FOLLOWER_PORT);  fh = PacketHandler(0)
lp.openPort(); lp.setBaudRate(1_000_000)
fp.openPort(); fp.setBaudRate(1_000_000)
time.sleep(0.2)

# Enable torque + speed limit on follower wrist_flex
fh.write1ByteTxRx(fp, MOTOR_ID, 40, 1)
fh.write2ByteTxRx(fp, MOTOR_ID, 46, 200)
time.sleep(0.05)

print(f"Wrist follow active for {DURATION}s")
print(f"  Move the leader wrist_flex freely.")
print(f"  Leader safe:   {L_LO}–{L_HI}")
print(f"  Follower safe: {F_LO}–{F_HI}")
print()

running = True
def stop(sig, frame):
    global running
    running = False
signal.signal(signal.SIGINT, stop)
signal.signal(signal.SIGTERM, stop)

start = time.time()
last_target = -1
while running and (time.time() - start) < DURATION:
    lval, lc, _ = lh.read2ByteTxRx(lp, MOTOR_ID, 56)
    fval, fc, _ = fh.read2ByteTxRx(fp, MOTOR_ID, 56)
    if lc == 0 and fc == 0:
        target = map_val(lval, L_LO, L_HI, F_LO, F_HI)
        if abs(target - last_target) >= 3:
            fh.write2ByteTxRx(fp, MOTOR_ID, 42, target)
            last_target = target
        elapsed = time.time() - start
        print(f"  t={elapsed:5.1f}s  leader={lval:4d}  →  target={target:4d}  follower={fval:4d}", end="\r")
    time.sleep(0.05)

fh.write1ByteTxRx(fp, MOTOR_ID, 40, 0)
lp.closePort()
fp.closePort()
print("\nDone. Torque off.")
