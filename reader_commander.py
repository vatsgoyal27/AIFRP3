import os
import time
from python_st3215 import ST3215
import numpy as np
import pandas as pd

PORT = os.environ.get("ST3215_PORT", "COM7") 

# column in CSV -> servo ID  (edit to match your real IDs)
# columns: 0 = theta1, 1 = theta2, 2 = theta3, 3 = q4
JOINT_TO_ID = {0: 1, 1: 2, 2: 3, 3: 4}

ACCEL = 30
STEP_DELAY = 0.2   
TICK_MIN, TICK_MAX = 0, 4095

steps = pd.read_csv("joint_trajectory_ticks.csv", header=None).to_numpy()
steps_read = np.zeros_like(steps)
N = steps.shape[0]

print("tick range per column (min):", steps.min(axis=0))
print("tick range per column (max):", steps.max(axis=0))

# clamp to valid servo range so an out-of-range value doesn't silently fail
steps = np.clip(steps, TICK_MIN, TICK_MAX)

with ST3215(PORT) as controller:
    print("Scanning for servos...\n")
    servos = controller.list_servos()

    if not servos:
        print("No servos found!")
        exit(1)

    print(f"Found {len(servos)} servo(s): {servos}\n")

    # check every expected servo is present
    missing = [sid for sid in JOINT_TO_ID.values() if sid not in servos]
    if missing:
        print(f"WARNING: expected servo IDs not found: {missing}")

    # build servo objects only for joints that exist
    wrapped = {}
    for col, sid in JOINT_TO_ID.items():
        if sid in servos:
            wrapped[col] = controller.wrap_servo(sid)

    # print limits for each servo so you can compare with your ticks
    for col, servo in wrapped.items():
        sid = JOINT_TO_ID[col]
        print(f"Joint col {col} (ID {sid}): "
              f"min/max = {servo.eeprom.read_min_angle_limit()} / "
              f"{servo.eeprom.read_max_angle_limit()}, "
              f"temp = {servo.sram.read_current_temperature()}C")

    print("\nmoving to start pose")
    for col, servo in wrapped.items():
        servo.sram.torque_enable()
        servo.sram.write_acceleration(ACCEL)
        servo.sram.write_target_location(int(steps[0, col]))
    time.sleep(3)

    print("starting movement now")
    for i in range(N):
        # write all targets first
        for col, servo in wrapped.items():
            servo.sram.write_target_location(int(steps[i, col]))
        time.sleep(STEP_DELAY)
        # then read all positions
        for col, servo in wrapped.items():
            pos = servo.sram.read_current_location()
            steps_read[i, col] = pos if pos is not None else -1

    np.savetxt("steps_read.csv", steps_read, delimiter=",", fmt="%d")

    err = steps_read - steps
    print("mean abs err (ticks per joint):", np.abs(err).mean(axis=0))
    print("max abs err  (ticks per joint):", np.abs(err).max(axis=0))

    time.sleep(2)
    print("turning off motors now")
    for col, servo in wrapped.items():
        servo.sram.torque_disable()