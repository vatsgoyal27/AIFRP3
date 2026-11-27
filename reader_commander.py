import os
import time
from python_st3215 import ST3215
import numpy as np
import pandas as pd

steps = pd.read_csv("joint_trajectory_ticks.csv", header=None).to_numpy()
steps_read = np.zeros_like(steps)
t1_list = steps[:,0]
t2_list = steps[:,1]
t3_list = steps[:,2]
t4_list = steps[:,3]
#print(np.size(t1_list))

PORT = os.environ.get("ST3215_PORT", "COM7")  # use your actual COM number

with ST3215(PORT) as controller:
    print("Scanning for servos...\n")
    servos = controller.list_servos() #find list of servos and save it to servos

    if not servos:
        print("No servos found!")
        exit(1)
    else:
        print(f"Found {len(servos)} servo(s)\n")
        print(servos)

        #mode_names = {0: "Position", 1: "Constant Speed", 2: "PWM", 3: "Stepper"}
        #for servo_id in servos: # for each found servo
            #servo = controller.wrap_servo(servo_id) #create servo object
            #voltage_raw = servo.sram.read_current_voltage() #read its voltage
            #voltage_str = f"{voltage_raw / 10:.1f}V" if voltage_raw is not None else "N/A"
            #mode = servo.eeprom.read_operating_mode() #read its mode
            #print(f"\nServo ID: {servo_id}")
            #print(
            #    f"  Firmware: v{servo.eeprom.read_firmware_major_version()}.{servo.eeprom.read_firmware_minor_version()}" # read its firmware
            #)
            #print(f"  Position: {servo.sram.read_current_location()}") #read its position
            #print(f"  Temperature: {servo.sram.read_current_temperature()}°C") #read its temperature
            #print(f"  Voltage: {voltage_str}") #print its voltage
            #print(
            #    f"  Min/Max Angle: {servo.eeprom.read_min_angle_limit()} / {servo.eeprom.read_max_angle_limit()}" #read set min/max
            #)
            #print(f"  Operating Mode: {mode_names.get(mode, 'Unknown')}") #print mode
    print("moving to start pose")
    for servo_id in servos:
        servo = controller.wrap_servo(servo_id)
        servo.sram.torque_enable()
        servo.sram.write_acceleration(30)
        servo.sram.write_target_location(int(steps[0, servo_id]))
    time.sleep(3)

    print("starting movement now")
    for i in range(len(t1_list)):
        for servo_id in servos: #first, we write
            servo = controller.wrap_servo(servo_id)
            servo.sram.write_target_location(int(steps[i, servo_id]))
        for servo_id in servos: #then, we read!
            servo = controller.wrap_servo(servo_id)
            steps_read[i, servo_id] = servo.sram.read_current_location()
    np.savetxt("steps_read.csv", steps_read, delimiter=",", fmt="%d")
    err = steps_read - steps
    print(np.abs(err).mean(axis=0), np.abs(err).max(axis=0))   # ticks per joint

    print("turning off motors now")
    for servo_id in servos:
        servo = controller.wrap_servo(servo_id)
        servo.sram.torque_disable()


#Simple position control - move servo to different positions.
#with ST3215(PORT) as controller:
    #servo = controller.wrap_servo(1)
    #servo.sram.torque_enable()

    #print("Moving to position 1000...")
    #servo.sram.write_target_location(1000)
    #time.sleep(1)

   # print("Moving to position 3000...")
    #servo.sram.write_target_location(3000)
    #time.sleep(1)

    #print("Moving to center (2048)...")
    #servo.sram.write_target_location(2048)
    #time.sleep(1)
    #servo.sram.torque_disable()


#Read and display current servo position in real-time.
#with ST3215(PORT) as controller:
    #servo = controller.wrap_servo(1)
    #servo.sram.torque_disable()
    #print("Manually move the servo.  Press Ctrl+C to exit.")

    #try:
        #while True:
            #position = servo.sram.read_current_location()
            #speed = servo.sram.read_current_speed()
            #load = servo.sram.read_current_load()

            #pos_str = f"{position:5d}" if position is not None else "  N/A"
            #spd_str = f"{speed:5d}" if speed is not None else "  N/A"
            #load_str = f"{load:4d}" if load is not None else " N/A"

           # print(f"Position: {pos_str} | Speed: {spd_str} | Load: {load_str}", end="\r")
           # time.sleep(0.1)
    #except KeyboardInterrupt:
        #print("\nStopped.")