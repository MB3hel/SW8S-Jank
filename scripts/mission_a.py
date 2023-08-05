import time


if __name__ == "__main__":
    print("Do not run this script directly. Use launch.py to run it.")
    exit(1)


from control_board import ControlBoard, Simulator
import meb
import serial
import threading
import os

################################################################################
# MEB Comms
################################################################################

def meb_task():
    ser = None
    try:
        ser = serial.Serial("/dev/ttyACM2", 57600)
    except:
        print("Failed to connect to MEB")
        return
    
    while True:
        # Read message and handle it
        msg_id, msg = meb.read_msg(ser)
        if msg.startswith(b'TARM'):
            is_armed = (msg[4] == 1)
            if not is_armed:
                print("Thrusters killed. Ending code!")
                os._exit(6)

################################################################################


################################################################################
# Mission code
################################################################################


# Delay for an amount of time (nearest 20ms) while keeping 
# the control board watchdog fed (thus motors keep moving)
def moving_delay(cb: ControlBoard, sec: float):
    start_time = time.time()
    while time.time() - start_time < sec:
        cb.feed_motor_watchdog()
        time.sleep(0.02)


def run(cb: ControlBoard, s: Simulator) -> int:
    mission_depth = -0.7

    meb_thread = threading.Thread(target=meb_task, daemon=True)
    meb_thread.start()

    # Enable automatic reading of sensor data (and wait for it to start)
    cb.read_bno055_periodic(True)
    cb.read_ms5837_periodic(True)
    time.sleep(0.5)

    # Store initial heading
    initial_heading = cb.get_bno055_data().yaw

    # Submerge holding heading
    print("Submerging")
    cb.set_sassist2(0, 0, 0, 0, initial_heading, mission_depth)
    moving_delay(cb, 1.25)

    # Move forward maintaining same heading
    print("Moving forward")
    cb.set_sassist2(0, 0.4, 0, 0, initial_heading, mission_depth)
    moving_delay(cb, 3)

    # Do style points spins
    if True:
        # Yaw spins
        print("Spinning yaw")
        cb.set_sassist1(0, 0, 0.6, 0, 0, mission_depth)
        moving_delay(cb, 5)
    else:
        # Roll spins
        print("Spinning roll")
        cb.set_dhold(0, 0, 0, 0.8, 0, mission_depth)
        moving_delay(cb, 5)

    # Get back to correct orientation
    print("Fixing orientation")
    print(initial_heading)
    cb.set_sassist2(0, 0, 0, 0, initial_heading, mission_depth)
    while abs(cb.get_bno055_data().pitch - 0) > 5 or \
            abs(cb.get_bno055_data().roll - 0) > 5 or \
            abs(cb.get_bno055_data().yaw - initial_heading) > 5:
        moving_delay(cb, 0.02)
    moving_delay(cb, 1)

    # Move forward
    print("Moving forward again")
    cb.set_sassist2(0, 0.4, 0, 0, initial_heading, mission_depth)
    moving_delay(cb, 2)

    # Stop motion
    print("Stopping")
    cb.set_local(0, 0, 0, 0, 0, 0)

    return 0

################################################################################
