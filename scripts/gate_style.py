import time

if __name__ == "__main__":
    print("Do not run this script directly. Use launch.py to run it.")
    exit(1)

from control_board import ControlBoard, Simulator
import comm
import serial
import threading
import os

spin_after = 10.0       # Seconds to do style spins after
spin_duration = 5.0     # How long to spin for

def moving_delay(cb: ControlBoard, sec: float):
    start_time = time.time()
    while time.time() - start_time < sec:
        cb.feed_motor_watchdog()
        time.sleep(0.1)

def kill_check():
    ser = serial.Serial("/dev/ttyACM2", 57600)
    while True:
        msg_id, msg = comm.read_msg(ser)
        if msg.startswith(b'TARM'):
            is_armed = (msg[4] == 1)
            if not is_armed:
                print("KILLED EXITING")
                os._exit(6)

def run(cb: ControlBoard, s: Simulator) -> int:
    t = threading.Thread(target=kill_check, daemon=True)
    t.start()

    # Enable automatic reading of sensor data (and wait for it to start)
    cb.read_bno055_periodic(True)
    cb.read_ms5837_periodic(True)
    time.sleep(0.5)

    # Store initial heading
    initial_heading = cb.get_bno055_data().yaw

    # Submerge holding heading
    print("Submerging")
    cb.set_sassist2(0, 0, 0, 0, initial_heading, -1.5)
    moving_delay(cb, 1.25)

    # Move forward maintaining same heading
    print("Moving forward")
    cb.set_sassist2(0, 0.8, 0, 0, initial_heading, -1.5)
    moving_delay(cb, spin_after)

    # Do style points spins
    print("Spinning")
    cb.set_sassist1(0, 0, 1.0, 0, 0, -1.5)
    moving_delay(cb, spin_duration)

    # Get back to correct heading
    print("Fixing heading")
    cb.set_sassist2(0, 0.0, 0, 0, initial_heading, -1.5)
    moving_delay(cb, 3)

    # Move forward
    print("Moving forward again")
    cb.set_sassist2(0, 0.8, 0, 0, initial_heading, -1.5)
    moving_delay(cb, 5)

    # Stop motion
    print("Stopping")
    cb.set_local(0, 0, 0, 0, 0, 0)

    return 0
