import time

if __name__ == "__main__":
    print("Do not run this script directly. Use launch.py to run it.")
    exit(1)

from control_board import ControlBoard, Simulator


def moving_delay(cb: ControlBoard, sec: float):
    start_time = time.time()
    while time.time() - start_time < sec:
        cb.feed_motor_watchdog()
        time.sleep(0.1)


def run(cb: ControlBoard, s: Simulator) -> int:
    # Enable automatic reading of sensor data (and wait for it to start)
    cb.read_bno055_periodic(True)
    cb.read_ms5837_periodic(True)
    time.sleep(0.5)

    # Store initial heading
    initial_heading = cb.get_bno055_data().yaw

    # Submerge holding heading
    print("Submerging")
    cb.set_sassist2(0, 0, 0, 0, initial_heading, -0.8)
    moving_delay(cb, 1.25)

    # Move forward maintaining same heading
    print("Moving forward")
    cb.set_sassist2(0, 0.4, 0, 0, initial_heading, -0.8)
    moving_delay(cb, 20)

    # Determine any heading error due to PID tuning
    final_heading = cb.get_bno055_data().yaw

    # Surface
    print("Surfacing")
    cb.set_sassist2(0, 0, 0, 0, initial_heading, 0.0)
    moving_delay(cb, 1.5)
    
    # Stop motion
    print("Stopping")
    cb.set_local(0, 0, 0, 0, 0, 0)

    # Print results
    print("While moving forward, vehicle heading changed by {:.16f} degrees due to PID errors. Other heading errors are due to IMU drift.".format(abs(final_heading - initial_heading)))

    return 0
