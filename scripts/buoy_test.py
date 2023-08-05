import time


if __name__ == "__main__":
    print("Do not run this script directly. Use launch.py to run it.")
    exit(1)


from control_board import ControlBoard, Simulator
import meb
import serial
import threading
import os
import cv2
from math import abs
from yolov5_detect import get_center_diffs_yolo

################################################################################
# Vision / CV / ML
################################################################################

class CV:
    def __init__(self):
        self.frame = None
    
    def set_frame(self, frame):
        self.frame = frame
    
    def get_frame(self):
        return self.frame

cv = CV()

cv_model_path = "/home/sw8/SW8S-Java/app/models/buoy_640.onnx"

net = cv2.dnn.readNet(cv_model_path)
net.setPreferableBackend(cv2.dnn.DNN_BACKEND_DEFAULT)


def start_capture():
    video = cv2.VideoCapture(0)
    ret, im = video.read()
    cv.set_frame(im)

################################################################################


################################################################################
# MEB
################################################################################

def meb_task():
    ser = serial.Serial("/dev/ttyACM2", 57600)
    while True:
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

def moving_delay(cb: ControlBoard, sec: float):
    start_time = time.time()
    while time.time() - start_time < sec:
        cb.feed_motor_watchdog()
        time.sleep(0.02)


def run(cb: ControlBoard, s: Simulator) -> int:
    cam = threading.Thread(target=start_capture)
    cam.start()
    

    meb_thread = threading.Thread(target=meb_task, daemon=True)
    meb_thread.start()

    mission_depth = -0.7

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

    if False:
        # Move towards buoy (dead reckon)
        print("Go to buoy")
        cb.set_sassist2(0, 0.4, 0, 0, initial_heading, mission_depth)
        moving_delay(cb, 2)
    else:
        # try and find buoys (CV / ML)
        while True:
            frame = cv.get_frame()
            strafe_speed = 0.3
            strafe = 0
            if frame != None:
                diffs = get_center_diffs_yolo()
                if diffs != None:
                    if abs(diffs["x"]) > 50: # if the center of the screen X is within 50 pixels of the buoy target center average X
                        strafe = strafe_speed if diffs["x"] < 0 else -strafe_speed
                    else:
                        strafe = 0
            cb.set_sassist2(strafe, 0.4, 0, 0, initial_heading, mission_depth)

    # Stop motion
    print("Stopping")
    cb.set_local(0, 0, 0, 0, 0, 0)

    return 0

################################################################################