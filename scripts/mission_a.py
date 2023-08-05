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
import time
import random
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

cv_model_path = os.path.dirname(os.path.dirname(__file__)) + "/models/buoy_320.onnx"

net = cv2.dnn.readNet(cv_model_path)
net.setPreferableBackend(cv2.dnn.DNN_BACKEND_DEFAULT)


def save_file():
    f = ""
    while True:
        f = "/mnt/data/py_record/camFRONT_" + str(random.randint(0, 999999)) + ".mp4"
        if not os.path.exists(f):
            break
    return f


def start_capture():
    cap0Pl = "v4l2src device=/dev/video0 ! image/jpeg, width=800, height=600, framerate=30/1" + " ! jpegdec ! tee name=raw " + \
            "raw. ! queue  ! videoconvert ! appsink " + \
            "raw. ! queue  ! videoconvert ! x264enc tune=zerolatency speed-preset=ultrafast bitrate=2048000 ! video/x-h264,profile=baseline" + " ! tee name=h264 " +\
            "h264. ! queue ! h264parse config_interval=-1 ! video/x-h264,stream-format=byte-stream,alignment=au ! rtspclientsink location=rtsp://127.0.0.1:8554/cam0 " + \
            "h264. ! queue ! mpegtsmux ! filesink location=\"" + save_file() + "\" "
    video = cv2.VideoCapture(cap0Pl, cv2.CAP_GSTREAMER)
    while True:
        ret, im = video.read()
        cv.set_frame(im)

# def start_capture():
#     while True:
#         im = cv2.imread("/home/marcus/buoy1.png")
#         cv.set_frame(im)
#         time.sleep(0.015)

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

    mission_depth = -2.5
    forward_speed = 0.8

    # Enable automatic reading of sensor data (and wait for it to start)
    cb.read_bno055_periodic(True)
    cb.read_ms5837_periodic(True)
    time.sleep(0.5)

    # Store initial heading
    initial_heading = cb.get_bno055_data().yaw

    # Submerge holding heading
    print("Submerging")
    cb.set_sassist2(0, 0, 0, 0, initial_heading, mission_depth)
    moving_delay(cb, 3)

    # Move forward maintaining same heading
    print("Moving forward")
    cb.set_sassist2(0, forward_speed, 0, 0, initial_heading, mission_depth)
    moving_delay(cb, 17)

    # Do style points spins
    if False:
        # Yaw spins
        print("Spinning yaw")
        cb.set_sassist1(0, 0, 0.8, 0, 0, mission_depth)
        sayaw = cb.get_bno055_data().accum_yaw
        while abs(cb.get_bno055_data().accum_yaw - sayaw) < 720:
            moving_delay(cb, 0.5)
    else:
        # Roll spins
        print("Spinning roll")
        cb.set_dhold(0, 0, 0, 0.6, 0, mission_depth)
        saroll = cb.get_bno055_data().accum_roll
        while abs(cb.get_bno055_data().accum_roll - saroll) < 270:
            moving_delay(cb, 0.5)

    # Get back to correct orientation
    print("Fixing orientation")
    cb.set_sassist2(0, 0, 0, 0, initial_heading, mission_depth)
    start_wait = time.time()
    while abs(cb.get_bno055_data().pitch - 0) > 5 or \
            abs(cb.get_bno055_data().roll - 0) > 5 or \
            abs(cb.get_bno055_data().yaw - initial_heading) > 5:
        moving_delay(cb, 0.02)
        if time.time() - start_wait > 5:
            break
    moving_delay(cb, 3)

    # Move forward
    # print("Moving forward again")
    # cb.set_sassist2(0, forward_speed, 0, 0, initial_heading, mission_depth)
    # moving_delay(cb, 2)

    # Yaw to face buoy & submerge more
    buoy_depth = -2.5
    print("Yaw to buoy")
    cb.set_sassist2(0, 0, 0, 0, initial_heading + 15, buoy_depth)
    buoy_heading = initial_heading + 15
    start_wait = time.time()
    while abs(cb.get_bno055_data().yaw - buoy_heading) > 3:
        moving_delay(cb, 0.02)
        if time.time() - start_wait > 5:
            break
    moving_delay(cb, 2)

    if False:
        # Move towards buoy (dead reckon)
        print("Go to buoy")
        cb.set_sassist2(0, forward_speed, 0, 0, buoy_heading, buoy_depth)
        moving_delay(cb, 20)
    else:
        # try and find buoys (CV / ML)
        print("Vision to buoy")
        start_wait = time.time()
        while True:
            frame = cv.get_frame()
            strafe_speed = 0.3
            strafe = 0
            if frame is not None:
                diffs = get_center_diffs_yolo(frame, net)
                if diffs != None and "x" in diffs:
                    if not hasattr(diffs["x"], "__len__"):
                        xdiff = diffs["x"]
                        if abs(xdiff) > 50: # if the center of the screen X is within 50 pixels of the buoy target center average X
                            strafe = strafe_speed if xdiff < 0 else -strafe_speed
                        else:
                            strafe = 0
                    else:
                        strafe = 0
            cb.set_sassist2(strafe, forward_speed, 0, 0, buoy_heading, buoy_depth)
            if time.time() - start_wait > 20:
                break
    
    if False:
        print("Backup")
        cb.set_sassist2(0, -0.4, 0, 0, buoy_heading, buoy_depth)
        moving_delay(cb, 3)

        print("Forward again")
        cb.set_sassist2(0, 0.4, 0, 0, buoy_heading, buoy_depth)
        moving_delay(cb, 10)


    # Stop motion
    print("Stopping")
    cb.set_local(0, 0, 0, 0, 0, 0)

    return 0

################################################################################
