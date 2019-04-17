"""
This file holds functions that may be helpful in the state machine. It's purpose is to keep the state_machine file
as clean as possible.
"""
import cv2
import numpy as np
import imutils
from pyzbar import pyzbar
from imutils.video import VideoStream
import time
import re
#import matplotlib.pyplot as plt
import os

# LOAD the camera parameters.
with np.load(os.path.abspath(os.path.join(os.path.dirname(__file__), 'camera_cal_output.npz'))) as X:
    MTX, DIST, _, _ = [X[i] for i in ('mtx', 'dist', 'rvecs', 'tvecs')]

DRIVE_SPEED = 50

def get_camera():
    """
    Returns a VideoStream object. It will try to start a VideoStream using the picamera, and if it that is unsuccessful,
    it will start a video stream using the default camera.
    :return: VideoStream object.
    """
    try:
        # this will only run if it's on the pi
        vs = VideoStream(usePiCamera=True)
        time.sleep(0.5)
        vs.start()
    except:
        # if it's not running on the pi
        vs = VideoStream(src=0).start()

    time.sleep(2.0)
    return vs


def extract_floor_barcode_data(data):
    output_dict = {}
    # <x: 080.0, y: 127.5> !pz!
    location_data = re.match('<x:\s*(?P<x>\d*\.\d*),\s*y:\s*(?P<y>\d*\.\d*)>.*', data)
    if location_data:
        output_dict['location'] = {}
        output_dict['location']['x'] = float(location_data.group('x'))
        output_dict['location']['y'] = float(location_data.group('y'))

    # Parse pallet rack data
    pallet_rack_data = re.match('.*PRacks:?([\d+,]+).*', data)
    if pallet_rack_data:
        output_dict['pallet_racks'] = [int(x) for x in pallet_rack_data.groups()[0].split(',')]

    pallets_data = re.match('.*Pallets:([\d+,]+).*', data)
    if pallets_data:
        output_dict['pallets'] = [int(x) for x in pallets_data.groups()[0].split(',')]

    output_dict['time'] = time.time()
    return output_dict


def extract_goal_barcode_data(data):
    output_dict = {'goal': {}}
    output_dict['goal']['x'] = 100.0
    output_dict['goal']['y'] = 200.0
    return output_dict


def nothing(nothing):
    """
    doesn't do anything
    used with trackbars
    """
    pass


def read_goal_qr():
    """
    Parses QR Code data as formatted for the competition.
    :param vs: imutils.video.VideoStream object
    :param show_video: set to True to have a live stream pop up.
    :return: Array of QR code dictionaries.
    """

    vs = get_camera()

    # cap.set(15, contrast)
    frame = vs.read()
    qr_found = False
    output_dict = None
    while not qr_found:
        frame = vs.read()
        frame = imutils.resize(frame, width=400)
        try:
            barcodes = pyzbar.decode(frame)
        except TypeError:
            continue

        if barcodes is not None and len(barcodes) == 1:
            barcode = barcodes[0]
            barcode_data = barcode.data.decode("utf-8").replace(' ', '')

            # Parse barcode location data
            output_dict = extract_goal_barcode_data(barcode_data)

            if output_dict:
                qr_found = True

    vs.stream.stop()
    return output_dict



def read_qr(show_video=False):
    """
    Parses QR Code data as formatted for the competition.
    :param vs: imutils.video.VideoStream object
    :param show_video: set to True to have a live stream pop up.
    :return: Array of QR code dictionaries.
    """

    # Set up video capture. This is different if using the picamera, but we're not for this application?
    cap = cv2.VideoCapture(1)
    # cap.set(15, 0.05)

    # set up stuff for the slider
    alpha_slider_max = 1

    # Open frame window. For testing only.
    cv2.namedWindow('frame')
    cv2.createTrackbar('contrast', 'frame', 0, alpha_slider_max, nothing)

    while 1:  # while 1 for testing only
        contrast = cv2.getTrackbarPos('contrast', 'frame')
        # cap.set(15, contrast)
        ret, frame = cap.read()

        barcodes = pyzbar.decode(frame)

        output_dicts = []
        observer_info = {}
        cv2.imshow('frame', frame)

        for barcode in barcodes:
            # set up dictionary for the barcodes
            # Get coordinates for the bounding box of the barcode
            (x, y, w, h) = barcode.rect

            barcode_data = barcode.data.decode("utf-8").replace(' ', '')

            # Parse barcode location data
            output_dict = extract_floor_barcode_data(barcode_data)

            # Get the pose of the barcode
            observer_info, img = determine_pose(frame, barcode, output_dict)
            cv2.circle(img, barcode.polygon[0], 5,[255,0,0], 3)
            cv2.imshow("pose", img)

            if output_dict:
                # output_dict['frame_location'] = [(frame_x + w)/2, frame_y]
                output_dicts.append(output_dict)

        # world_fig = plt.figure()
        # ax = world_fig.add_subplot(1, 1, 1)
        # for d in output_dicts:
        #
        #     ax.scatter(d['location']['x'], d['location']['y'], marker="s")
        # if observer_info:
        #     ax.scatter(observer_info['location']['x'], observer_info['location']['y'])
        # if output_dicts:
        #     ax.scatter(0,0, marker="x")
        # plt.show()
        # plt.pause(1)
        # plt.close()
        # return output_dicts
        if cv2.waitKey(5) & 0xFF == ord('q'):
            cap.release()
            cv2.destroyAllWindows()
            exit(0)


def read_floor_qr(cap):
    """
    Parses QR Code data as formatted for the competition.
    :param vs: imutils.video.VideoStream object
    :param show_video: set to True to have a live stream pop up.
    :return: Array of QR code dictionaries.
    """
    ret, frame = cap.read()

    barcodes = pyzbar.decode(frame)
    output_dict = None

    if barcodes is not None and len(barcodes) == 1:
        print("I see barcode.")
        barcode = barcodes[0]
        barcode_data = barcode.data.decode("utf-8").replace(' ', '')

        # Parse barcode location data
        output_dict = extract_floor_barcode_data(barcode_data)
    # cv2.imshow('test', frame)
    # if cv2.waitKey(5) & 0xFF == ord('q'):
    #     cap.release()
    #     cv2.destroyAllWindows()
    #     exit(0)
    return output_dict



def determine_pose(image, barcode, barcode_info):
    """
    Procedure to get pose of the qr code:
    1. generate axis
    :param image:
    :return:
    """
    # Create an Axis with no transformation
    axis = np.float32([[3, 0,  0],
                       [0, 3,  0],
                       [0, 0, -3]]).reshape(-1, 3)

    # Convert the image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # get the corners of the qr code
    try:
        (lower_left, lower_right, upper_right, upper_left) = barcode.polygon
    except ValueError:
        return

    cv2.circle(image, upper_right, 5, (255, 0, 0), -1)

    corners = [[lower_left], [lower_right], [upper_right], [upper_left]]
    corners = np.float32(corners).reshape(-1, 2)
    # generate object points - Array of object points in the object coordinate space
    objp = np.float32([[0, 0, 0],
                       [0, 1, 0],
                       [1, 1, 0],
                       [1, 0, 0]])

    # Solve the transformation between object space and image space.
    ret, rvecs, tvecs = cv2.solvePnP(objp, corners, MTX, DIST)

    observer = {}
    observer['location'] = {}
    observer['location']['x'] = barcode_info['location']['x'] + tvecs[2] * 10
    print("Normal Distance: {}".format(tvecs[2]))
    observer['location']['y'] = barcode_info['location']['y'] + tvecs[0] * 10
    print("X translation: {}".format(tvecs[0]+3))

    # project the points to the image coordinate frame
    imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, MTX, DIST)
    img = draw(image, corners, imgpts)
    return observer, img


def draw(img, corners, imgpts):
    corner = tuple(corners[0].ravel())
    cv2.line(img, corner, tuple(imgpts[0].ravel()), (255, 0, 0), 5)
    cv2.line(img, corner, tuple(imgpts[1].ravel()), (0, 255, 0), 5)
    cv2.line(img, corner, tuple(imgpts[2].ravel()), (0, 0, 255), 5)
    return img


def set_motor_speed(serial_port, id, speed):
    # print("writing motor id: %d" %(id)) 
    motor_id_str = bytes([int(id)])
    serial_port.write(motor_id_str)
    
    # read_ser=serial_port.readline()
    # print("reading id:" + read_ser)
    
    # print("writing motor speed: %d" %speed)
    motor_speed_str = speed.to_bytes(1, byteorder='little', signed=True)
    # motor_speed_str = bytes([int(speed)])
    serial_port.write(motor_speed_str)

    # read_ser=serial_port.readline()
    # print("reading speed:" + read_ser)

    # print("------------")
    return


def nudge_right(serial_port):
    # move forward
    set_motor_speed(serial_port, 1, 100)
    set_motor_speed(serial_port, 2, 100)
    time.sleep(.05)

    # stop motors
    stop_motors(serial_port)

    return


def nudge_left(serial_port):
    # move backwards
    set_motor_speed(serial_port, 1, -100)
    set_motor_speed(serial_port, 2, -100)
    time.sleep(.05)

    # stop motors
    stop_motors(serial_port)

    return    


def drive_forward(serial_port):
    # print("driving forward")
    set_motor_speed(serial_port, 1, 50 + DRIVE_SPEED)
    set_motor_speed(serial_port, 2, 50 + DRIVE_SPEED)
    return


def drive_backward(serial_port):
    # print("driving forward")
    set_motor_speed(serial_port, 1, 50 - DRIVE_SPEED)
    set_motor_speed(serial_port, 2, 50 - DRIVE_SPEED)
    return


def stop_motors(serial_port):
    # print("stopping motors")
    set_motor_speed(serial_port, 1, 0)
    set_motor_speed(serial_port, 2, 0)


def turn_90_left(serial_port):
    # print("turning left")
    set_motor_speed(serial_port, 1, 50 - DRIVE_SPEED)
    set_motor_speed(serial_port, 2, 50 + DRIVE_SPEED)
    print("turning...")
    time.sleep(10)
    stop_motors(serial_port)


def turn_90_right(serial_port):
    # print("turning right")
    set_motor_speed(serial_port, 1, 50 + DRIVE_SPEED)
    set_motor_speed(serial_port, 2, 50 - DRIVE_SPEED)
    print("turning...")
    time.sleep(10)
    stop_motors(serial_port)


if __name__ == "__main__":
    # build_map()
    read_qr()
