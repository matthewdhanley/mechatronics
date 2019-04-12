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
import matplotlib.pyplot as plt
import os

with np.load(os.path.abspath(os.path.join(os.path.dirname(__file__), 'camera_cal_output.npz'))) as X:
    MTX, DIST, _, _ = [X[i] for i in ('mtx', 'dist', 'rvecs', 'tvecs')]


def get_camera():
    """
    Returns a VideoStream object. It will try to start a VideoStream using the picamera, and if it that is unsuccessful,
    it will start a video stream using the default camera.
    :return: VideoStream object.
    """
    try:
        # this will only run if it's on the pi
        vs = VideoStream(usePiCamera=True).start()

    except:
        # if it's not running on the pi
        vs = VideoStream(src=0).start()
    time.sleep(2.0)
    return vs


def extracct_barcode_data(data):
    output_dict = {}
    location_data = re.match('.*Loc:(?P<x>\d+),(?P<y>\d+).*', data)
    if location_data:
        output_dict['location'] = {}
        output_dict['location']['x'] = int(location_data.group('x'))
        output_dict['location']['y'] = int(location_data.group('y'))

    # Parse pallet rack data
    pallet_rack_data = re.match('.*PRacks:?([\d+,]+).*', data)
    if pallet_rack_data:
        output_dict['pallet_racks'] = [int(x) for x in pallet_rack_data.groups()[0].split(',')]

    pallets_data = re.match('.*Pallets:([\d+,]+).*', data)
    if pallets_data:
        output_dict['pallets'] = [int(x) for x in pallets_data.groups()[0].split(',')]

    output_dict['time'] = time.time()
    return output_dict


def read_qr(vs, show_video=False):
    """
    Parses QR Code data as formatted for the competition.
    :param vs: imutils.video.VideoStream object
    :param show_video: set to True to have a live stream pop up.
    :return: Array of QR code dictionaries.
    """

    # plt.show()

    while 1:
        frame = vs.read()
        # frame = imutils.resize(frame, width=800)

        barcodes = pyzbar.decode(frame)

        output_dicts = []
        observer_info = {}
        for barcode in barcodes:
            # set up dictionary for the barcodes
            # Get coordinates for the bounding box of the barcode
            (x, y, w, h) = barcode.rect
            # print("Bounding Box Coordinates:\n\tx: {}, y: {}, w: {}, h: {}".format(x, y, w, h))
            # cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
            # cv2.arrowedLine(frame, a, b, (0, 0, 255), 3)  # x axis
            # cv2.line(frame, b, c, (0, 255, 0))
            # cv2.line(frame, c, d, (0, 255, 0))
            # cv2.arrowedLine(frame, a, d, (0, 255, 0), 3)  # y axis
            # x_unit = np.linalg.norm([b.x - a.x, b.y - a.y, 0])
            # y_unit = np.linalg.norm([d.x - a.x, d.y - a.y, 0])
            # z = np.cross(x_unit,y_unit)

            barcode_data = barcode.data.decode("utf-8").replace(' ', '')
            barcode_type = barcode.type

            # print("[INFO] found {}:\n\tbarcode: {}".format(barcode_type, barcode_data))

            # Parse barcode location data
            output_dict = extracct_barcode_data(barcode_data)

            # Get the pose of the barcode
            observer_info, img = determine_pose(frame, barcode, output_dict)
            cv2.imshow("pose", img)
            cv2.waitKey(5)

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
    
    read_ser=serial_port.readline()
    # print("reading id:" + read_ser)
    
    # print("writing motor speed: %d" %speed)
    motor_speed_str = bytes([int(speed)])
    serial_port.write(motor_speed_str)

    read_ser=serial_port.readline()
    # print("reading speed:" + read_ser)

    # print("------------")
    return

def nudge_right(serial_port):

    # move forward
    set_motor_speed(serial_port, 1, 100)
    set_motor_speed(serial_port, 2, 100)
    time.sleep(.05)

    # stop motors
    set_motor_speed(serial_port, 1, 0)
    set_motor_speed(serial_port, 2, 0)

    return

def nudge_left(serial_port):

    # move backwards
    set_motor_speed(serial_port, 1, -100)
    set_motor_speed(serial_port, 2, -100)
    time.sleep(.05)

    # stop motors
    set_motor_speed(serial_port, 1, 0)
    set_motor_speed(serial_port, 2, 0)

    return    


if __name__ == "__main__":
    camera = get_camera()
    read_qr(camera, show_video=True)
