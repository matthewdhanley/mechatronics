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


with np.load('state_machine/camera_cal_output.npz') as X:
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
    except ModuleNotFoundError:
        # if it's not running on the pi
        vs = VideoStream(src=0).start()
    time.sleep(2.0)
    return vs


def read_qr(vs, show_video=False):
    """
    Parses QR Code data as formatted for the competition.
    :param vs: imutils.video.VideoStream object
    :param show_video: set to True to have a live stream pop up.
    :return: Array of QR code dictionaries.
    """
    while 1:
        frame = vs.read()
        # frame = imutils.resize(frame, width=800)

        barcodes = pyzbar.decode(frame)

        output_dicts = []
        for barcode in barcodes:
            output_dict = {}
            (x, y, w, h) = barcode.rect
            print("x: {}, y: {}, w: {}, h: {}".format(x,y,w,h))
            # cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
            # cv2.arrowedLine(frame, a, b, (0, 0, 255), 3)  # x axis
            # cv2.line(frame, b, c, (0, 255, 0))
            # cv2.line(frame, c, d, (0, 255, 0))
            # cv2.arrowedLine(frame, a, d, (0, 255, 0), 3)  # y axis
            # x_unit = np.linalg.norm([b.x - a.x, b.y - a.y, 0])
            # y_unit = np.linalg.norm([d.x - a.x, d.y - a.y, 0])
            # z = np.cross(x_unit,y_unit)
            img = determine_pose(frame, barcode)
            cv2.imshow("pose", img)
            cv2.waitKey(5)
            barcode_data = barcode.data.decode("utf-8").replace(' ', '')
            # cv2.circle(frame, a, 5, (255, 0, 0), -1)
            # cv2.circle(frame, d, 5, (255, 0, 0), -1)
            # cv2.circle(frame, b, 5, (255, 0, 0), -1)
            barcode_type = barcode.type
            text = "{} ({})".format(barcode_data, barcode_type)
            # cv2.putText(image, text, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

            print("[INFO] found {} barcode: {}".format(barcode_type, barcode_data))

            # This might be formatted differently later?? - Lubna

            # Parse x,y location data
            location_data = re.match('.*Loc:(?P<x>\d+),(?P<y>\d+).*', barcode_data)
            if location_data:
                output_dict['location'] = {}
                output_dict['location']['x'] = int(location_data.group('x'))
                output_dict['location']['y'] = int(location_data.group('y'))

            # Parse pallet rack data
            pallet_rack_data = re.match('.*PRacks:?([\d+,]+).*', barcode_data)
            if pallet_rack_data:
                output_dict['pallet_racks'] = [int(x) for x in pallet_rack_data.groups()[0].split(',')]

            pallets_data = re.match('.*Pallets:([\d+,]+).*', barcode_data)
            if pallets_data:
                output_dict['pallets'] = [int(x) for x in pallets_data.groups()[0].split(',')]

            if output_dict:
                output_dict['time'] = time.time()
                output_dicts.append(output_dict)

    if show_video:
        cv2.imshow("Barcode Scanner", frame)
        cv2.waitKey(5)

    return output_dicts


def determine_pose(image, barcode):
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
    (lower_left, lower_right, upper_right, upper_left) = barcode.polygon

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

    # project the points to the image coordinate frame
    imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, MTX, DIST)
    print(rvecs)
    print(tvecs)
    img = draw(image, corners, imgpts)
    return img


def draw(img, corners, imgpts):
    corner = tuple(corners[0].ravel())
    cv2.line(img, corner, tuple(imgpts[0].ravel()), (255, 0, 0), 5)
    cv2.line(img, corner, tuple(imgpts[1].ravel()), (0, 255, 0), 5)
    cv2.line(img, corner, tuple(imgpts[2].ravel()), (0, 0, 255), 5)
    return img