"""
This file holds functions that may be helpful in the state machine. It's purpose is to keep the state_machine file
as clean as possible.
"""
import cv2
import imutils
from pyzbar import pyzbar
from imutils.video import VideoStream
import time
import re


def get_camera():
    """
    Returns a VideoStream object. It will try to start a VideoStream using the picamera, and if it that is unsuccessful,
    it will start a video stream using the default camera.
    :return: VideoStream object.
    """
    # try:
        # this will only run if it's on the pi
    vs = VideoStream(usePiCamera=True).start()
    # except:
        # if it's not running on the pi
        # vs = VideoStream(src=0).start()
    time.sleep(2.0)
    return vs


def read_qr(vs, show_video=False):
    """
    Parses QR Code data as formatted for the competition.
    :param vs: imutils.video.VideoStream object
    :param show_video: set to True to have a live stream pop up.
    :return: Array of QR code dictionaries.
    """
    frame = vs.read()
    frame = imutils.resize(frame, width=400)

    barcodes = pyzbar.decode(frame)

    output_dicts = []
    for barcode in barcodes:
        output_dict = {}
        (frame_x, frame_y, w, h) = barcode.rect
        cv2.rectangle(frame, (frame_x, frame_y), (frame_x + w, frame_y + h), (0, 0, 255), 2)

        # Locates corner for QR code:
        # we can use this to center robot on pallet
        cv2.rectangle(frame, (frame_x, frame_y), (frame_x + 2, frame_y + 2), (255, 0, 0), 2)
        barcode_data = barcode.data.decode("utf-8").replace(' ', '')
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
            output_dict['frame_location'] = [(frame_x + w)/2, frame_y]
            output_dicts.append(output_dict)

    if show_video:
        cv2.imshow("Barcode Scanner", frame)
        cv2.waitKey(5)

    return output_dicts

def set_motor_speed(serial_port, id, speed):

    # Motor 1: Left
    # Motor 2: Right
    # Motor 3: Horizontal
    # Motor 4: Vertical

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
    # Used for aligning robot with QR code

    # move forward
    set_motor_speed(serial_port, 1, 50)
    set_motor_speed(serial_port, 2, 50)
    time.sleep(.05)

    # stop motors
    set_motor_speed(serial_port, 1, 0)
    set_motor_speed(serial_port, 2, 0)

    return

def nudge_left(serial_port):
    # Used for aligning robot with QR code

    # move backwards
    set_motor_speed(serial_port, 1, -50)
    set_motor_speed(serial_port, 2, -50)
    time.sleep(.05)

    # stop motors
    set_motor_speed(serial_port, 1, 0)
    set_motor_speed(serial_port, 2, 0)

    return  

def lower_one_row(serial_port):
    row_steps = 100
    set_motor_speed(serial_port, 4, -row_steps)
    return

def rise_one_row(serial_port):
    set_motor_speed(serial_port, 4, row_steps)
    return