"""
Implementing a state machine from the transitions python package
Documentation is available here: https://github.com/pytransitions/transitions

Basically what I'm doing here is setting up a state machine. Functions referenced with helpers.some_name() are in the
helpers.py file in this directory.
"""
from transitions import Machine
import time
import logging
import cv2

import helpers as helpers
import path_planner as path_planner
import numpy as np
# from state_machine import helpers
# import state_machine.helpers
import serial

logging.basicConfig(level=logging.DEBUG)
logging.getLogger('transitions').setLevel(logging.INFO)

QR_MOTOR_SPEED = -20
TRAVEL_MOTOR_SPEED = -50
TURN_TIME = 7  # seconds


def robot_sm(test=False):
    """
    Creates a state machine with states defined in the 'states' array and transitions added manually
    :return: state machine object
    """
    # Different states the robot can be in. Add whatever we need -------------------------------------------------------
    states = ['startup', 'determine_target', 'safe', 'navigation', 'align_pallet', 'pickup_pallet', 'align_rack']

    # The state machine is initialized with methods defined in the RobotActions class found below.
    robot = RobotActions(test=test)

    # Initialize the state machine
    machine = Machine(model=robot, states=states, initial='startup', ignore_invalid_triggers=True)

    # Possible transitions go here -------------------------------------------------------------------------------------
    # The syntax is 'some_trigger', source='current_state', dest='desired_state'
    # so if we called robot.begin(), the below line says that if we're in the 'startup' state, the trigger will send us
    # to the 'determine_target' state.
    machine.add_transition('begin', source='startup', dest='determine_target')
    machine.add_transition('goto_safe', source='*', dest='safe')
    machine.add_transition('drive_to_pallet', source='*', dest='navigation')
    machine.add_transition('drive_to_dropoff', source='*', dest='navigation')
    machine.add_transition('pickup', source='*', dest='pickup_pallet')
    machine.add_transition('align', source='*', dest='align_pallet')
    machine.add_transition('align_rack', source='*', dest='align_rack')

    return robot


class RobotActions(object):
    """
    This class is what the state machine is built upon. Define any attributes in the __init__ constructor.
    If in a class method and want to change states, set self.queued_trigger to the desired trigger method.
    For example, if I'm in the on_enter_determine_target() method and want to go to the safe state, I would set the
    queued_trigger to self.goto_safe() then return from that method. You can see this implemented in that method below.

    Note that the state machine is an extension of this class. Thus, the state machine itself has all the methods below
    and more that are added when rune the machine = Machine(model=robot, ... ) code in the robot_sm() function above.
    This means that we can call "state machine" functions from within this class. I.e. if I wanted to access the current
    state, I could get it with self.state
    """
    def __init__(self, test=False):
        """
        Constructor. This function runs when a new object is initialized.
        :return: None
        """
        self.timeout = 5000  # seconds
        self.goal_qr = {}  # to store the goal QR code
        self.intermediate_goal_loc = np.array([0, 0])  # where we want the robot to drive to
        self.current_location = np.array([0, 0])  # where we currently are. Assuming we start at 0,0
        self.nav_thresh = 2  # threshold in inches for how close we need to be to our target.
        self.drive_timeout = 300  # how long before we give up on driving.
        self.path = None
        self.racks = path_planner.get_racks()
        # Set up serial interface with Arduino
        self.test = test
        self.G = path_planner.get_graph()
        if not self.test:
            self.serial_nav = serial.Serial("/dev/ttyUSB1", 9600, timeout=1)  #change ACM number as found from ls /dev/tty/ACM*
            self.serial_nav.baudrate =9600
            time.sleep(2)
            self.serial_grip = serial.Serial("/dev/ttyUSB0", 9600, timeout=1)  #change ACM number as found from ls /dev/tty/ACM*
            self.serial_grip.baudrate =9600
            time.sleep(2)
        else:
            self.serial_nav = serial.Serial("COM2", 9600,
                                            timeout=1)  # change ACM number as found from ls /dev/tty/ACM*
            self.serial_nav.baudrate = 9600

            self.serial_grip = serial.Serial("COM4", 9600,
                                             timeout=1)  # change ACM number as found from ls /dev/tty/ACM*
            self.serial_grip.baudrate = 9600

        self.direction = np.array([1, 0])

    def __del__(self):
        """
        Destructor. This funtion runs when the object is destroyed.
        :return: None
        """
        cv2.destroyAllWindows()

    def update_path(self, current_node, target_node):
        print("Target Node: {}, Current Node: {}".format(target_node, current_node))
        self.G = path_planner.get_graph()
        path_planner.dijkstra(self.G, self.G.get_vertex(current_node))
        target = self.G.get_vertex(target_node)
        path = [target_node]
        path_planner.shortest(target, path)
        return path[::-1]

    def queued_trigger(self):
        """
        This method is intended to be overwritten when changing states.
        """
        raise NotImplementedError

    # ----------- STATE MACHINE CALLBACKS ---------------------
    '''
    When a state is entered, the corresponding on_enter_<state_name> function will be called. So if we change states
    to 'navigation', the on_enter_navigation method will start to execute. There's also on_exit_<state_name> as well as
    many others (see transitions documentation, link at the top of this file)
    '''
    def on_enter_startup(self):
        """
        This should never really be called. UNLESS the state machine is reset using the self.reset() method. Then we
        will need to add something here.
        :return:
        """
        raise NotImplementedError

    def on_enter_determine_target(self):
        """
        This function waits until a valid "Goal" QR Code is decoded. Then it transitions to the "navigation" state.
        Procedure:
            1. Try to read QR Code
            2. If it's decoded and correct, set the qr_goal, navigation goal, and queued_trigger. Close all cv2 windows and
               return to initiate transition to navigation state
            3. Check if time has exceeded the timeout value (set in constructor). If it has, set the queued_trigger to safe
               and return to initiate the transition to that state.
        :return: Nothing. Returning initiates self.queued_trigger
        """
        print("Waiting for Initial QR Code.")
        begin_time = int(time.time())  # time the loop started
        while 1:
            self.goal_qr = helpers.read_goal_qr(self.racks)

            if self.goal_qr is not None:  # in reality, it should never be None based on the nature of the loop
                # If we see one QR Code, store it somehow
                self.target = self.goal_qr['rack'].qr1
                self.path = self.update_path('qr1', self.goal_qr['rack'].qr1)
                self.queued_trigger = self.drive_to_pallet()
                # self.queued_trigger = self.align()
                return

            # Check for Timeout
            if int(time.time()) - begin_time > self.timeout:
                self.queued_trigger = self.goto_safe()
                return

    def on_enter_navigation(self):
        """
        Somehow figure out how to drive to the goal.
        :return: Nothing. Returning initiates trigger self.queued_trigger
        """
        time.sleep(3)
        # ROTATION MATRICES
        left_rotation = np.array([[np.cos(np.pi/2), -np.sin(np.pi/2)], [np.sin(np.pi/2), np.cos(np.pi/2)]])
        right_rotation = np.array([[np.cos(-np.pi/2), -np.sin(-np.pi/2)], [np.sin(-np.pi/2), np.cos(-np.pi/2)]])

        # Get webcam stream
        try:
            cap = cv2.VideoCapture(0)  # turn on webcam
            time.sleep(0.5)
            ret, _ = cap.read()
            assert ret
        except AssertionError:
            cap = cv2.VideoCapture(1)  # turn on webcam
            time.sleep(0.5)
            ret, _ = cap.read()
            assert ret

        # store the last update time.
        last_location_update = time.time()

        # create an iterator for the path. It's like a for loop that can be passed around.
        path_iterator = iter(self.path)

        # the next intermediate goal is the first element in the path.
        intermediate_goal = next(path_iterator)
        print("New goal: {}".format(intermediate_goal))

        # Grab and store the location of the intermediate goal
        intermediate_goal_loc = self.G.get_vertex(intermediate_goal).get_location()
        self.intermediate_goal_loc[0] = intermediate_goal_loc[0]
        self.intermediate_goal_loc[1] = intermediate_goal_loc[1]

        # variable to hold last string that was printed to the screen
        direction = ''

        location = None
        helpers.drive_forward(self.serial_nav, QR_MOTOR_SPEED)
        while location is None:
            ret, frame = cap.read()
            if ret:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            else:
                exit(1)
                # check to see if there could be a qr code in the frame

            # try to extract info from the QR code
            location = helpers.read_floor_qr(frame)

            if location is not None:
                self.current_location[0] = location['location']['x']
                self.current_location[1] = location['location']['y']
                new_node = self.G.get_nearest(self.current_location)
                print("Updated location. Current node: {}".format(new_node))

                if new_node == 'qr1':
                    self.direction = np.array([0, 1])
                    print("Updated Direction")
                elif new_node == 'qr2':
                    self.direction = np.array([1, 0])
                    print("Updated Direction")
                elif new_node == 'qr14':
                    self.direction = np.array([0, -1])
                    print("Updated Direction")
                elif new_node == 'qr15':
                    self.direction = np.array([1, 0])
                    print("Updated Direction")
                elif new_node == 'qr9':
                    self.direction = np.array([0, 1])
                    print("Updated Direction")
                elif new_node == 'qr8':
                    print("Updated Direction")
                    self.direction = np.array([0, -1])
                else:
                    print("Cannot determine direction.")
                    location = None
                    continue

                print("Updated Path:")
                # target1 = racks[self.goal_qr.rack].qr1
                # path1 = self.update_path(new_node, target1)
                # target2 = racks[self.goal_qr.rack].qr2
                # path2 = self.update_path(new_node, target2)
                # Compare path lengths
                # Choose shorter path and record choice (qr1 or qr2)
                # self.target = self.goal_qr.rack.qr1
                self.path = self.update_path(new_node, self.target)
                print(self.path)
                path_iterator = iter(self.path)
                intermediate_goal = next(path_iterator)
                intermediate_goal_loc = self.G.get_vertex(intermediate_goal).get_location()
                self.intermediate_goal_loc[0] = intermediate_goal_loc[0]
                self.intermediate_goal_loc[1] = intermediate_goal_loc[1]
                last_location_update = time.time()

        # Continuous while loop.
        location_prev = location
        while 1:
            # grab an image from the webcam
            ret, frame = cap.read()
            if ret:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            else:
                exit(1)

            # check to see if there could be a qr code in the frame
            white = helpers.check_for_white(frame)

            if white:
                # Slow the robot down so it has a chance at reading the QR code
                motor_speed = QR_MOTOR_SPEED

                # try to extract info from the QR code

                location = helpers.read_floor_qr(frame)

                if location is not None and location != location_prev:
                    location_prev = location
                    self.current_location[0] = location['location']['x']
                    self.current_location[1] = location['location']['y']
                    print(self.current_location)
                    new_node = self.G.get_nearest(self.current_location)
                    print("Updated location. Current node: {}".format(new_node))
                    print("New Path:")
                    self.path = self.update_path(new_node, self.target)
                    print(self.path)
                    path_iterator = iter(self.path)
                    intermediate_goal = next(path_iterator)
                    print("New goal: {}".format(intermediate_goal))
                    intermediate_goal_loc = self.G.get_vertex(intermediate_goal).get_location()
                    self.intermediate_goal_loc[0] = intermediate_goal_loc[0]
                    self.intermediate_goal_loc[1] = intermediate_goal_loc[1]
                    print(self.intermediate_goal_loc)
                    last_location_update = time.time()

            else:
                # speed the robot up to get to the next location
                motor_speed = TRAVEL_MOTOR_SPEED

            # check for timeout error
            if time.time() - last_location_update > self.drive_timeout:
                print("Navigation timeout.")
                self.queued_trigger = self.goto_safe()

            # this logic will turn us when we are aligned with the target in the current direction.
            diff = self.intermediate_goal_loc - self.current_location
            # print(self.current_location)
            # print(self.intermediate_goal_loc)
            # print(diff)

            # check if we have reached our goal
            if np.max(np.abs(diff)) <= self.nav_thresh:
                try:
                    intermediate_goal = next(path_iterator)
                    intermediate_goal_loc = self.G.get_vertex(intermediate_goal).get_location()
                    self.intermediate_goal_loc[0] = intermediate_goal_loc[0]
                    self.intermediate_goal_loc[1] = intermediate_goal_loc[1]
                    diff = self.intermediate_goal_loc - self.current_location
                    print("New intermediate goal: {}".format(intermediate_goal))
                except StopIteration:
                    print("I did it!")
                    self.queued_trigger = self.align_rack()
                    return

            if np.dot(diff, self.direction) > self.nav_thresh:
                if direction != 'forward':
                    direction = 'forward'
                    print("going forward")
                helpers.drive_forward(self.serial_nav, motor_speed)

            elif np.dot(diff, np.inner(left_rotation, self.direction).round()) > 0:
                if direction != 'left':
                    direction = 'left'
                    print("going left")
                time.sleep(TURN_TIME)
                helpers.turn_90_left(self.serial_nav)
                self.direction = np.inner(left_rotation, self.direction).round()

            elif np.dot(diff, np.inner(right_rotation, self.direction).round()) > 0:
                if direction != 'right':
                    direction = 'right'
                    print("going right")
                time.sleep(TURN_TIME)
                helpers.turn_90_right(self.serial_nav)
                self.direction = np.inner(right_rotation, self.direction).round()

            else:
                helpers.drive_backward(self.serial_nav, motor_speed)
                # print("not doing anything")
                
    def on_enter_align_rack(self):
        print("Aligning to rack")
        rack_dir = racks[self.goal_qr.rack].direction
        while(1):
            if np.dot(rack_dir, self.direction) > self.nav_thresh:
                if direction != 'forward':
                    direction = 'forward'
                    print("going forward")
                helpers.drive_forward(self.serial_nav, motor_speed)
                break

            elif np.dot(rack_dir, np.inner(left_rotation, self.direction).round()) > 0:
                if direction != 'left':
                    direction = 'left'
                    print("going left")
                helpers.turn_90_left(self.serial_nav)
                self.direction = np.inner(left_rotation, self.direction).round()

            elif np.dot(diff, np.inner(right_rotation, self.direction).round()) > 0:
                if direction != 'right':
                    direction = 'right'
                    print("going right")
                helpers.turn_90_right(self.serial_nav)
                self.direction = np.inner(right_rotation, self.direction).round()

            else:
                print("turning around")
                helpers.turn_90_right(self.serial_nav)
                self.direction = np.inner(right_rotation, self.direction).round()
        # align height
        while(1):
            pallet_qr = helpers.read_pallet_qr()
            if pallet_qr.pallet == self.goal_qr.pallet:
                print("right pallet")
                break
            print("wrong pallet")
        self.queued_trigger = self.align()
        return
    
    def on_enter_align_pallet(self):
        """
        align robot with pallet
        command gripper to pick it up
        """
        print("Aligning Robot")
        vs = helpers.get_camera()
        # # x val neeeded to be centered
        x_center = 103
        while 1:
            qr_codes = helpers.read_qr(vs)
            if qr_codes is not None and len(qr_codes) == 1:
                print(qr_codes[0]['frame_location'][0])
                x_error = qr_codes[0]['frame_location'][0] - x_center
                print(x_error)

                if x_error > 3:
                    print("nudge right")
                    helpers.nudge_right(self.serial_nav)

                if x_error < -3:
                    print("nudge left")
                    helpers.nudge_left(self.serial_nav)
                
                if abs(x_error) <=2:
                    print("Centered")
                    self.queued_trigger = self.pickup()

                time.sleep(2)

    def on_enter_pickup_pallet(self):
        """
        align robot with pallet
        command gripper to pick it up
        """
        print("Picking Up Pallet")

        # horizontal actuator
        # +100 = forward
        time.sleep(5)
        print("Forward")
        helpers.set_motor_speed(self.serial_grip, 3, 450)


        time.sleep(60)

        # time.sleep(5)
        # # veritcal actuator
        # # +100 = upwards
        # print("Up")
        # helpers.set_motor_speed(self.serial_grip, 4, 8000)
        # time.sleep(15)

        # print("Back")
        # helpers.set_motor_speed(self.serial_grip, 3, -450)
        # time.sleep(5)

        # print("Down")
        # helpers.set_motor_speed(self.serial_grip, 4, -8000)
        # time.sleep(15)

        # horizontal actuator
        # -100 = backwards

    def on_enter_safe(self):
        """
        Performs actions to make the robot not kill anyone or itself. This state is reserved for keeping the robot
        and people around it healthy
        :return:
        """
        print(self.state)
        # Do whatever we need to do to make the robot stop here.
        helpers.stop_motors(self.serial_nav)
        helpers.stop_motors(self.serial_grip)
        raise TimeoutError("The robot has been safed.")

    def on_enter_extract_pallet(self):
        """
        TODO
        :return:
        """
        raise NotImplementedError

    def on_enter_drop_pallet(self):
        """
        TODO
        :return:
        """
        raise NotImplementedError

