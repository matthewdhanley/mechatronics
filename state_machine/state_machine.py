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
from state_machine import helpers
from state_machine import path_planner
import numpy as np
# from state_machine import helpers
# import state_machine.helpers
import serial

logging.basicConfig(level=logging.DEBUG)
logging.getLogger('transitions').setLevel(logging.INFO)


def robot_sm(test=False):
    """
    Creates a state machine with states defined in the 'states' array and transitions added manually
    :return: state machine object
    """
    # Different states the robot can be in. Add whatever we need -------------------------------------------------------
    states = ['startup', 'determine_target', 'safe', 'navigation', 'align_pallet', 'pickup_pallet']

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
        self.vs = helpers.get_camera()  # get camera
        self.goal_qr = {}  # to store the goal QR code
        self.navigation_goal = np.array([0, 0])  # where we want the robot to drive to
        self.current_location = np.array([0, 0])  # where we currently are. Assuming we start at 0,0
        self.nav_thresh = 5  # threshold in inches for how close we need to be to our target.
        self.drive_timeout = 30  # how long before we give up on driving.
        self.path = None
        # Set up serial interface with Arduino
        self.test = test
        self.G = path_planner.get_graph()
        if not self.test:
            self.serial_nav = serial.Serial("/dev/ttyUSB1", 9600, timeout=1)  #change ACM number as found from ls /dev/tty/ACM*
            self.serial_nav.baudrate =9600

            self.serial_grip = serial.Serial("/dev/ttyUSB0", 9600, timeout=1)  #change ACM number as found from ls /dev/tty/ACM*
            self.serial_grip.baudrate =9600
        else:
            self.serial_nav = serial.Serial("COM2", 9600,
                                            timeout=1)  # change ACM number as found from ls /dev/tty/ACM*
            self.serial_nav.baudrate = 9600

            self.serial_grip = serial.Serial("COM4", 9600,
                                             timeout=1)  # change ACM number as found from ls /dev/tty/ACM*
            self.serial_grip.baudrate = 9600

        self.direction = np.array([1, 0])  # assuming that we are starting faced in the positive X direction.

    def __del__(self):
        """
        Destructor. This funtion runs when the object is destroyed.
        :return: None
        """
        self.vs.stop()
        cv2.destroyAllWindows()

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
            self.goal_qr = helpers.read_goal_qr()

            if self.goal_qr is not None:  # in reality, it should never be None based on the nature of the loop
                # If we see one QR Code, store it somehow
                self.navigation_goal[0] = self.goal_qr['goal']['x']
                self.navigation_goal[1] = self.goal_qr['goal']['y']
                path_planner.dijkstra(self.G, self.G.get_vertex('qr1'))
                target = self.G.get_vertex('qr16')
                path = [target.get_id()]
                path_planner.shortest(target, path)
                self.path = path[::-1]
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
        print("Driving to goal x: {}, y: {}".format(self.navigation_goal[0], self.navigation_goal[1]))
        right_rotation = np.array([[np.cos(np.pi/2), -np.sin(np.pi/2)], [np.sin(np.pi/2), np.cos(np.pi/2)]])
        left_rotation = np.array([[np.cos(-np.pi/2), -np.sin(-np.pi/2)], [np.sin(-np.pi/2), np.cos(-np.pi/2)]])
        cap = cv2.VideoCapture(1)  # turn on webcam
        last_location_update = time.time()
        path_iterator = iter(self.path)
        goal1 = next(path_iterator)
        goal_loc = self.G.get_vertex(goal1).get_location()
        self.navigation_goal['location']['x']
        while 1:
            location = helpers.read_floor_qr(cap)
            if location is not None:
                print("updated location.")
                last_location_update = time.time()
                self.current_location[0] = location['location']['x']
                self.current_location[1] = location['location']['y']

            if time.time() - last_location_update > self.drive_timeout:
                print("I'm scared and lonely and haven't figured out where I am in a long time.")
                self.queued_trigger = self.goto_safe()

            # this logic will turn us when we are aligned with the target in the current direction.
            diff = self.navigation_goal - self.current_location

            if np.max(diff) <= self.nav_thresh:
                print("I did it!")
                self.queued_trigger = self.align()
                return

            if np.dot(diff, self.direction) > 0:
                helpers.drive_forward(self.serial_nav)

            elif np.dot(diff, np.inner(left_rotation, self.direction).round()) > 0:
                helpers.turn_90_left(self.serial_nav)
                self.direction = np.inner(left_rotation, self.direction).round()

            elif np.dot(diff, np.inner(right_rotation, self.direction).round()) > 0:
                helpers.turn_90_right(self.serial_nav)
                self.direction = np.inner(right_rotation, self.direction).round()

            else:
                print("I'm confused.")
                self.queued_trigger = self.goto_safe()
                return


    def on_enter_align_pallet(self):
        """
        align robot with pallet
        command gripper to pick it up
        """
        print("Aligning Robot")

        # # x val neeeded to be centered
        x_center = 103
        while 1:
            qr_codes = helpers.read_qr(self.vs, show_video=True)

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
        raise TimeoutError("The timeout limit was reached and the robot has been safed.")

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

