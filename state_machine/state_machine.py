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
import numpy as np

logging.basicConfig(level=logging.DEBUG)
logging.getLogger('transitions').setLevel(logging.INFO)


def robot_sm():
    """
    Creates a state machine with states defined in the 'states' array and transitions added manually
    :return: state machine object
    """
    # Different states the robot can be in. Add whatever we need -------------------------------------------------------
    states = ['startup', 'determine_target', 'safe', 'navigation', 'extract_pallet', 'drop_pallet']

    # The state machine is initialized with methods defined in the RobotActions class found below.
    robot = RobotActions()

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
    machine.add_transition('drop_off_pallet', source='*', dest='drop_pallet')
    machine.add_transition('extract_pallet', source='*', dest='extract_pallet')

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
    def __init__(self):
        """
        Constructor. This function runs when a new object is initialized.
        :return: None
        """
        self.timeout = 5000  # seconds
        self.vs = helpers.get_camera()  # get camera
        self.goal_qr = {}  # to store the goal QR code
        self.navigation_goal = {'x': None, 'y': None}  # where we want the robot to drive to
        self.current_location = {'x': 0, 'y': 0}  # where we currently are. Assuming we start at 0,0

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
            qr_codes = helpers.read_qr(self.vs, show_video=True)
            if qr_codes is not None and len(qr_codes) == 1:
                # If we see one QR Code, store it somehow
                self.goal_qr = qr_codes[0]
                self.navigation_goal = self.goal_qr['location']
                cv2.destroyAllWindows()
                self.queued_trigger = self.drive_to_pallet()
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
        print("Driving to goal x: {}, y: {}".format(self.navigation_goal['x'], self.navigation_goal['y']))
        while 1:
            # Figure out how we get to the goal here
            # TODO we will also need a way to differentiate whether we are driving to pick up or drop off. Likely just
            # another class variable.
            pass

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
