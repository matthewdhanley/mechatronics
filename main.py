"""
This is the main file for the robot.
Calling sequence: python main.py

This file imports the state_machine file found in the state_machine directory below this file.

In the main loop, the state machine is initialized. Then the queued_trigger is set to the "begin" trigger. A continuous
while loop is then entered. In this while loop, the "robot.queued_trigger()" trigger is run. Triggers are queued in the
RobotActions class in state_machine.py. There is more information in that file. Everytime a state returns, the while
loop continues to call the queued_trigger method again. This goes on...forever...
"""
import state_machine.state_machine as sm


def main():
    robot = sm.robot_sm()
    robot.queued_trigger = robot.begin()
    while 1:
        robot.queued_trigger()


if __name__ == "__main__":
    main()
