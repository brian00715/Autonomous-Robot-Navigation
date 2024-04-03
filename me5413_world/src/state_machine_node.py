#!/usr/bin/env python

import rospy
from std_msgs.msg import String

class State:
    def __init__(self, robot):
        self.robot = robot

    def execute(self):
        pass

class IdleState(State):
    def execute(self):
        rospy.loginfo("机器人处于待机状态")

class MoveState(State):
    def execute(self):
        rospy.loginfo("机器人正在移动")

class StopState(State):
    def execute(self):
        rospy.loginfo("机器人停止移动")

class Robot:
    def __init__(self):
        self.idle_state = IdleState(self)
        self.move_state = MoveState(self)
        self.stop_state = StopState(self)
        self.current_state = self.idle_state

    def set_state(self, state):
        self.current_state = state

    def execute_state(self):
        self.current_state.execute()

def state_callback(data):
    global robot
    if data.data == "MOVE":
        robot.set_state(robot.move_state)
    elif data.data == "STOP":
        robot.set_state(robot.stop_state)
    elif data.data == "IDLE":
        robot.set_state(robot.idle_state)



if __name__ == '__main__':
    global robot
    rospy.init_node('robot_state_machine', anonymous=True)
    robot = Robot()
    rospy.Subscriber("robot_state", String, state_callback)
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        robot.execute_state()
        rate.sleep()