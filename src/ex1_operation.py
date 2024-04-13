#!/usr/bin/env python3
# coding: utf-8

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32

#### Joint settings #####
DRIVE_JOINT_NAME = [
              'right_front_wheel_joint', 
              'right_rear_wheel_joint',
              'left_front_wheel_joint', 
              'left_rear_wheel_joint',
              ]

STEER_JOINT_NAME = [
              'right_front_steering_joint',
              'left_front_steering_joint', 
              ]
#########################


##### Joy setting ######
DRIVING_INDEX = 1
STEERING_INDEX = 3
MAX_STEERING_ANGLE = 0.785 # 45deg
MAX_VELOCITY = 1.0 #m/s
RADIUS = 0.09 #90mm
VEL_TO_OMEGA = 1/RADIUS
########################

class DriveSteeringController:
    """
    Input topic: joy
    Output topic: steering_command, drive_command(JointState)
    In future, we will use ackermann_msgs/AckermannDriveStamped
    We have two nodes: joy to ackermann and ackermann to joint state
    """
    def __init__(self):
        self.drive_sub = rospy.Subscriber('/drive', Float32, self.drive_callback)
        self.steer_sub = rospy.Subscriber('/steer', Float32, self.steer_callback)
        self.steer_pub = rospy.Publisher('steering_command', JointState, queue_size=10)
        self.drive_pub = rospy.Publisher('drive_command', JointState, queue_size=10)
        self.steer_joint_state = JointState()
        self.drive_joint_state = JointState()

        self.steer_joint_state.name = STEER_JOINT_NAME
        self.steer_joint_state.position = [0.0] * len(STEER_JOINT_NAME)

        self.drive_joint_state.name = DRIVE_JOINT_NAME
        self.drive_joint_state.velocity = [0.0] * len(DRIVE_JOINT_NAME)
        self.joy_driving = 0.0
        self.joy_steer = 0.0
    
    def drive_callback(self, msg):
        self.joy_driving = msg.data
        self.update_joint_vel(self._joy_to_driving(self.joy_driving))
        self.drive_pub.publish(self.drive_joint_state)

    def steer_callback(self, msg):
        self.joy_steer = msg.data
        self.update_joint_pos(self._joy_to_steering(self.joy_steer))
        self.steer_pub.publish(self.steer_joint_state)
    
    def update_joint_pos(self, value):
        for i in range(len(STEER_JOINT_NAME)):
            self.steer_joint_state.position[i] = value
        self.steer_joint_state.header.stamp = rospy.Time.now()

    def update_joint_vel(self, value):
        for i in range(len(DRIVE_JOINT_NAME)):
            self.drive_joint_state.velocity[i] = value
        self.drive_joint_state.header.stamp = rospy.Time.now()

    def _joy_to_driving(self, joy_driving):
        return VEL_TO_OMEGA * MAX_VELOCITY * joy_driving
    
    ### Naive mapping ###
    def _joy_to_steering(self, joy_steer):
        return MAX_STEERING_ANGLE * joy_steer

if __name__ == '__main__':
    rospy.init_node('ex1_joy')
    r = rospy.Rate(20)
    node = DriveSteeringController()
    rospy.spin()