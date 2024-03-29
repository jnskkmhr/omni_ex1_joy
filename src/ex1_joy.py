#!/usr/bin/env python3
# coding: utf-8

import rospy
from sensor_msgs.msg import Joy, JointState

#### Joint settings #####
DRIVE_JOINT_NAME = ['right_front_wheel_joint', 
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

class JoySteeringController:
    """
    Input topic: joy
    Output topic: steering_command, drive_command(JointState)
    In future, we will use ackermann_msgs/AckermannDriveStamped
    We have two nodes: joy to ackermann and ackermann to joint state
    """
    def __init__(self):
        """
        Initialize the node.
        Subscriber: 
            joy (sensor_msgs/Joy)
        Publisher: 
            steering_command(sensor_msgs/JointState)
            drive_command(sensor_msgs/JointState)
        """
        self.sub = rospy.Subscriber('joy', Joy, self.joy_callback)
        self.steer_pub = rospy.Publisher('steering_command', JointState, queue_size=1)
        self.drive_pub = rospy.Publisher('drive_command', JointState, queue_size=1)
        self.steer_joint_state = JointState()
        self.drive_joint_state = JointState()

        self.steer_joint_state.name = STEER_JOINT_NAME
        self.steer_joint_state.position = [0.0] * len(STEER_JOINT_NAME)

        self.drive_joint_state.name = DRIVE_JOINT_NAME
        self.drive_joint_state.velocity = [0.0] * len(DRIVE_JOINT_NAME)
        self.joy_driving = 0.0
        self.joy_steer = 0.0
    
    def joy_callback(self, msg):
        """
        Callback function for joy subscriber.
        Args:
            msg (sensor_msgs/Joy): Joy message"""
        self.joy_driving = msg.axes[DRIVING_INDEX] # vel throttle
        self.joy_steer = msg.axes[STEERING_INDEX] # steer throttle
        self.update_joint_pos(self._joy_to_steering(self.joy_steer))
        self.update_joint_vel(self._joy_to_driving(self.joy_driving))
        self.steer_pub.publish(self.steer_joint_state)
        self.drive_pub.publish(self.drive_joint_state)
    
    def update_joint_pos(self, value):
        """
        Update the joint position.
        Args:
            value (float): joint position in rad"""
        for i in range(len(STEER_JOINT_NAME)):
            self.steer_joint_state.position[i] = value
        self.steer_joint_state.header.stamp = rospy.Time.now()

    def update_joint_vel(self, value):
        """
        Update the joint velocity.
        Args:
            value (float): joint velocity in rad/s"""
        for i in range(len(DRIVE_JOINT_NAME)):
            self.drive_joint_state.velocity[i] = value
        self.drive_joint_state.header.stamp = rospy.Time.now()

    def _joy_to_driving(self, joy_driving):
        """
        Convert joy_driving to joint velocity."""
        return VEL_TO_OMEGA * MAX_VELOCITY * joy_driving
    
    ### Naive mapping ###
    def _joy_to_steering(self, joy_steer):
        """
        Convert joy_steer to joint position."""
        return MAX_STEERING_ANGLE * joy_steer

if __name__ == '__main__':
    rospy.init_node('ex1_joy')
    node = JoySteeringController()
    rospy.spin()