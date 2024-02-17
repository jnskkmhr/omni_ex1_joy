#!/usr/bin/env python3
# coding: utf-8

import rospy
from sensor_msgs.msg import JointState
from ackermann_msgs.msg import AckermannDriveStamped
import math

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

##### robot configuration #####
RADIUS = 0.09 #0.09m
WIDTH = 0.453 # distance between left and right wheel
LENGTH = 0.673 # distance between revolution axis
GAMMA = 1.0 # ackermann ratio

class AckermannSteeringController:
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
            ackermann_cmd (ackermann_msgs/AckermannDriveStamped)
        Publisher: 
            steering_command(sensor_msgs/JointState)
            drive_command(sensor_msgs/JointState)
        """
        self.sub = rospy.Subscriber('ackermann_cmd', AckermannDriveStamped, self.joy_callback)
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
        Callback function for ackermann_cmd subscriber.
        Args:
            msg (ackermann_msgs/AckermannDriveStamped): Ackermann drive command"""
        self.update_joint_pos(self.steer_to_steer_ack(msg.steering_angle))
        self.update_joint_vel(self.vel_to_omega(msg.speed))
        self.steer_pub.publish(self.steer_joint_state)
        self.drive_pub.publish(self.drive_joint_state)
    
    def update_joint_pos(self, steer_ack):
        """
        Update the joint position (steering angle) based on ackermann steering.
        Args:
            steer_ack (float): command_steering/ackermann_ratio"""
        steer_left = math.atan(WIDTH*math.tan(steer_ack)/(WIDTH + 0.5*LENGTH*math.tan(steer_ack)))
        steer_right = math.atan(WIDTH*math.tan(steer_ack)/(WIDTH - 0.5*LENGTH*math.tan(steer_ack)))
        self.steer_joint_state.position[0] = steer_left
        self.steer_joint_state.position[1] = steer_right
        self.steer_joint_state.header.stamp = rospy.Time.now()

    def update_joint_vel(self, velocity):
        """
        Update the joint velocity.
        Args:
            value (float): joint velocity in rad/s"""
        for i in range(len(DRIVE_JOINT_NAME)):
            self.drive_joint_state.velocity[i] = velocity
        self.drive_joint_state.header.stamp = rospy.Time.now()
        
    @staticmethod
    def vel_to_omega(vel):
        """
        Convert linear velocity to angular velocity.
        Args:
            vel (float): linear velocity in m/s
        Returns:
            float: angular velocity in rad/s"""
        return vel / RADIUS
    
    @staticmethod
    def steer_to_steer_ack(steer):
        """
        Convert steering angle to ackermann steering.
        Args:
            steer (float): steering angle in rad
        Returns:
            float: ackermann steering"""
        return steer/GAMMA

if __name__ == '__main__':
    rospy.init_node('ackermann_steering')
    node = AckermannSteeringController()
    rospy.spin()