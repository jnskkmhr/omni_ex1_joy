#!/usr/bin/env python3
# coding: utf-8

import rospy
from sensor_msgs.msg import Joy, JointState
from geometry_msgs.msg import Twist

#### Joint settings #####
JOINT_NAME = ['right_front_wheel_joint', 
              'right_rear_wheel_joint',
              'left_front_wheel_joint', 
              'left_rear_wheel_joint', 
              'right_front_steering_joint',
              'left_front_steering_joint', 
              ]
STEER_JOINT_INDEX = [4, 5]
DRIVE_JOINT_INDEX = [0, 1, 2, 3]
#########################


##### Joy setting ######
DRIVING_INDEX = 1
STEERING_INDEX = 3
MAX_STEERING_ANGLE = 0.785 # 45deg
MAX_VELOCITY = 0.5 #m/s
########################

class AckermannJoySteeringController:
    """
    Input topic: joy
    Output topic: joint_states(JointState), cmd_vel(Twist)
    """
    def __init__(self):
        self.sub = rospy.Subscriber('joy', Joy, self.joy_callback)
        self.steer_pub = rospy.Publisher('joint_states', JointState, queue_size=1)
        self.drive_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.joint_state = JointState()
        self.joint_state.name = JOINT_NAME
        self.joint_state.position = [0.0] * len(JOINT_NAME)
        self.joint_state.velocity = [0.0] * len(JOINT_NAME)
        self.joy_driving = 0.0
        self.joy_steer = 0.0
        self.twist = Twist()
    
    def joy_callback(self, msg):
        self.joint_state.position = [0.0] * len(JOINT_NAME)
        self.joint_state.velocity = [0.0] * len(JOINT_NAME)
        self.joy_driving = msg.axes[DRIVING_INDEX] # vel (m/s)
        self.joy_steer = msg.axes[STEERING_INDEX] # steer (rad)
        self.update_joint_pos(STEER_JOINT_INDEX, self._joy_to_steering(self.joy_steer))
        self.update_twist(self._joy_to_driving(self.joy_driving))
        self.steer_pub.publish(self.joint_state)
        self.drive_pub.publish(self.twist)
    
    def update_joint_pos(self, joint_index, value):
        for i in joint_index:
            self.joint_state.position[i] = value
    def update_joint_vel(self, joint_index, value):
        for i in joint_index:
            self.joint_state.velocity[i] = value
    def update_twist(self, linear):
        self.twist.linear.x = linear
        self.drive_pub.publish(self.twist)

    def _joy_to_driving(self, joy_driving):
        return MAX_VELOCITY * joy_driving
    
    ### Ackermann steering ###
    def _joy_to_steering(self, joy_steer):
        raise NotImplementedError
    ##########################

if __name__ == '__main__':
    rospy.init_node('ex1_ackermann_joy')
    node = AckermannJoySteeringController()
    rospy.spin()