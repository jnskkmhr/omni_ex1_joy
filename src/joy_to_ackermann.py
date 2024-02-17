#!/usr/bin/env python3
# coding: utf-8

import rospy
from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDriveStamped

##### Joy setting ######
DRIVING_INDEX = 1
STEERING_INDEX = 3
MAX_STEERING_ANGLE = 0.785 # 45deg
MAX_VELOCITY = 1.0 #m/s
########################

class JoyToAckermannCmd:
    """
    Input topic: joy
    Output topic: ackermann_cmd(ackermann_msgs/AckermannDriveStamped)
    """
    def __init__(self):
        """
        Initialize the node.
        Subscriber: 
            joy (sensor_msgs/Joy)
        Publisher: 
            ackermann_cmd(ackermann_msgs/AckermannDriveStamped)
        """
        self.sub = rospy.Subscriber('joy', Joy, self.joy_callback)
        self.pub = rospy.Publisher('ackermann_cmd', AckermannDriveStamped, queue_size=1)
        self.ackermann_cmd = AckermannDriveStamped()
    
    def joy_callback(self, msg):
        """
        Callback function for joy subscriber.
        """
        self.ackermann_cmd.drive.speed = MAX_VELOCITY * msg.axes[DRIVING_INDEX]
        self.ackermann_cmd.drive.steering_angle = MAX_STEERING_ANGLE * msg.axes[STEERING_INDEX]
        self.ackermann_cmd.header.stamp = rospy.Time.now()
        self.pub.publish(self.ackermann_cmd)

if __name__ == '__main__':
    rospy.init_node('joy_to_ackermann_cmd')
    joy_to_ackermann_cmd = JoyToAckermannCmd()
    rospy.spin()