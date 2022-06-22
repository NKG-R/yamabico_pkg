#!/usr/bin/env python
#import roslib; roslib.load_manifest('teleop_twist_joy')
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class JoyTwist(object):
    def __init__(self):
        self._joy_sub = rospy.Subscriber('/joy', Joy, self.joy_callback, queue_size=1)
        self._twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.max_speed  = rospy.get_param('~max_speed', 0.1)
        self.max_turn   = rospy.get_param('~max_turn', 0.1)

    def joy_callback(self, joy_msg):

        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0

        # Translational motion --------------------------
        twist.linear.x = joy_msg.axes[1] * self.max_speed

        # Rotational motion -----------------------------
        twist.angular.z = joy_msg.axes[3] * self.max_turn

        #print twist
        self._twist_pub.publish(twist)


if __name__=="__main__":

    rospy.init_node('teleop_twist_joy')
    joy_twist = JoyTwist()
    rospy.spin()
