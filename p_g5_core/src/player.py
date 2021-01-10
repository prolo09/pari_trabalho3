#!/usr/bin/env python

import rospy
import tf2_ros
import geometry_msgs
from geometry_msgs.msg import Twist
import math
import tf_conversions
global br
global t
global pub


def main():
    global br
    global t
    global pub

    rospy.init_node('player', anonymous=False)
    pub = rospy.Publisher('p_g5/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(50)  # 10hz


    try:

        while not rospy.is_shutdown():

            twist = Twist()
            twist.linear.x = 1.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = -1.0

            pub.publish(twist)

            rate.sleep()


    except:

            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0
            pub.publish(twist)

    finally:
            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0
            pub.publish(twist)


if __name__ == '__main__':
    main()
