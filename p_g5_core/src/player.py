#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist





def verOdomCall(msg):
    global p_x, p_y, theta
    p_x=msg.pose.pose.position.x
    p_y = msg.pose.pose.position.y


def moveTo():
    global publisher

    possicao= Twist()



def main():


    rospy.init_node('palyer',anonymous=False)
    #ver a possicao mmas nao e muito importante aqui neste exercicio
    rospy.Subscriber("p_g5/odom", Odometry, verOdomCall)

    publisher = rospy.Publisher("/p_g5/cmd_vel", Twist, queue_size=10)
    rate=rospy.Rate(1)
    possicao = Twist()
    possicao.linear.x=0.0
    possicao.linear.y = 0.0
    possicao.linear.z = 0.0
    possicao.angular.x = 0.0
    possicao.angular.y = 0.0
    possicao.angular.z = 0.0

    while not rospy.is_shutdown():
        moveTo()

    publisher.publish(possicao)
    rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    main()