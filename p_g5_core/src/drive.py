#!/usr/bin/env python
import math

import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist


def moveT():
    global publisher,poseGoal, goalGlobal
    goalGlobal.header.stamp=rospy.Time.now()
    goal_baselink=poseGoal.transform(goalGlobal, "/p_g5/base_link", rospy.Duration(1))

#
#____________

    distancia=math.sqrt(goal_baselink.pose.possition.y**2+goal_baselink.pose.possition.x**2 )



    print(distancia)
    trageto=Twist()


#fazendo com o prof disse nao presisa de saber onde estou pola mesaguem odometria
# def verOdomCall(msg):
#     global p_x, p_y, theta
#     p_x = msg.pose.pose.position.x
#     p_y = msg.pose.pose.position.y



def possicaoGoal(msg):
    global poseGoal, tf_parent,goalGlobal
    goalGlobal=msg
    #dica de tranformar para o base link
    time = rospy.Time.now()         #acho que ha problemas com o time
    poseGoal=tf_parent.transform(msg, "/p_g5/odom",time)


def main():
    global publisher
    global tf_parent
    global tf_clidren
    tf_parent=tf2_ros.Buffer()
    tf_clidren=tf2_ros.TransformListener(tf_clidren)

    rospy.init_node('palyer', anonymous=False)


    publisher=rospy.Publisher("/p_g5/cmd_vel",Twist,queue_size=10)

    #rospy.Subscriber("/p_g5/odom", Odometry, verOdomCall)

    rospy.Subscriber("/p_g5/move_base_simple/goal",PoseStamped,possicaoGoal)

if __name__ == '__main__':
    main()