#!/usr/bin/env python
import time

from colorama import Fore
from tf.transformations import euler_from_quaternion
import rospy
import tf2_ros
import geometry_msgs
from geometry_msgs.msg import Twist
import math
import tf_conversions
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelStates

global br
global t
global pub
global posicao_final
global posicao_atual
global orientation
global new_goal
global active_goal


def read_goal(message):
    global pose_final
    global pose_atual
    global t
    global active_goal

    if t==1 and active_goal==True:
        print(Fore.BLUE + 'Target Change, abort previous mission' + Fore.RESET)

    active_goal=True
    tfBuffer = tf2_ros.Buffer()
    pose_final = message.pose
    t = 1

def my_position(message):
    global pose_final
    global pose_atual
    global t
    global q
    global x
    global y
    global theta
    global new_goal

    #Tenho de ler apenas o posicao do meu robot

    names = message.name
    index=0
    index_name=None
    for name in names:
        if name=="p_g5":
            index_name=index
            break
        index+=1

    x = message.pose[index_name].position.x
    y = message.pose[index_name].position.y
    rot_q = message.pose[index_name].orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    q=1



def aim_to_goal():
    global pub
    global pose_atual
    global pose_final
    global orientation
    global theta
    global active_goal
    singularidade=0

    linear_velocity=0
    twist = Twist()
    sinal = 1

    while not rospy.is_shutdown():

        if active_goal==True:

            inc_x = pose_final.position.x - x
            inc_y = pose_final.position.y - y
            angle_to_goal = math.atan2(inc_y, inc_x)


            #Estamos desorientados
            if abs(angle_to_goal - theta) > 0.4:

                if linear_velocity>0.2:
                    linear_velocity = linear_velocity - 0.05
                else:
                    linear_velocity = 0.2



                twist.linear.x = linear_velocity

                #Determminar o sinal



                if abs(angle_to_goal)>2.9 and abs(angle_to_goal)<3.2:
                    if theta>0:
                        sinal=sinal
                    else:
                        sinal=-sinal
                    singularidade=1
                else:
                    singularidade=0


                if singularidade != 1:
                    if angle_to_goal-theta >0:
                        sinal=1
                    else:
                        sinal=-1


                if abs(angle_to_goal - theta)>1.5:

                        angular_velocity=sinal*1.0


                else:
                    angular_velocity=sinal*abs(angle_to_goal - theta)/1.5

                twist.angular.z = angular_velocity



            else:
                if (math.sqrt(inc_y**2+inc_x**2))>1:
                    if linear_velocity < 1.2:
                        linear_velocity+=0.1
                    else:

                        linear_velocity=1.2
                    twist.linear.x = linear_velocity
                    twist.angular.z = 0.0

                else:
                    if linear_velocity > 0.5:
                        linear_velocity-=0.1
                    else:
                        linear_velocity=math.sqrt(inc_y**2+inc_x**2)/2

                    twist.linear.x = linear_velocity
                    twist.angular.z = 0.0



            if math.sqrt(inc_y**2+inc_x**2)<0.4:
                linear_velocity = 0
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                print(Fore.GREEN + 'Final Destination!  Do you want to send me somewhere else?' + Fore.RESET)
                active_goal=False
            else:
                print('Chasing Target with linear speed of ' + Fore.LIGHTRED_EX + str(twist.linear.x) + Fore.RESET + ' and angular velocity of ' +
                      Fore.LIGHTRED_EX + str(twist.angular.z) + Fore.RESET )


            pub.publish(twist)
        rospy.sleep(0.1)


def main():
    global br
    global t
    global pub
    global q
    global active_goal

    active_goal=False
    t=0
    rospy.init_node('driver_pg5', anonymous=False)

    # pub = rospy.Publisher(name +'/cmd_vel', Twist, queue_size=10)
    pub = rospy.Publisher('p_g5/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber("p_g5/move_base_simple/goal", PoseStamped, read_goal)
    rospy.Subscriber("/gazebo/model_states", ModelStates, my_position)

    print('Where should I go?')

    while not rospy.is_shutdown():



        if t==1 and q==1:

            aim_to_goal()







if __name__ == '__main__':
    main()
