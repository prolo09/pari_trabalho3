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
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

global br
global t
global pub
global posicao_final
global posicao_atual
global orientation
global new_goal
global cv_image
global exist_image
global blue_limits
global red_limits
global green_limits
global biggest_target
global height
global width
global index_color
global blue_mask
global green_mask
global red_mask
global hunter_team
global prey_team

def DecisionMaking(list_centroid):
    global state
    global index_color

    if list_centroid[index_color[hunter_team]]!=None and list_centroid[index_color[prey_team]]==None:
        # Caso eu veja um atacante meu e nao veja presas, FUGIR
        state='flee'
    elif list_centroid[index_color[hunter_team]]!=None and list_centroid[index_color[prey_team]]!=None:
        #Se eu vir os 2, tenho de tomar uma decisao dependendo da distancia entre eles

        #print(list_centroid[index_color['red']][0])
        #print(list_centroid[index_color['blue']][0])
        #print(str(float(list_centroid[index_color['red']][0])-float(list_centroid[index_color['blue']][0])))

        distance_prey_hunter=abs(float(list_centroid[index_color[hunter_team]][0])-float(list_centroid[index_color[prey_team]][0]))
        #print(distance_prey_hunter)
        if distance_prey_hunter > width/3:
            state='atack'
        else:
            state='flee'
    elif list_centroid[index_color[hunter_team]]==None and list_centroid[index_color[prey_team]]!=None:
        state = 'atack'
    elif list_centroid[index_color[hunter_team]]==None and list_centroid[index_color[prey_team]]==None:
        state = 'wait'

    return list_centroid

def find_direction(centroid_coord):
    global height
    global width

    #print(width)
    #print(centroid_coord[0])

    #if > 0 turn left
    #if < 0 turn right
    return (width/2) - centroid_coord[0]



def get_centroid(connectivity):
    global biggest_target
    global blue_mask
    global green_mask
    global red_mask
    global state
    global index_color

    mask_list = [blue_mask,green_mask,red_mask]

    for_index=0
    list_biggest_target=[]
    list_centroid=[]


    for mask in mask_list:

        output = cv2.connectedComponentsWithStats(mask, connectivity, cv2.CV_32S)
        num_labels = output[0]  # integer with the number of object in the image
        labels = output[1]  # in labels we have an image, and each element has a value equivalent to its label
        stats = output[2]  # in stats we have all data for each object
        centroids = output[3]  # in centroids we have all centroids coordinates for each object

        # finding the object with bigger area
        anyone = True
        maximum_area = 0
        object_index = 1
        # if num_labels == 1 means that there is no object, so we cannot paint!
        if num_labels == 1:
            anyone = False
        for i in range(1, num_labels):

            object_area = stats[i, cv2.CC_STAT_AREA]

            if object_area > maximum_area:
                maximum_area = object_area
                object_index = i

        # if maximum_area <500 the object is too small, so its possible that it is not the phone but noise instead
        if maximum_area < 20:
            anyone = False
        # extracting biggest object from segmentation limits
        biggest_target = (labels == object_index)
        biggest_target = biggest_target.astype(np.uint8) * 255
        list_biggest_target.append(biggest_target)

        if anyone:
            centroid_coord = centroids[object_index, :].astype(np.uint)
            centroid_coord = tuple(centroid_coord)
            list_centroid.append(centroid_coord)
        else:
            list_centroid.append(None)

        for_index+=1


    #print(list_centroid)

    #Algoritmo de decisao

    DecisionMaking(list_centroid)

    return list_centroid


def ImageCallback(message):

    global cv_image
    global exist_image
    global blue_limits
    global red_limits
    global green_limits
    global red_mask
    global blue_mask
    global green_mask
    global state

    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')

    blue_mask = cv2.inRange(cv_image, (blue_limits['B']['min'], blue_limits['G']['min'], blue_limits['R']['min']),
                       (blue_limits['B']['max'], blue_limits['G']['max'], blue_limits['R']['max']))

    red_mask = cv2.inRange(cv_image, (red_limits['B']['min'], red_limits['G']['min'], red_limits['R']['min']),
                       (red_limits['B']['max'], red_limits['G']['max'], red_limits['R']['max']))

    green_mask = cv2.inRange(cv_image, (green_limits['B']['min'], green_limits['G']['min'], green_limits['R']['min']),
                                       (green_limits['B']['max'], green_limits['G']['max'], green_limits['R']['max']))

    exist_image=1


def take_action(list_centroid):
    global pub
    global state

    twist = Twist()

    if state=='atack':

        if list_centroid[index_color[prey_team]]!=None:

            horizontal_distance=find_direction(list_centroid[index_color[prey_team]])
            #print(horizontal_distance)



            twist.linear.x = 1.0
            twist.angular.z = horizontal_distance/500

    elif state=='flee':

        horizontal_distance = find_direction(list_centroid[index_color[hunter_team]])
        #print(horizontal_distance)

        if horizontal_distance>0:
            signal = -1
        else:
            signal = 1

        twist = Twist()

        twist.linear.x = 1.5
        twist.angular.z = signal*1.5



    else:
        twist.linear.x = 0.5
        twist.angular.z = 0.3

    pub.publish(twist)

def main():
    global cv_image
    global br
    global t
    global pub
    global q
    global exist_image
    global blue_limits
    global green_limits
    global red_limits
    global biggest_target
    global height
    global width
    global index_color
    global red_mask
    global blue_mask
    global green_mask
    global hunter_team
    global prey_team




    exist_image=0
    # connectivity between pixels
    connectivity = 4
    t = 0
    rospy.init_node('driver', anonymous=False)
    name = rospy.get_name().strip('/')
    names_red = rospy.get_param('/red_players')
    names_green = rospy.get_param('/green_players')
    names_blue = rospy.get_param('/blue_players')
    my_team = None
    index_color = {'blue': 0, 'green': 1, 'red': 2}
    # print(index_color['blue'])

    blue_limits = {'B': {'max': 255, 'min': 100}, 'G': {'max': 50, 'min': 0}, 'R': {'max': 50, 'min': 0}}
    red_limits = {'B': {'max': 50, 'min': 0}, 'G': {'max': 50, 'min': 0}, 'R': {'max': 255, 'min': 100}}
    green_limits = {'B': {'max': 50, 'min': 0}, 'G': {'max': 255, 'min': 100}, 'R': {'max': 50, 'min': 0}}



    if name in names_red:
        my_team = 'red'
        hunter_team='blue'
        prey_team='green'
        my_team_players = names_red
        prey_team_players = names_green
        hunter_team_players = names_blue
    elif name in names_green:
        my_team = 'green'
        hunter_team='red'
        prey_team='blue'
        my_team_players = names_green
        prey_team_players = names_blue
        hunter_team_players = names_red
    elif name in names_blue:
        my_team = 'blue'
        hunter_team='green'
        prey_team='red'
        my_team_players = names_blue
        prey_team_players = names_red
        hunter_team_players = names_green
    else:
        rospy.logfatal('Something is wrong \n You have been suspended from your own team')
        exit(0)

    print('My name is ' + name + ' I am team ' + my_team  + Fore.GREEN + ' I am hunting ' + Fore.RESET + str(prey_team_players)
         + Fore.RED + ' and fleeing from ' + Fore.RESET + str(hunter_team_players))


    pub = rospy.Publisher(name +'/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber(name+"/camera/rgb/image_raw", Image, ImageCallback)
    rate = rospy.Rate(10)  # 10hz
    size_bool=False
    while not rospy.is_shutdown():

        try:

            if exist_image:


                if size_bool==False:
                    height = cv_image.shape[0]
                    width = cv_image.shape[1]
                    size_bool=True




                #get_centroid diz logo o que e preciso fazer aseguir, ou atacar ou fugir
                list_centroid=get_centroid(connectivity)

                take_action(list_centroid)

                if state=='flee':
                    print(Fore.RED + state + ' from ' + hunter_team + Fore.RESET)
                elif state=='atack':
                    print(Fore.GREEN + state + ' and kill ' + prey_team + Fore.RESET)
                else:
                    print(Fore.BLUE + state + ' for my next prey ' + Fore.RESET)

                # cv2.imshow('Image', cv_image)
                # key = cv2.waitKey(1)


                rate.sleep()
        except:
            twist=Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0

            pub.publish(twist)



    # ---------------
    # program's end
    # ---------------
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
