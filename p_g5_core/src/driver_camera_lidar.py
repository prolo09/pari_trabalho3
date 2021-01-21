#!/usr/bin/env python
import sys
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
from sensor_msgs.msg import LaserScan
import sys


class Player:

    def __init__(self):

        rospy.init_node('driver', anonymous=False)
        name = rospy.get_name().strip('/')
        rospy.sleep(0.2)  # make sure the rospy time works

        self.pub = rospy.Publisher(name + '/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber(name + "/camera/rgb/image_raw", Image, self.ImageCallback)
        rospy.Subscriber(name + "/scan", LaserScan, self.lidar_callback)


        #-------------------------------------------------------------------
        #---------------------Variable Initialization-----------------------
        #-------------------------------------------------------------------

        # make sure that the code only works it there is image
        self.exist_image = 0

        # connectivity between pixels
        self.connectivity = 4

        self.my_team = None
        self.index_color = {'blue': 0, 'green': 1, 'red': 2}

        self.blue_limits = {'B': {'max': 255, 'min': 100}, 'G': {'max': 50, 'min': 0}, 'R': {'max': 50, 'min': 0}}
        self.red_limits = {'B': {'max': 50, 'min': 0}, 'G': {'max': 50, 'min': 0}, 'R': {'max': 255, 'min': 100}}
        self.green_limits = {'B': {'max': 50, 'min': 0}, 'G': {'max': 255, 'min': 100}, 'R': {'max': 50, 'min': 0}}

        self.size_bool = False
        self.list_centroid=[]
        self.state='wait'
        self.substate=None
        self.rate = rospy.Rate(10)  # 10hz

        # -------------------------------------------------------------------
        # ---------------------Read Parameters-------------------------------
        # -------------------------------------------------------------------

        names_red = rospy.get_param('/red_players')
        names_green = rospy.get_param('/green_players')
        names_blue = rospy.get_param('/blue_players')






        if name in names_red:
            self.my_team = 'red'
            self.hunter_team = 'blue'
            self.prey_team = 'green'
            self.my_team_players = names_red
            self.prey_team_players = names_green
            self.hunter_team_players = names_blue
        elif name in names_green:
            self.my_team = 'green'
            self.hunter_team = 'red'
            self.prey_team = 'blue'
            self.my_team_players = names_green
            self.prey_team_players = names_blue
            self.hunter_team_players = names_red
        elif name in names_blue:
            self.my_team = 'blue'
            self.hunter_team = 'green'
            self.prey_team = 'red'
            self.my_team_players = names_blue
            self.prey_team_players = names_red
            self.hunter_team_players = names_green
        else:
            rospy.logfatal('Something is wrong \n You have been suspended from your own team')
            exit(0)

        print('My name is ' + name + ' I am team ' + self.my_team + Fore.GREEN + ' I am hunting ' + Fore.RESET + str(
            self.prey_team_players)
              + Fore.RED + ' and fleeing from ' + Fore.RESET + str(self.hunter_team_players))



        # -------------------------------------------------------------------
        # ---------------------Callbacks Functions---------------------------
        # -------------------------------------------------------------------



        rospy.Timer(rospy.Duration(0.1), self.print_state, oneshot=False)



    def ImageCallback(self,message):

        bridge = CvBridge()
        self.cv_image = bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')

        self.blue_mask = cv2.inRange(self.cv_image, (self.blue_limits['B']['min'], self.blue_limits['G']['min'], self.blue_limits['R']['min']),
                           (self.blue_limits['B']['max'], self.blue_limits['G']['max'], self.blue_limits['R']['max']))

        self.red_mask = cv2.inRange(self.cv_image, (self.red_limits['B']['min'], self.red_limits['G']['min'], self.red_limits['R']['min']),
                           (self.red_limits['B']['max'], self.red_limits['G']['max'], self.red_limits['R']['max']))

        self.green_mask = cv2.inRange(self.cv_image, (self.green_limits['B']['min'], self.green_limits['G']['min'], self.green_limits['R']['min']),
                                           (self.green_limits['B']['max'], self.green_limits['G']['max'], self.green_limits['R']['max']))

        self.exist_image=1


    def lidar_callback(self,message):

        angle = message.angle_min

        #print(message.ranges[0])

        if message.ranges[0]<1.3:
            self.substate="escape_wall"
        else:
            self.substate=None

    def get_centroid(self):

        if self.exist_image:

            if self.size_bool == False:
                self.height = self.cv_image.shape[0]
                self.width = self.cv_image.shape[1]
                self.size_bool = True


            mask_list = [self.blue_mask,self.green_mask,self.red_mask]

            for_index=0
            list_biggest_target=[]
            self.list_centroid=[]


            for mask in mask_list:

                output = cv2.connectedComponentsWithStats(mask, self.connectivity, cv2.CV_32S)
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
                    self.list_centroid.append(centroid_coord)
                else:
                    self.list_centroid.append(None)

                for_index+=1


            #Algoritmo de decisao

            self.DecisionMaking()


    def DecisionMaking(self):



        if self.list_centroid[self.index_color[self.hunter_team]]!=None and self.list_centroid[self.index_color[self.prey_team]]==None:
            # Caso eu veja um atacante meu e nao veja presas, FUGIR
            self.state='flee'
        elif self.list_centroid[self.index_color[self.hunter_team]]!=None and self.list_centroid[self.index_color[self.prey_team]]!=None:
            #Se eu vir os 2, tenho de tomar uma decisao dependendo da distancia entre eles



            distance_prey_hunter=abs(float(self.list_centroid[self.index_color[self.hunter_team]][0])-float(self.list_centroid[self.index_color[self.prey_team]][0]))
            #print(distance_prey_hunter)
            if distance_prey_hunter > self.width/3:
                self.state='atack'
            else:
                self.state='flee'
        elif self.list_centroid[self.index_color[self.hunter_team]]==None and self.list_centroid[self.index_color[self.prey_team]]!=None:
            self.state = 'atack'
        elif self.list_centroid[self.index_color[self.hunter_team]]==None and self.list_centroid[self.index_color[self.prey_team]]==None:
            self.state = 'wait'

        self.take_action()


    def take_action(self):

        twist = Twist()

        if self.state=='atack':

            if self.list_centroid[self.index_color[self.prey_team]]!=None:

                horizontal_distance=self.find_direction(self.list_centroid[self.index_color[self.prey_team]])
                #print(horizontal_distance)

                twist.linear.x = 1.0
                twist.angular.z = horizontal_distance/500

        elif self.state=='flee':

            horizontal_distance = self.find_direction(self.list_centroid[self.index_color[self.hunter_team]])
            #print(horizontal_distance)

            if horizontal_distance>0:
                signal = -1
            else:
                signal = 1

            twist = Twist()

            twist.linear.x = 1.5
            twist.angular.z = signal*1.5



        else:

            if self.substate=="escape_wall":
                #fugir da parede
                twist.linear.x = 0.2
                twist.angular.z = 2

            else:
                twist.linear.x = 0.6
                twist.angular.z = 0.3

        self.pub.publish(twist)


    def find_direction(self,centroid_coord):

        #if > 0 turn left
        #if < 0 turn right
        return (self.width/2) - centroid_coord[0]

    def print_state(self,event):

        if self.state == 'flee':
            print(Fore.RED + self.state + ' from ' + self.hunter_team + Fore.RESET)
        elif self.state == 'atack':
            print(Fore.GREEN + self.state + ' and kill ' + self.prey_team + Fore.RESET)
        else:
            if self.substate==None:
                print(Fore.BLUE + self.state + ' for my next prey ' + Fore.RESET)
            else:
                print(Fore.BLUE +'Too close of a wall, better turn around' + Fore.RESET)





def main():

    player = Player()


    while not rospy.is_shutdown():

        player.get_centroid()

        player.rate.sleep()



    # ---------------
    # program's end
    # ---------------
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
