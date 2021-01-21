#!/usr/bin/env python

import math

import cv2
import random
import sched
import time
from pprint import pprint

import rospy
from cv_bridge import CvBridge
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist, PoseStamped, Pose
from math import pow, atan2, sqrt
from sensor_msgs.msg import Image, LaserScan
import numpy as np
import threading

# !!!!!!!!!!!!!!!!! Attention !!!!!!!!!!!!!!!!!!!!!!
# for use this scrip i should use the (re)param using __name:=XXXXXX
# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

global s


class TurtleBot:

    def __init__(self):
        # Creates a node with name 'turtlebot_controller' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node('p_treis_drive_follow_arrow', anonymous=True)

        # get node name
        player_name = rospy.get_name().strip('/')
        self.player_name = player_name

        # Publisher which will publish to the topic '/turtle1/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('/' + player_name + '/cmd_vel',
                                                  Twist, queue_size=10)
        print('/' + str(player_name) + '/cmd_vel')

        # A subscriber to the topic '/turtle1/pose'. self.update_pose is called
        # when a message of type Pose is received.
        self.pose_subscriber = rospy.Subscriber('/gazebo/model_states/',
                                                ModelStates, self.update_pose)

        self.laser_subscriber = rospy.Subscriber('/' + player_name + '/scan', LaserScan,self.laser_scan)


        # init vel
        self.vel_x = 0.5
        self.vel_z = 0.5
        self.hunt_vel_z_portion = 0.003
        self.hunt_vel_x = 0.8
        self.escape_vel_z = 1
        self.escape_vel_x = 1
        self.unlock_vel_z = 4
        self.unlock_vel_x = 3

        # not hit in walls
        self.unlock_front = False
        self.unlock_back = False

        # preying and hunting
        self.run_away = False
        self.go_away = False
        self.using_vision = False
        self.attack = False

        # Pose
        self.pose = Pose()
        self.rate = rospy.Rate(120)

        # self.unlockme = True
        self.past_y = 0
        self.past_x = 0

        self.goal_pose = Pose()
        self.stop = True

        red_players = rospy.get_param("/red_players")
        green_players = rospy.get_param("/green_players")
        blue_players = rospy.get_param("/blue_players")
        self.my_team = None

        if player_name in red_players:
            self.my_team = "Red"
            my_team_players = red_players
            prey_team_players = green_players
            hunter_team_players = blue_players
        elif player_name in blue_players:
            self.my_team = "Blue"
            my_team_players = blue_players
            prey_team_players = red_players
            hunter_team_players = green_players
        elif player_name in green_players:
            self.my_team = "Green"
            my_team_players = green_players
            prey_team_players = blue_players
            hunter_team_players = red_players
        else:
            rospy.logfatal("Something is wrong")

        print("My name is " + player_name + " I am team " + self.my_team + " I am hunting " + str(prey_team_players) +
              " and fleeing from " + str(hunter_team_players))

        # Subscribe image topic
        rospy.Subscriber(self.player_name + "/camera/rgb/image_raw", Image, self.vision)

    def laser_scan(self,msg):
        # print("range 0 " + str(msg.ranges[0]))
        # print("range 180 " + str(msg.ranges[180]))
        if round(msg.ranges[0]) < 1:
            self.unlock_front = True
        elif round(msg.ranges[180]) < 1:
            self.unlock_back = True
        else:
            self.unlock_back = False
            self.unlock_front = False
    def vision(self, image):
        window_name = 'Hunter vision'
        # cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE) # commented for not over kill my pc
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(image, desired_encoding='passthrough')
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
        # cv2.imshow(window_name, cv_image) # commented for not over kill my pc
        if self.my_team == "Red":
            limits = [(0, 1, 0), (0, 255, 0)]  # BlueGreenRed, limits for find preys
        elif self.my_team == "Blue":
            limits = [(0, 0, 1), (0, 0, 255)]
        elif self.my_team == "Green":
            limits = [(1, 0, 0), (255, 0, 0)]
        mask = cv2.inRange(cv_image, limits[0], limits[1])
        mask = np.uint8(mask)

        # Find largest contour in intermediate image
        output = cv2.connectedComponentsWithStats(mask, connectivity=4)
        big_boy = 0
        big_area = 0
        if output[0] > 1:
            self.using_vision = True
            for i in range(1, output[0]):
                if output[2][i][4] > big_area:
                    big_area = output[2][i][4]
                    big_boy = i
            x = output[2][big_boy][0]
            y = output[2][big_boy][1]
            w = output[2][big_boy][2]
            h = output[2][big_boy][3]
            # find mass center
            mass_center = output[3][big_boy]
            cross_x = int(mass_center[0])
            cross_y = int(mass_center[1])
            cv2.line(mask, (cross_x - 8, cross_y), (cross_x + 8, cross_y), (0, 0, 255, 0), 2)
            cv2.line(mask, (cross_x, cross_y - 8), (cross_x, cross_y + 8), (0, 0, 255, 0), 2)

            image_size = output[1].shape
            img2 = np.zeros(image_size)
            img2[output[1] == big_boy] = 255
            # cv2.imshow("Biggest Area", img2) # commented for not over kill my pc
            # next make robot run/walk for a goal using vision
            if self.attack:
                self.hunter_move(image_size, cross_x)

        else:
            self.using_vision = False
        # cv2.imshow("hunting", mask)  # commented for not over kill my pc
        cv2.waitKey(1)

        # run away
        if self.my_team == "Red":
            limits_hunters = [(1, 0, 0), (255, 0, 0)]  # limits to find my hunters
        elif self.my_team == "Blue":
            limits_hunters = [(0, 1, 0), (0, 255, 0)]
        elif self.my_team == "Green":
            limits_hunters = [(0, 0, 1), (0, 0, 255)]
        mask_hunters = cv2.inRange(cv_image, limits_hunters[0], limits_hunters[1])
        mask_hunters = np.uint8(mask_hunters)

        # Find largest contour in intermediate image
        output_hunter = cv2.connectedComponentsWithStats(mask_hunters, connectivity=4)
        big_boy_hunter = 0
        big_area_hunter = 0
        if output_hunter[0] > 1:
            self.run_away = True
            for j in range(1, output_hunter[0]):
                if output_hunter[2][j][4] > big_area_hunter:
                    big_area_hunter = output_hunter[2][j][4]
                    big_boy_hunter = j
            # find mass center
            mass_center_hunter = output_hunter[3][big_boy_hunter]
            cross_x_hunter = int(mass_center_hunter[0])

            image_size_hunter = output_hunter[1].shape
            img2_hunter = np.zeros(image_size_hunter)
            img2_hunter[output_hunter[1] == big_boy_hunter] = 255
            # cv2.imshow("Biggest Area", img2) # commented for not over kill my pc
            # next make robot run/walk for a goal using vision
            if self.go_away:
                self.withdrawal(image_size_hunter, cross_x_hunter)

    def withdrawal(self, image_size, x):
        vel_msg = Twist()

        # Linear velocity in the x-axis.
        vel_msg.linear.x = self.escape_vel_x
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0

        # Angular velocity in the z-axis.
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        if x > round(image_size[1] / 2):
            vel_msg.angular.z = self.escape_vel_z
        elif x < round(image_size[1] / 2):
            vel_msg.angular.z = -self.escape_vel_z

        # Publishing our vel_msg
        self.velocity_publisher.publish(vel_msg)

        # Publish at the desired rate.
        self.rate.sleep()

        print(vel_msg)

    def hunter_move(self, image_size, x):
        # print(round(image_size[1]/2))
        # create the message for send
        vel_msg = Twist()

        # Linear velocity in the x-axis.
        vel_msg.linear.x = self.hunt_vel_x
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0

        # Angular velocity in the z-axis.
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        if x > round(image_size[1] / 2):
            vel_msg.angular.z = -abs(x - round(image_size[1] / 2)) * self.hunt_vel_z_portion
        elif x < round(image_size[1]/2):
            vel_msg.angular.z = abs(x - round(image_size[1] / 2)) * self.hunt_vel_z_portion

        # Publishing our vel_msg
        self.velocity_publisher.publish(vel_msg)

        # Publish at the desired rate.
        self.rate.sleep()

        print(vel_msg)

    def update_pose(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        # aux1 =
        index = data.name.index(self.player_name)
        aux = data.pose[index]  # search for name of the robot
        self.pose.position.x = aux.position.x
        self.pose.position.y = aux.position.y
        self.pose.orientation.x = aux.orientation.x
        self.pose.orientation.y = aux.orientation.y
        self.pose.orientation.z = aux.orientation.z
        self.pose.orientation.w = aux.orientation.w

    def euclidean_distance(self):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((self.goal_pose.position.x - self.pose.position.x), 2) +
                    pow((self.goal_pose.position.y - self.pose.position.y), 2))

    def linear_vel(self, constant=0.12):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * self.euclidean_distance()

    def steering_angle(self):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return atan2(self.goal_pose.position.y - self.pose.position.y, self.goal_pose.position.x - self.pose.position.x)

    def angular_vel(self, constant=0.5):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        Z_robot = quaternion_to_euler(self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z,
                                      self.pose.orientation.w)
        return constant * (self.steering_angle() - Z_robot * math.pi / 180)

    def direction(self, data):
        """Moves the turtle to the goal."""
        # self.goal_pose = Pose()
        self.stop = False
        self.goal_pose.position.x = data.pose.position.x
        self.goal_pose.position.y = data.pose.position.y

    # def stoped_robot(self):
    #     self.past_x = round(self.pose.position.x, 2)
    #     self.past_y = round(self.pose.position.y, 2)
    #     threading.Timer(3, self.stoped_robot).start()


def quaternion_to_euler(x, y, z, w):
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    Z = math.degrees(math.atan2(t3, t4))

    return Z


def main():
    turtle = TurtleBot()

    turtle.pose_subscriber = rospy.Subscriber('/gazebo/model_states/',
                                              ModelStates, turtle.update_pose)

    # create the message for send
    vel_msg = Twist()

    while not rospy.is_shutdown():

        rospy.Subscriber(turtle.player_name + "/move_base_simple/goal", PoseStamped,
                         turtle.direction)  # subscribe gazebo/model_states

        # rospy.Subscriber(turtle.player_name + "/camera/rgb/image_raw", Image, turtle.vision)
        distance_tolerance = 0.1

        # if turtle.using_vision and not turtle.run_away:
        #     turtle.attack = True
        if turtle.using_vision :
            turtle.attack = True
            # print("Hunting")

        # elif turtle.using_vision and turtle.run_away:
        #     turtle.go_away = True

        elif not turtle.using_vision:
            if turtle.unlock_front:
                vel_msg.linear.x = -turtle.unlock_vel_x
            elif turtle.unlock_back:
                vel_msg.linear.x = turtle.unlock_vel_x
            else:
                vel_msg.linear.x = turtle.vel_x
            vel_msg.linear.y = 0.0
            vel_msg.linear.z = 0.0

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0.0
            vel_msg.angular.y = 0.0
            if turtle.unlock_back or turtle.unlock_front:
                vel_msg.angular.z = turtle.unlock_vel_z
            else:
                vel_msg.angular.z = -turtle.vel_z

            # Publishing our vel_msg
            turtle.velocity_publisher.publish(vel_msg)

            # Publish at the desired rate.
            turtle.rate.sleep()

        else:
            if turtle.euclidean_distance() >= distance_tolerance and turtle.stop == False:
                # Linear velocity in the x-axis.
                vel_msg.linear.x = turtle.linear_vel()
                vel_msg.linear.y = 0.0
                vel_msg.linear.z = 0.0

                # Angular velocity in the z-axis.
                vel_msg.angular.x = 0.0
                vel_msg.angular.y = 0.0
                vel_msg.angular.z = turtle.angular_vel()

                # Publishing our vel_msg
                turtle.velocity_publisher.publish(vel_msg)

                # Publish at the desired rate.
                turtle.rate.sleep()
            else:
                # Stopping our robot after the movement is over.
                vel_msg.linear.x = 0.0
                vel_msg.angular.z = 0.0
                turtle.velocity_publisher.publish(vel_msg)

        # print(vel_msg)


if __name__ == '__main__':
    main()
