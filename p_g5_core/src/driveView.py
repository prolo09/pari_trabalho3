#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
import laser_geometry.laser_geometry as lg
import numpy as np




def devolve_certruid(mask):

    mask_inv = cv2.bitwise_not(mask)
    um_labels, labels, stats, centroids_inv = cv2.connectedComponentsWithStats(mask_inv, connectivity=4)

    stats[np.where(stats[:,4]==stats[:,4].max())[0][0], :]=0
    big_area_idx=np.where(stats[:,4]==stats[:,4].max())[0][0]

    x, y=centroids_inv[big_area_idx]


    return centroids_inv


def imagueCallBack(msg):
    global pub, my_team,cont, mantem_direcao

    twist=Twist()
    # rospy.loginfo(msg)

    #read a image an conver for ros
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    #tamanho das telas
    tela_width = cv_image.shape[1]
    tela_heigh = cv_image.shape[0]

    # cv2.imshow('windos_name', cv_image)
    #prossecing the msg


    # limites vermelho, azul
    lim_inf_red=[0,  0, 50]
    lim_sup_red = [0, 0, 255]

    lim_inf_green = [0, 50, 0]
    lim_sup_green = [0, 255, 0]

    lim_inf_blue = [50, 0, 0]
    lim_sup_blue = [255, 0, 0]

    # the mask
    mask_red = cv2.inRange(cv_image, np.array(lim_inf_red), np.array(lim_sup_red))
    mask_green=cv2.inRange(cv_image, np.array(lim_inf_green), np.array(lim_sup_green))
    mask_blue = cv2.inRange(cv_image, np.array(lim_inf_blue), np.array(lim_sup_blue))



    # visao para cacar

    if my_team == "red":
        mask_geral_h=mask_green
    elif my_team=="green":
        mask_geral_h=mask_blue
    elif my_team=="blue":
        mask_geral_h=mask_red


    centroids_h=devolve_certruid(mask_geral_h)



    # cv2.imshow('windos_paracacar', mask_geral_h)
    # visao para fugir
    if my_team == "red":
        mask_geral_p=mask_blue
    elif my_team=="green":
        mask_geral_p=mask_red
    elif my_team=="blue":
        mask_geral_p=mask_green
    # cv2.imshow('windos_fujirr', mask_geral_p)

    centroids_p = devolve_certruid(mask_geral_p)




    #______________________ guiar vaiculo______________________-

    # variavael que me informa se ha parede
    olha_parede=HaParede()
    print(olha_parede)
    #valores mais mais baixos direita

    #possicao do objeto na imaguem
    possicao_h=tela_width-centroids_h[1][0]
    possicao_p=tela_width-centroids_p[1][0]


    if olha_parede==olha_parede != "parede":

        #velocidade fixa liniar
        twist.linear.x=0.8

        # !!!tem na mira presa
        # print(int(possicao_p))
        # print(int(possicao_h))
        # print(tela_width / 2)


        if int(possicao_h)!= tela_width/2 and int(possicao_p)==tela_width/2:
            print('cacar')

            if possicao_h<tela_width/2:
                twist.angular.z=0.7
            elif int(possicao_h)==tela_width/2:
                twist.angular.z = 0
            else:
                twist.angular.z = -0.7



        #tem na mira presa e cacador

        if int(possicao_p) != tela_width / 2 and int(possicao_h)!=tela_width/2:
            print('fugir do cacador e presa')

            twist.linear.x = 0.3
            if possicao_p < tela_width / 2:
                mantem_direcao=-0.9
                twist.angular.z = mantem_direcao
            else:
                mantem_direcao = 0.9
                twist.angular.z =mantem_direcao





            #spentiar
            # cont+=1
            #
            #
            # if cont<10:
            #     if int(possicao_p) < tela_width / 2:
            #         twist.angular.z = 0.5
            #
            #     else:
            #         twist.angular.z = -0.5
            # else:
            #     if cont>20:
            #         cont=0
            #     if int(possicao_p) < tela_width / 2:
            #         twist.angular.z = -0.5
            #
            #     else:
            #         twist.angular.z = 0.5



        #tem na mira cacador
        if int(possicao_p) != tela_width / 2 and int(possicao_h)==tela_width/2:
            print('fugir')
            twist.linear.x = 0.25
            if possicao_p < tela_width / 2:
                mantem_direcao = -0.9
                twist.angular.z = mantem_direcao
            else:
                mantem_direcao = 0.9
                twist.angular.z = mantem_direcao

    else:


        twist.angular.z = mantem_direcao
        twist.linear.x=-0.1




    #tem nao tem nada na mira

    pub.publish(twist)
    k=cv2.waitKey(1)

def scanCallBack(msg):
    # lp=lg.LaserProjection()
    # cartesianas=lp.projectLaser(msg)
    # rospy.loginfo(cartesianas)

    global n_obj_total

    # saber se temos parede ou robo
    lado_dir=msg.ranges[-45:-1]
    n_inf_obj_dir=sum(i<1 for i in lado_dir)

    lado_esq=msg.ranges[0:45]
    n_inf_obj_esq = sum(i < 1 for i in lado_esq)

    n_obj_total=n_inf_obj_esq+n_inf_obj_dir


    # print(str(n_obj_total))
    # print('zero:'+ str(msg.ranges[-60:-1]))
    # rospy.loginfo(msg.ranges)

def HaParede():
    global n_obj_total

    dado_lidar=None
    if n_obj_total > 30:
        # print("e parede")
        dado_lidar="parede"
    elif n_obj_total<30 and n_obj_total != 0:
        # print("e robo")
        dado_lidar = "robo"
    elif n_obj_total==0:
        # print("nada no lidar")
        dado_lidar = "nada"

    return dado_lidar


def main():
    global pub, my_team, cont, mantem_direcao

    rospy.init_node('driverView', anonymous=False)

    # defenir uma modo de virar para quando nao tem objetivos
    mantem_direcao = 0.9

    # reconhece jogador , equipa, e objetivos

    my_name=rospy.get_name().strip('/')

    name_red=rospy.get_param('/red_players')
    name_green = rospy.get_param('/green_players')
    name_blue=rospy.get_param('/blue_players')

    my_team=None

    if my_name in name_red:
        my_team='red'
        my_team_players=name_red
        prey_team_players=name_green
        hunter_team_players=name_blue
    elif my_name in name_green:
        my_team = 'green'
        my_team_players = name_green
        prey_team_players = name_blue
        hunter_team_players = name_red
    elif my_name in name_blue:
        my_team = 'blue'
        my_team_players = name_blue
        prey_team_players = name_red
        hunter_team_players = name_green
    else:
        rospy.logfatal('algum erro o seu nome nao esta na lista das equipas')
        exit()

    print('My name is '+ my_name +' I am team +' + my_team + '+ I am hunting +' + str(prey_team_players) + '+ and fleeing from'+ str(hunter_team_players))

    # recive info by camara

    cont=0
    rospy.Subscriber(my_name+"/camera/rgb/image_raw", Image, imagueCallBack)

    rospy.Subscriber(my_name+"/scan",LaserScan,scanCallBack)



    pub=rospy.Publisher(my_name+"/cmd_vel",Twist,queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    main()
