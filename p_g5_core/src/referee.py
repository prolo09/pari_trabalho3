#!/usr/bin/env python

import rospy
from colorama import Fore
from gazebo_msgs.srv import DeleteModel, DeleteModelRequest # Import the service message used by the service /gazebo/delete_model
from gazebo_msgs.msg import ContactsState
from readchar import readchar

global delete_model_service
global red_alive
global green_alive
global blue_alive


def Red1Contact(message):
    global red_alive
    global green_alive
    global blue_alive
    global delete_model_service

    #print(message.header)
    #print(message.states)
    try:
        if green_alive == True:
            if 'green' in message.states[0].collision1_name or 'green' in message.states[0].collision2_name:

                delete_model_service('green1')
                print(Fore.RED + 'green1 is dead' + Fore.RESET )
                green_alive=False
        if red_alive==True:
            if 'blue' in message.states[0].collision1_name or 'blue' in message.states[0].collision2_name:
                delete_model_service('red1')
                print(Fore.RED + 'red1 is dead' + Fore.RESET)
                red_alive = False

    except:
        pass

def Green1Contact(message):

    global red_alive
    global green_alive
    global blue_alive
    global delete_model_service

    try:
        if blue_alive==True:
            if 'blue' in message.states[0].collision1_name or 'blue' in message.states[0].collision2_name:
                delete_model_service('blue1')
                print(Fore.RED + 'blue1 is dead' + Fore.RESET)
                blue_alive=False

    except:
        pass

def main():
    global delete_model_service
    global red_alive
    global green_alive
    global blue_alive

    red_alive=True
    green_alive=True
    blue_alive=True

    rospy.init_node('referee', anonymous=False)
    rospy.Subscriber("/red1/contact", ContactsState, Red1Contact)
    rospy.Subscriber("/green1/contact", ContactsState, Green1Contact)
    rospy.wait_for_service('/gazebo/delete_model')  # Wait for the service client /gazebo/delete_model to be running
    delete_model_service = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)  # Create the connection to the service
    kk = DeleteModelRequest()  # Create an object of type DeleteModelRequest
    # kk.model_name = "green1"
    # delete_model_service('green1')# Fill the variable model_name of this object with the desired value
    # print('ola')

    lista_players=['red1','green1','blue1']

    print(Fore.CYAN + 'Let the games begin' + Fore.RESET)

    rospy.sleep(5)

    while not rospy.is_shutdown():

        lista=[red_alive,green_alive,blue_alive]

        if lista==[True,True,True]:
            print(Fore.CYAN + 'Who is going to die first?' + Fore.RESET)
        elif lista==[False,True,True] or lista==[True,False,True] or lista==[True,True,False]:
            print(Fore.CYAN + 'The final has begun! Who will be the last survivor?' + Fore.RESET)
        elif lista==[True,False,False] or lista==[False,True,False] or lista==[False,False,True]:
            index_winner=lista.index(True)
            winner=lista_players[index_winner]
            print(Fore.GREEN + 'And the last survivor is ' + winner + Fore.RESET)
            print('Do you want to play again? y/n')
            char=readchar()
            if char=='y':
                print(Fore.RED + 'You cant, the players are already dead!' + Fore.RESET)
                exit(0)
            else:
                print('Closing')
                exit(0)
        rospy.sleep(1)

if __name__ == '__main__':
    main()
