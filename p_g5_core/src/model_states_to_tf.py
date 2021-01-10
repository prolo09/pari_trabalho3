#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg

#variaveis gloobais
global broad
global t


#__________________________________nota______________________________________
#nao esquecer que o nome dos robos a intruduzir sao os nomes que esta na lista
#atualiar da forma a ter os nomes aumaticos
#____________________________________________________________________________


def callbackCoorGazibo(msg):
    global broad,t

    name=msg.name
    msg_pose=msg.pose


    lista_name=["p_g5","p_g5_red","p_g5_grenn","p_g5_blue","red1", "red2", "red3","blue1", "blue2", "blue3","green1", "green2", "green3"]


    # para lidar com varios robos

    for i, is_name in enumerate(name):
        if is_name in lista_name:

            # print(is_name)
            # print(msg_pose.position(0).x)

            #nome para o father
            t.header.frame_id="world"

            #nome para o child
            name_frame= is_name + "/odom"
            t.child_frame_id =name_frame

            # aplicar as tranformacoes para a coordenadas para a odometria
            #tranlacao
            t.header.stamp = rospy.Time.now()
            t.transform.translation.x=msg_pose[i].position.x
            t.transform.translation.y = msg_pose[i].position.y
            t.transform.translation.z=msg_pose[i].position.z
            #rotacao
            t.transform.rotation.x=msg_pose[i].orientation.x
            t.transform.rotation.y = msg_pose[i].orientation.y
            t.transform.rotation.z = msg_pose[i].orientation.z
            t.transform.rotation.w = msg_pose[i].orientation.w
            broad.sendTransform(t)

def main():
    global broad,t

    #inicializar o no
    rospy.init_node("model_state_to_tf", anonymous=False)

    #topic do gazebo a subscrever
    gazebo_frame="/gazebo/model_states"
    rospy.Subscriber(gazebo_frame, ModelStates, callbackCoorGazibo)

    # step tranformadas para do gazebo para o odom
    broad=tf2_ros.TransformBroadcaster()
    t=geometry_msgs.msg.TransformStamped()

    rospy.spin()


if __name__ == '__main__':
    main()