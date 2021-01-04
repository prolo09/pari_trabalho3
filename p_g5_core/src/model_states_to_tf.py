#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from gazebo_msgs.msg import ModelStates
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg

# def talker(data):
#     talker_topic_name = 'point_cloud'
#     pub = rospy.Publisher(talker_topic_name, PointCloud2, queue_size=10)
#
#     lp = lg.LaserProjection()
#     cartesian = lp.projectLaser(data)
#
#     rospy.loginfo(cartesian)
#     pub.publish(cartesian)


def callback(data):
   # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.pose)
    rospy.loginfo(data.pose)
    # no para fazer a tranformacao

    # # rospy.init_node('Wold')
    # br = tf2_ros.TransformBroadcaster()
    # t = geometry_msgs.msg.TransformStamped()
    #
    # t.header.stamp = rospy.Time.now()
    # t.header.frame_id = "father"
    # t.child_frame_id = "child_name"
    #
    # while not rospy.is_shutdown():
    #     t.transform.translation.x = data.pose.position.x
    #     t.transform.translation.y =data.pose.position.y
    #     t.transform.translation.z =data.pose.position.z
    #
    #     t.header.stamp = rospy.Time.now()
    #     br.sendTransform(t)
    #
    # rospy.spin()

def listener():
    listener_topic_name = '/gazebo/model_states'
    rospy.init_node('Gazebo_subscriber', anonymous=False)
    rospy.Subscriber(listener_topic_name, ModelStates, callback)


    rospy.spin()


if __name__ == '__main__':
    listener()
