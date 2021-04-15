#!/usr/bin/env python3
import rospy

# to get commandline arguments
import sys

# because of transformations
import tf
import tf_conversions
import tf2_ros
import geometry_msgs.msg
import gazebo_msgs.msg
import tf2_geometry_msgs
import std_msgs.msg
import math
buffer_tf2 = tf2_ros.Buffer()
time_published = None

def callback_enu_to_ned_base_link(msg: gazebo_msgs.msg.ModelStates, br: tf2_ros.TransformBroadcaster):
    global time_published
    robot_name = "uuv_bluerov2_heavy"
    try:
        index_robot = msg.name.index(robot_name)
    except:
        return 1


    transformation_enu_to_ned = buffer_tf2.lookup_transform("map_ned", 'map', rospy.Time())
    tmp = geometry_msgs.msg.PoseStamped(pose=msg.pose[index_robot])
    pose_ned = tf2_geometry_msgs.do_transform_pose(tmp, transformation_enu_to_ned)

    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    if time_published != t.header.stamp:
        #print("time_published_old=",time_published)
        #print("time_published_current=",t.header.stamp)
        time_published = t.header.stamp
        t.header.frame_id = "map_ned"
        t.child_frame_id = "base_link_gt"
        t.transform.translation.x = pose_ned.pose.position.x
        t.transform.translation.y = pose_ned.pose.position.y
        t.transform.translation.z = pose_ned.pose.position.z
        t.transform.rotation.x = pose_ned.pose.orientation.x
        t.transform.rotation.y = pose_ned.pose.orientation.y
        t.transform.rotation.z = pose_ned.pose.orientation.z
        t.transform.rotation.w = pose_ned.pose.orientation.w

        br.sendTransform(t)

def callback_px4_to_tf(msg: geometry_msgs.msg.PoseStamped,br: tf2_ros.TransformBroadcaster):
    global time_published



    transformation_enu_to_ned = buffer_tf2.lookup_transform("map_ned", 'map', rospy.Time())
    tmp = geometry_msgs.msg.PoseStamped(pose=msg.pose)
    pose_ned = tf2_geometry_msgs.do_transform_pose(tmp, transformation_enu_to_ned)


    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    if time_published != t.header.stamp:
    #print("time_published_old=",time_published)
    #print("time_published_current=",t.header.stamp)
        time_published = t.header.stamp
        t.header.frame_id = "map_ned"
        t.child_frame_id = "base_link"
        t.transform.translation.x = pose_ned.pose.position.x
        t.transform.translation.y = pose_ned.pose.position.y
        t.transform.translation.z = pose_ned.pose.position.z
        t.transform.rotation.x = pose_ned.pose.orientation.x
        t.transform.rotation.y = pose_ned.pose.orientation.y
        t.transform.rotation.z = pose_ned.pose.orientation.z
        t.transform.rotation.w = pose_ned.pose.orientation.w

        br.sendTransform(t)

def callback_base_link_2_top_rotating_sonar(msg: std_msgs.msg.Float64 ,br: tf2_ros.TransformBroadcaster):
    global time_published
    transformStamped = geometry_msgs.msg.TransformStamped()

    transformStamped.header.stamp = rospy.Time.now()
    transformStamped.header.frame_id = "rotating_sonar_bot"
    transformStamped.child_frame_id = "rotating_sonar_top"


    transformStamped.transform.translation.x = 0
    transformStamped.transform.translation.y = 0
    transformStamped.transform.translation.z = 0

    quat2 = tf_conversions.transformations.quaternion_from_euler(math.pi/2.0, 0.0, msg.data)


    transformStamped.transform.rotation.x = quat2[0]
    transformStamped.transform.rotation.y = quat2[1]
    transformStamped.transform.rotation.z = quat2[2]
    transformStamped.transform.rotation.w = quat2[3]
    if time_published != transformStamped.header.stamp:
        time_published = transformStamped.header.stamp
        br.sendTransform(transformStamped)


if __name__ == '__main__':
    #rospy.sleep(5)
    rospy.init_node('dynamic_tf')
    br = tf2_ros.TransformBroadcaster()
    transform_listener = tf2_ros.TransformListener(buffer_tf2)

    rospy.Subscriber('mavros/local_position/pose', geometry_msgs.msg.PoseStamped, callback_px4_to_tf, br)
    rospy.Subscriber('gazebo/model_states', gazebo_msgs.msg.ModelStates, callback_enu_to_ned_base_link, br)
    rospy.Subscriber('sonar/currentRelativeAngleSonar', std_msgs.msg.Float64, callback_base_link_2_top_rotating_sonar, br)
    rospy.spin()
