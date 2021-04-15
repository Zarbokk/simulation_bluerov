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
import math
def get_q_enu_to_ned():
    return tf_conversions.transformations.quaternion_from_euler(math.pi, 0.0,
                                                                -math.pi/2.0)

def get_pose_base_to_camera():
    quat = tf_conversions.transformations.quaternion_from_euler(
        0, -0.4, 0)
    quat2 = tf_conversions.transformations.quaternion_from_euler(
    math.pi/2.0, 0, math.pi/2.0)# transforms first z axis then y then x
    rotation_base_to_camera=tf_conversions.transformations.quaternion_multiply(quat,quat2)
    return [[0,0,0.2],rotation_base_to_camera]

def static_transform_blf_2_fcl():
    static_transformStamped = geometry_msgs.msg.TransformStamped()

    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "base_link_frd"
    static_transformStamped.child_frame_id = "front_camera_link"

    [position_base_to_camera,rotation_base_to_camera] = get_pose_base_to_camera()

    static_transformStamped.transform.translation.x = position_base_to_camera[0]
    static_transformStamped.transform.translation.y = position_base_to_camera[1]
    static_transformStamped.transform.translation.z = position_base_to_camera[2]

    static_transformStamped.transform.rotation.x = rotation_base_to_camera[0]
    static_transformStamped.transform.rotation.y = rotation_base_to_camera[1]
    static_transformStamped.transform.rotation.z = rotation_base_to_camera[2]
    static_transformStamped.transform.rotation.w = rotation_base_to_camera[3]

    return static_transformStamped

def static_transform_blf_2_fcl_gt():
    static_transformStamped = geometry_msgs.msg.TransformStamped()

    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "base_link_frd_gt"
    static_transformStamped.child_frame_id = "front_camera_link_gt"

    [position_base_to_camera,rotation_base_to_camera] = get_pose_base_to_camera()

    static_transformStamped.transform.translation.x = position_base_to_camera[0]
    static_transformStamped.transform.translation.y = position_base_to_camera[1]
    static_transformStamped.transform.translation.z = position_base_to_camera[2]

    static_transformStamped.transform.rotation.x = rotation_base_to_camera[0]
    static_transformStamped.transform.rotation.y = rotation_base_to_camera[1]
    static_transformStamped.transform.rotation.z = rotation_base_to_camera[2]
    static_transformStamped.transform.rotation.w = rotation_base_to_camera[3]

    return static_transformStamped

def static_transform_bl_2_blfrd_gt(buffer_tf2: tf2_ros.Buffer):

    not_online = True
    while not_online:
        try:
            static_transform_tmp = buffer_tf2.lookup_transform("base_link_frd", 'base_link', rospy.Time())
            not_online = False
        except:
            rospy.sleep(1)

    static_transform_tmp.header.frame_id = "base_link_gt"
    static_transform_tmp.child_frame_id = "base_link_frd_gt"
    return static_transform_tmp

def static_transform_blfrdt_2_psl_gt():

    static_transformStamped = geometry_msgs.msg.TransformStamped()

    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "base_link_frd_gt"
    static_transformStamped.child_frame_id = "ping_sonar_link_gt"

    static_transformStamped.transform.translation.x = 0
    static_transformStamped.transform.translation.y = 0
    static_transformStamped.transform.translation.z = -0.2

    quat2 = tf_conversions.transformations.quaternion_from_euler(math.pi, 0, 0.0)


    static_transformStamped.transform.rotation.x = quat2[0]
    static_transformStamped.transform.rotation.y = quat2[1]
    static_transformStamped.transform.rotation.z = quat2[2]
    static_transformStamped.transform.rotation.w = quat2[3]

    return static_transformStamped

def static_transform_blf_2_rots():
    static_transformStamped = geometry_msgs.msg.TransformStamped()

    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "base_link_frd_gt"
    static_transformStamped.child_frame_id = "rotating_sonar_bot"


    static_transformStamped.transform.translation.x = 0
    static_transformStamped.transform.translation.y = 0
    static_transformStamped.transform.translation.z = -(0.2+0.029335+0.095455)

    quat2 = tf_conversions.transformations.quaternion_from_euler(math.pi, 0, 0.0)


    static_transformStamped.transform.rotation.x = quat2[0]
    static_transformStamped.transform.rotation.y = quat2[1]
    static_transformStamped.transform.rotation.z = quat2[2]
    static_transformStamped.transform.rotation.w = quat2[3]

    return static_transformStamped


if __name__ == '__main__':
    rospy.init_node('static_tf')
    buffer_tf2 = tf2_ros.Buffer()
    transform_listener = tf2_ros.TransformListener(buffer_tf2)

    broadcaster = tf2_ros.StaticTransformBroadcaster()

    transform_1 = static_transform_blf_2_fcl()
    transform_2 = static_transform_blf_2_fcl_gt()
    transform_3 = static_transform_bl_2_blfrd_gt(buffer_tf2)
    transform_4 = static_transform_blfrdt_2_psl_gt()
    transform_5 = static_transform_blf_2_rots()

    broadcaster.sendTransform([transform_1,transform_2,transform_3,transform_4,transform_5])

    rospy.spin()
