#!/usr/bin/env python
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
buffer_tf2 = tf2_ros.Buffer()

def get_q_enu_to_ned():
    return tf_conversions.transformations.quaternion_from_euler(math.pi, 0.0,
                                                                -math.pi/2.0)


def callback_enu_to_ned_base_link(msg: gazebo_msgs.msg.ModelStates, br: tf2_ros.TransformBroadcaster):
    t = geometry_msgs.msg.TransformStamped()
    robot_name = "uuv_bluerov2_heavy"
    index_robot = msg.name.index(robot_name)


    transformation_enu_to_ned = buffer_tf2.lookup_transform("map_ned", 'map', rospy.Time())
    #print(tf_conversions.transformations.euler_from_quaternion([transformation_enu_to_ned.transform.rotation.x,
    #                                                            transformation_enu_to_ned.transform.rotation.y,
    #                                                            transformation_enu_to_ned.transform.rotation.z,
    #                                                            transformation_enu_to_ned.transform.rotation.w]))
    tmp = geometry_msgs.msg.PoseStamped(pose=msg.pose[index_robot])
    #print("tmp",tmp)
    pose_ned = tf2_geometry_msgs.do_transform_pose(tmp, transformation_enu_to_ned)

    #print(tf_conversions.transformations.quaternion_multiply(pose_ned.pose.orientation , get_q_enu_to_ned()))


    t.header.stamp = rospy.Time.now()
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

def start_static_transforms(broadcaster: tf2_ros.StaticTransformBroadcaster):
    static_transformStamped = geometry_msgs.msg.TransformStamped()

    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "base_link_frd"
    static_transformStamped.child_frame_id = "front_camera_link"

    static_transformStamped.transform.translation.x = 0
    static_transformStamped.transform.translation.y = 0
    static_transformStamped.transform.translation.z = 0.2

    quat = tf.transformations.quaternion_from_euler(
        0, -0.4, 0)
    quat2 = tf.transformations.quaternion_from_euler(
        math.pi/2.0, 0, math.pi/2.0)# transforms first z axis then y then x
    rotation_base_to_camera=tf_conversions.transformations.quaternion_multiply(quat,quat2)
    static_transformStamped.transform.rotation.x = rotation_base_to_camera[0]
    static_transformStamped.transform.rotation.y = rotation_base_to_camera[1]
    static_transformStamped.transform.rotation.z = rotation_base_to_camera[2]
    static_transformStamped.transform.rotation.w = rotation_base_to_camera[3]

    broadcaster.sendTransform(static_transformStamped)

    static_transformStamped.header.frame_id = "base_link_frd_gt"
    static_transformStamped.child_frame_id = "front_camera_link_gt"

    broadcaster.sendTransform(static_transformStamped)

def start_static_transform_baselink(broadcaster: tf2_ros.StaticTransformBroadcaster):

    not_online = True
    while not_online:
        try:
            static_transform_tmp = buffer_tf2.lookup_transform("base_link_frd", 'base_link', rospy.Time())
            not_online = False
        except:
            rospy.sleep(1)

    static_transform_tmp.header.frame_id = "base_link_gt"
    static_transform_tmp.child_frame_id = "base_link_frd_gt"
    broadcaster.sendTransform(static_transform_tmp)

def start_static_transform_sonar(broadcaster: tf2_ros.StaticTransformBroadcaster):

    static_transformStamped = geometry_msgs.msg.TransformStamped()

    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "base_link_frd_gt"
    static_transformStamped.child_frame_id = "ping_sonar_link_gt"

    static_transformStamped.transform.translation.x = 0
    static_transformStamped.transform.translation.y = 0
    static_transformStamped.transform.translation.z = -0.2

    quat2 = tf.transformations.quaternion_from_euler(math.pi, 0, 0.0)

    static_transformStamped.transform.rotation.x = quat2[0]
    static_transformStamped.transform.rotation.y = quat2[1]
    static_transformStamped.transform.rotation.z = quat2[2]
    static_transformStamped.transform.rotation.w = quat2[3]

    broadcaster.sendTransform(static_transformStamped)





def callback_px4_to_tf(msg: geometry_msgs.msg.PoseStamped,br: tf2_ros.TransformBroadcaster):#propably useless

    t = geometry_msgs.msg.TransformStamped()


    transformation_enu_to_ned = buffer_tf2.lookup_transform("map_ned", 'map', rospy.Time())
    tmp = geometry_msgs.msg.PoseStamped(pose=msg.pose)
    #print("tmp",tmp)
    pose_ned = tf2_geometry_msgs.do_transform_pose(tmp, transformation_enu_to_ned)

    #print(tf_conversions.transformations.quaternion_multiply(pose_ned.pose.orientation , get_q_enu_to_ned()))


    t.header.stamp = rospy.Time.now()
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


if __name__ == '__main__':
    rospy.init_node('coordinate_system_broadcaster')
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    br = tf2_ros.TransformBroadcaster()
    transform_listener = tf2_ros.TransformListener(buffer_tf2)

    start_static_transforms(broadcaster)
    start_static_transform_baselink(broadcaster)
    start_static_transform_sonar(broadcaster)

    rospy.Subscriber('mavros/local_position/pose', geometry_msgs.msg.PoseStamped, callback_px4_to_tf, br)
    rospy.Subscriber('gazebo/model_states', gazebo_msgs.msg.ModelStates, callback_enu_to_ned_base_link, br)
    rospy.spin()
