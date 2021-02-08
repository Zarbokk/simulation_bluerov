#!/usr/bin/env python
import numpy as np
from pyquaternion import Quaternion
import rospy

from geometry_msgs.msg import Pose, PoseArray, PoseStamped

# publisher_waypoint = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
# publisher_waypoint = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=1)
publisher_waypoint = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=1)

rate = None



def main():
    rospy.init_node('waypoint_pose_send')
    global rate, R, wanted_z_position, distance_to_point, thrust, carrot, yaw
    rate = rospy.Rate(30)

    try:
        while 1:
            send_waypoint = PoseStamped()
            send_waypoint.header.stamp = rospy.Time.now()


            roll = 0.0/180.0*np.pi
            pitch = 0.0/180.0*np.pi
            yaw = 0.0/180.0*np.pi
            x_des = 0
            y_des = 0
            z_des = 11

            qz_90n = Quaternion(axis=[0, 1, 0], angle=roll) * Quaternion(axis=[1, 0, 0], angle=pitch) * Quaternion(
                axis=[0, 0, 1], angle=-(yaw - np.pi / 2))

            send_waypoint.pose.orientation.x = qz_90n.x
            send_waypoint.pose.orientation.y = qz_90n.y
            send_waypoint.pose.orientation.z = qz_90n.z
            send_waypoint.pose.orientation.w = qz_90n.w
            send_waypoint.pose.position.x = y_des
            send_waypoint.pose.position.y = x_des
            send_waypoint.pose.position.z = -(z_des)

            publisher_waypoint.publish(send_waypoint)
            rate.sleep()
    except:
        pass


if __name__ == '__main__':
    main()