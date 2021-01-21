#!/usr/bin/env python
import numpy as np
import rospkg
from pyquaternion import Quaternion
import rospy
# import tf

from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from mavros_msgs.msg import PositionTarget, AttitudeTarget
# from numpy import genfromtxt
# import os
# import matplotlib.pyplot as plt
import csv

# publisher_waypoint = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
# publisher_waypoint = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=1)
publisher_waypoint = rospy.Publisher('mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)
publisher_marker = rospy.Publisher('/infinity', MarkerArray, queue_size=1)
current_pos_number = 10
N = 100
rate = None


# set_mode_srv = rospy.ServiceProxy('mavros/set_mode', SetMode)
# res = set_mode_srv(0, " OFFBOARD")

def create_inf(dist_y=0.7, dist_x=0.9, r=0.5, dist_circle=1.2, N=100):
    while not N % 4 == 0:
        N = N + 1
    waypoints_x = np.zeros(0)
    waypoints_y = np.zeros(0)
    p1_x = dist_x
    p1_y = dist_y + r
    p2_x = dist_x + dist_circle
    p2_y = dist_y - r
    p3_x = dist_x + dist_circle
    p3_y = dist_y + r
    p4_x = dist_x
    p4_y = dist_y - r

    x = np.linspace(0, np.pi, num=N / 4)
    x_circle_r = dist_x - np.sin(x) * r
    y_circle_r = dist_y - np.cos(x) * r

    waypoints_x = np.append(waypoints_x, x_circle_r)
    waypoints_y = np.append(waypoints_y, y_circle_r)
    # at point 1
    waypoints_x = np.append(waypoints_x, np.linspace(p1_x, p2_x, N / 4))
    waypoints_y = np.append(waypoints_y, np.linspace(p1_y, p2_y, N / 4))
    # at point 2
    x_circle_l = dist_x + dist_circle + np.sin(x) * r
    y_circle_l = dist_y - np.cos(x) * r
    waypoints_x = np.append(waypoints_x, x_circle_l)
    waypoints_y = np.append(waypoints_y, y_circle_l)
    # at point 3
    waypoints_x = np.append(waypoints_x, np.linspace(p3_x, p4_x, N / 4))
    waypoints_y = np.append(waypoints_y, np.linspace(p3_y, p4_y, N / 4))
    # at point 4
    #print(waypoints_x.shape)
    #print(waypoints_y.shape)
    # plt.plot(waypoints_x, waypoints_y)
    # plt.plot(np.zeros(10), np.linspace(0, 2, 10))
    # plt.plot(np.ones(10) * 4.1, np.linspace(0, 2, 10))
    # plt.plot(np.linspace(0, 4.1, 10), np.zeros(10))
    # plt.plot(np.linspace(0, 4.1, 10), np.ones(10) * 2)
    # axes = plt.gca()
    # axes.set_xlim([-0.5, 4.5])
    # axes.set_ylim([-0.5, 4.5])
    # plt.show()
    waypoints_x = np.asarray(waypoints_x)
    waypoints_y = np.asarray(waypoints_y)
    #print(waypoints_x.shape)
    angle = np.zeros(N)
    for i in range(N):
        left_point = i - 1
        right_point = i + 1
        if i == 0:
            left_point = N - 1
            right_point = i + 1
        if i == N - 1:
            left_point = i - 1
            right_point = 0
        angle_trajectory = np.arctan2((-waypoints_y[left_point] + waypoints_y[right_point]),
                                      (-waypoints_x[left_point] + waypoints_x[right_point]))
        angle[i] = angle_trajectory
        #print(angle_trajectory)

    return (np.asarray([range(0, N), waypoints_x, waypoints_y, angle]))


# def circle(dist_y=1, dist_x=1.9, r=0.5, dist_circle=1.2, N=100):
#     while not N % 4 == 0:
#         N = N + 1
#     waypoints_x = np.zeros(0)
#     waypoints_y = np.zeros(0)
#     p1_x = dist_x
#     p1_y = dist_y + r
#     p2_x = dist_x + dist_circle
#     p2_y = dist_y - r
#     p3_x = dist_x + dist_circle
#     p3_y = dist_y + r
#     p4_x = dist_x
#     p4_y = dist_y - r
#
#     x = np.linspace(0, np.pi * 2, num=N)
#     x_circle = dist_x - np.sin(x) * r
#     y_circle = dist_y - np.cos(x) * r
#
#     plt.plot(x_circle, y_circle)
#     plt.plot(np.zeros(10), np.linspace(0, 2, 10))
#     plt.plot(np.ones(10) * 4.1, np.linspace(0, 2, 10))
#     plt.plot(np.linspace(0, 4.1, 10), np.zeros(10))
#     plt.plot(np.linspace(0, 4.1, 10), np.ones(10) * 2)
#     axes = plt.gca()
#     axes.set_xlim([-0.5, 4.5])
#     axes.set_ylim([-0.5, 4.5])
#     # plt.show()
#     waypoints_x = np.asarray(x_circle)
#     waypoints_y = np.asarray(y_circle)
#     angle = np.zeros(N)
#     for i in range(N):
#         left_point = i - 1
#         right_point = i + 1
#         if i == 0:
#             left_point = N - 1
#             right_point = i + 1
#         if i == N - 1:
#             left_point = i - 1
#             right_point = 0
#         angle_trajectory = np.arctan2((-waypoints_y[left_point] + waypoints_y[right_point]),(-waypoints_x[left_point] + waypoints_x[right_point]))
#         angle[i] = angle_trajectory
#         print(angle_trajectory)
#     return (np.asarray([range(0, N), x_circle, y_circle, angle]))


# change:
# abstand zu punkt damit gedaempft(nicht sehr viel maybe R=0.25
# distance to point verringern damit desired winkel groesser
R = 0.2
wanted_z_position = 0
distance_to_point = 0.5
thrust = 0.2
carrot = 1
p = create_inf()


# print(np.transpose(p))
# np.savetxt("infinity_symbol.csv",p,delimiter=",")
# exit()


# for i in range(p.shape[1]):
#     print(p[1,i],p[2, i])
# print(p.shape)
# # print(p[0])
#
#
# print(p[1:3,0])
# exit()


def callback(msg):
    """"""
    global current_pos_number, N, R, p, rate, thrust, carrot, yaw
    # msg=PoseStamped()# SPAETER ENTFERNEN@@@@@@@@@@@@@@@@@@@@@@@@@@@

    # msg.pose.position.x
    # print(p.shape)
    # z 90 erst dann x 180
    current_pos = p[1:4, current_pos_number]

    if np.sqrt(
            (msg.pose.position.x - current_pos[0]) ** 2 + (msg.pose.position.y - current_pos[1]) ** 2) < R:  # define R
        current_pos_number = current_pos_number + 1
        if current_pos_number > N - 1:
            current_pos_number = 0
        current_pos = p[1:4, current_pos_number]
    rotation_body_frame = Quaternion(w=msg.pose.orientation.w,
                                     x=msg.pose.orientation.x,
                                     y=msg.pose.orientation.y,
                                     z=msg.pose.orientation.z)
    yaw, pitch, roll = rotation_body_frame.inverse.yaw_pitch_roll
    #print("first:",yaw* 180.0 / np.pi, pitch* 180.0 / np.pi, roll* 180.0 / np.pi)
    #yaw = (-yaw - 90 / 180.0 * np.pi + 360 / 180.0 * np.pi) % (np.pi * 2) - 180 / 180.0 * np.pi
    yaw_current=-yaw
    pitch_current = -pitch
    roll_current =-( (roll+ 360 / 180.0 * np.pi) % (np.pi * 2) - 180 / 180.0 * np.pi)
    # print(roll_current * 180.0 / np.pi, pitch_current * 180.0 / np.pi, yaw_current * 180.0 / np.pi)
    # print(np.sqrt((msg.pose.position.x - current_pos[0]) ** 2 + (msg.pose.position.y - current_pos[1]) ** 2))
    # print("wanted_pos:", current_pos)
    # print("current_pos:", msg.pose.position.x, msg.pose.position.y)
    # print(msg.pose.position)
    # send_waypoint = PoseStamped()
    # send_waypoint.header.stamp = rospy.Time.now()
    # send_waypoint.header.frame_id = "global_tank"
    # send_waypoint.pose.position.x = 0.1
    # send_waypoint.pose.position.y = 0.2
    # send_waypoint.pose.position.z = 0.3
    # send_waypoint.pose.orientation.x = 0
    # send_waypoint.pose.orientation.y = 0
    # send_waypoint.pose.orientation.z = 0
    # send_waypoint.pose.orientation.w = 1

    ####TEST ANDERES POSE SENDEN############
    # send_waypoint = PoseStamped()
    # send_waypoint.header.stamp = rospy.Time.now()
    # send_waypoint.header.frame_id = "LOCAL_NED"
    # send_waypoint.pose.position.x = current_pos[0]
    # send_waypoint.pose.position.y = current_pos[1]
    # send_waypoint.pose.position.z = 0

    # 2D controller

    # yaw = np.arctan2((current_pos[1] - msg.pose.position.y), (current_pos[0] - msg.pose.position.x))
    # qz_90n = Quaternion(axis=[0, 0, 1], angle=-(yaw - np.pi / 2))
    # 3d controller CARROT CONTROL
    if carrot == 1:
        yaw2 = np.arctan2((current_pos[1] - msg.pose.position.y), (current_pos[0] - msg.pose.position.x))
        # yaw = 0 / 180.0 * np.pi
        pitch = -np.arctan((wanted_z_position - msg.pose.position.z) / distance_to_point)
        # print(yaw2)
        # print(yaw2)
        # yaw2=yaw2+np.pi
        # print("current Depth:",msg.pose.position.z)
        # print("Pitch:",pitch*180.0/np.pi)
        # print(msg.pose.position.z , wanted_z_position)
        # print("Yaw:",yaw*180.0/np.pi)
        # pitch = 0
        # print(yaw2)
        #yaw2 = 0.0 / 180.0 * np.pi
        #pitch = 0.0 / 180.0 * np.pi
        roll_old = 0.0 / 180.0 * np.pi
        # pitch=-pitch_old
        roll = roll_old
        # pitch = -pitch_old*np.cos(yaw2) + roll_old * np.sin(yaw2)
        # roll = pitch_old * np.sin(yaw2) + roll_old * np.cos(yaw2)
        # print(pitch)

        # qz_90n = Quaternion(axis=[0, 1, 0], angle=roll) * Quaternion(axis=[1, 0, 0], angle=pitch) * Quaternion(
        #    axis=[0, 0, 1], angle=-( yaw2 - np.pi / 2))
        # qz_90n = Quaternion(axis=[1, 0, 0], angle=roll) * Quaternion(axis=[0, 1, 0], angle=pitch) * Quaternion(
        #    axis=[0, 0, 1], angle=yaw2)
        qz_90n = Quaternion(
            axis=[0, 0, 1], angle=-(yaw2 - np.pi / 2)) * Quaternion(axis=[0, 1, 0], angle=-pitch) * Quaternion(
            axis=[1, 0, 0], angle=roll)
        # qz_90n = qz_90n*Quaternion(w=msg.pose.orientation.w,x=msg.pose.orientation.x,y=msg.pose.orientation.y,z=msg.pose.orientation.z)

    else:

        # 3d controller DARPA Control
        # error noch ausrechnen
        yaw = current_pos[2] + 0.5 * np.arctan2((current_pos[1] - msg.pose.position.y),
                                                (current_pos[0] - msg.pose.position.x))
        print("alpha_k:", current_pos[2], "error:",
              np.arctan2((current_pos[1] - msg.pose.position.y), (current_pos[0] - msg.pose.position.x)), "yaw:", yaw)
        # yaw = 0 / 180.0 * np.pi
        pitch = np.arctan((msg.pose.position.z - wanted_z_position) / distance_to_point)
        # pitch = 0
        # pitch = 20 / 180.0 * np.pi
        roll = 0 / 180.0 * np.pi
        qz_90n = Quaternion(axis=[0, 1, 0], angle=roll) * Quaternion(axis=[1, 0, 0], angle=pitch) * Quaternion(
            axis=[0, 0, 1], angle=-(yaw - np.pi / 2))
    # roll= 100/180.0 * np.pi
    # qz_90n = Quaternion(axis=[1, 0, 0], angle=roll)
    # send_waypoint.pose.orientation.x = 0
    # send_waypoint.pose.orientation.y = 0
    # send_waypoint.pose.orientation.z = 0
    # send_waypoint.pose.orientation.w = 1
    # send_waypoint.pose.orientation.x = qz_90n.x
    # send_waypoint.pose.orientation.y = qz_90n.y
    # send_waypoint.pose.orientation.z = qz_90n.z
    # send_waypoint.pose.orientation.w = qz_90n.w
    # print(qz_90n)

    # send_waypoint = PositionTarget()
    # send_waypoint.coordinate_frame = send_waypoint.FRAME_LOCAL_NED
    # send_waypoint.header.stamp = rospy.Time.now()
    # send_waypoint.header.frame_id = "global_tank"
    # # send_waypoint.type_mask = 4 + 8 + 16 + 32 + 64 + 128 + 256 + 512 + 2048
    # send_waypoint.position.x = current_pos[0]
    # send_waypoint.position.y = current_pos[1]
    # send_waypoint.yaw = np.pi / 4

    send_waypoint = AttitudeTarget()
    send_waypoint.type_mask = 0
    send_waypoint.orientation.x = qz_90n.x
    send_waypoint.orientation.y = qz_90n.y
    send_waypoint.orientation.z = qz_90n.z
    send_waypoint.orientation.w = qz_90n.w
    # print(qz_90n.x,qz_90n.y,qz_90n.z,qz_90n.w)
    # 0.2 works
    send_waypoint.thrust = thrust * np.cos(yaw_current - yaw2)
    if abs(yaw_current-yaw2) > np.pi/2:
        send_waypoint.thrust = 0
    publisher_waypoint.publish(send_waypoint)
    rate.sleep()
    rviz = True
    if rviz:

        r = 0.05
        markerArray = MarkerArray()
        for i in range(p.shape[1]):
            if i == current_pos_number:
                marker = Marker()
                marker.header.frame_id = "global_tank"
                marker.id = i
                marker.type = marker.SPHERE
                marker.action = marker.ADD
                marker.scale.x = r * 2  # r*2 of distance to camera from tag_14
                marker.scale.y = r * 2
                marker.scale.z = r * 2
                marker.color.r = 1
                marker.color.a = 1  # transparency
                marker.pose.orientation.w = 1.0
                marker.pose.position.x = p[1, i]  # x
                marker.pose.position.y = p[2, i]  # y
                marker.pose.position.z = wanted_z_position  # z
                markerArray.markers.append(marker)
            else:
                marker = Marker()
                marker.header.frame_id = "global_tank"
                marker.id = i
                marker.type = marker.SPHERE
                marker.action = marker.ADD
                marker.scale.x = r  # r*2 of distance to camera from tag_14
                marker.scale.y = r
                marker.scale.z = r
                marker.color.g = 1
                marker.color.a = 1  # transparency
                marker.pose.orientation.w = 1.0
                marker.pose.position.x = p[1, i]  # x
                marker.pose.position.y = p[2, i]  # y
                marker.pose.position.z = wanted_z_position  # z
                markerArray.markers.append(marker)
    publisher_marker.publish(markerArray)


def main():
    rospy.init_node('waypoint_send')
    global rate, R, wanted_z_position, distance_to_point, thrust, carrot, yaw
    rate = rospy.Rate(30)
    rate_2 = rospy.Rate(5)
    rospy.Subscriber("/mavros/local_position/pose_NED", PoseStamped, callback, queue_size=1)

    rospack=rospkg.RosPack()
    data_path = rospack.get_path("utilities_localisation")+'/scripts/parameters.csv'

    while not rospy.is_shutdown():
        # while 1:

        try:
            with open(data_path, 'r') as f:
                reader = csv.reader(f, delimiter=',')
                # get header from first row
                headers = next(reader)
                # get all the rows as a list
                data = list(reader)
                # transform data into numpy array
                data = np.array(data).astype(float)
                R, wanted_z_position, distance_to_point, thrust, carrot, yaw = data[0]
                yaw = yaw / 180 * np.pi

                # print(R)
        except:
            pass
        rate_2.sleep()


if __name__ == '__main__':
    main()