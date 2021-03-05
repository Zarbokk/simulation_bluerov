#!/usr/bin/env python
import numpy as np
from pyquaternion import Quaternion
import rospy
import sensor_msgs.msg
import geometry_msgs.msg
import tf2_ros
import tf2_geometry_msgs
import rospkg
import csv

publisher_waypoint = rospy.Publisher('mavros/setpoint_position/local', geometry_msgs.msg.PoseStamped, queue_size=1)
subscriber_position = rospy.Subscriber
rate = None
current_pos_number = 0


def callback(msg: geometry_msgs.msg.PoseStamped, args):
    global current_pos_number
    buffer_tf2, x_des, y_des, z_des, yaw_des, N = args

    transformation_enu_to_ned = buffer_tf2.lookup_transform(source_frame=msg.header.frame_id, target_frame="map_ned",
                                                            time=rospy.Time())

    pose_ned = tf2_geometry_msgs.do_transform_pose(msg, transformation_enu_to_ned)

    if np.sqrt(
            (pose_ned.pose.position.x - x_des[current_pos_number]) ** 2 + (
                    pose_ned.pose.position.y - y_des[current_pos_number]) ** 2 + (
                    pose_ned.pose.position.z - z_des[current_pos_number]) ** 2) < 0.4:  # define R
        current_pos_number = current_pos_number + 1
        if current_pos_number > N - 1:
            current_pos_number = 0
    print("current_pos_number", current_pos_number)

    send_waypoint = geometry_msgs.msg.PoseStamped()
    send_waypoint.header.stamp = rospy.Time.now()

    roll = 0.0 / 180.0 * np.pi
    pitch = 0.0 / 180.0 * np.pi
    yaw = yaw_des[current_pos_number]
    x_des = x_des[current_pos_number]
    y_des = y_des[current_pos_number]
    z_des = z_des[current_pos_number]

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


def main():
    rospy.init_node('drive_scenario_01')
    rospack = rospkg.RosPack()
    data_path = rospack.get_path("simulation_bluerov") + '/config/where_to_move_list.csv'
    try:
        with open(data_path, 'r') as f:
            reader = csv.reader(f, delimiter=',')
            # get header from first row
            headers = next(reader)
            # get all the rows as a list
            data = list(reader)
            # transform data into numpy array
            data = np.array(data).astype(float)
            nWaypointsBetween = 10
            dataAugmented = np.zeros(((data.shape[0]-1)*nWaypointsBetween,4))
            for i in range(data.shape[0]-1):
                dataX=np.linspace(data[i,0],data[i+1,0],nWaypointsBetween)
                dataY=np.linspace(data[i,1],data[i+1,1],nWaypointsBetween)
                dataZ=np.linspace(data[i,2],data[i+1,2],nWaypointsBetween)
                datayaw=np.linspace(data[i,3],data[i+1,3],nWaypointsBetween)
                dataAugmented[i*nWaypointsBetween:i*nWaypointsBetween+(nWaypointsBetween),0]=dataX
                dataAugmented[i*nWaypointsBetween:i*nWaypointsBetween+(nWaypointsBetween),1]=dataY
                dataAugmented[i*nWaypointsBetween:i*nWaypointsBetween+(nWaypointsBetween),2]=dataZ
                dataAugmented[i*nWaypointsBetween:i*nWaypointsBetween+(nWaypointsBetween),3]=datayaw
            dataAugmented = np.array(dataAugmented).astype(float)


            data = dataAugmented

            x_des = data[:, 0]
            y_des = data[:, 1]
            z_des = data[:, 2]
            yaw_des = data[:, 3]
            yaw_des = yaw_des / 180 * np.pi

            N = data.shape[0]
            print(yaw_des)

            # print(R)
    except:
        pass

    buffer_tf2 = tf2_ros.Buffer()
    transform_listener = tf2_ros.TransformListener(buffer_tf2)
    rospy.Subscriber('/mavros/local_position/pose', geometry_msgs.msg.PoseStamped, callback,
                     (buffer_tf2, x_des, y_des, z_des, yaw_des, N))

    rospy.spin()


if __name__ == '__main__':
    main()
