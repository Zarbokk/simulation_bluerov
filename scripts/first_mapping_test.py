#!/usr/bin/env python3

import rospy
import tf2_ros
import tf_conversions
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
import sensor_msgs.msg
import numpy as np
import tf2_geometry_msgs
import pcl

def callback_laser_scan(msg: sensor_msgs.msg.LaserScan,buffer_tf2: tf2_ros.Buffer):
    #print(msg)
    example_point = msg.ranges
    print("tmp",example_point[0])
    transform_sonar_to_base = buffer_tf2.lookup_transform("ping_sonar_link_gt", 'base_link_frd_gt', rospy.Time())
    angle_list=np.linspace(msg.angle_min,msg.angle_max,np.size(msg.ranges))
    points=np.asarray(msg.ranges)*np.asarray([np.cos(angle_list),np.sin(angle_list)])
    print("shape",points)
    pc_msg = sensor_msgs.msg.PointCloud2()
    p = pcl.PointCloud(10)





def main():
    rospy.init_node('create_mapping_by_ground_truth')
    buffer_tf2 = tf2_ros.Buffer()
    transform_listener = tf2_ros.TransformListener(buffer_tf2)






    rospy.Subscriber('/rrbot/laser/scan', sensor_msgs.msg.LaserScan, callback_laser_scan,buffer_tf2)
    rospy.spin()



if __name__ == '__main__':
    main()