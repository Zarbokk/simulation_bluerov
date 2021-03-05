#include <stdio.h>
#include <ros/ros.h>
//#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include "sensor_msgs/LaserScan.h"
#include <tf/transform_listener.h>
//#include <pcl_ros/filters/voxel_grid.h>
//#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
class PCL_2_Marker
{
public:
    PCL_2_Marker(const std::string& publishName, const std::string& subscribeName)
    {
        //Topic you want to publish "cloud_topic";
        demoPublisher_ = n_.advertise<sensor_msgs::PointCloud2> (publishName,10);
        demoPublisher2_ = n_.advertise<sensor_msgs::PointCloud2>("debug_pc",10);
        //Topic you want to subscribe
        sub_ = n_.subscribe(subscribeName, 1000, &PCL_2_Marker::callback,this);
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZ>);
        current_map = tmp;
    }
    void downsample(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_input, pcl::PointCloud<pcl::PointXYZ>::Ptr &output) {
        //sor.setSaveLeafLayout(true);
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud_input);
        sor.setLeafSize(0.10f, 0.10f, 0.10f);
        sor.filter(*output);
        //display or do something else with output
    }
    void callback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        tf::StampedTransform transform;
        try{
            tf_listener.lookupTransform("/map_ned", "/ping_sonar_link_gt",
                                     ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
        //transform.tr
        //pcl::transformPointCloud();
        Eigen::Quaternionf quat_rot;
        Eigen::Matrix<float,3,1> shift;
        quat_rot.x() =  transform.getRotation().getX();
        quat_rot.y() =  transform.getRotation().getY();
        quat_rot.z() =  transform.getRotation().getZ();
        quat_rot.w() =  transform.getRotation().getW();

        shift(0) = transform.getOrigin().x();
        shift(1) = transform.getOrigin().y();
        shift(2) = transform.getOrigin().z();

        //Eigen::Matrix<float,3,3> rotation_matrix = quat_rot.toRotationMatrix();
        //std::cout << rotation_matrix << std::endl;
        //Eigen::Vector3f ea = rotation_matrix.eulerAngles(0, 1, 2);
        //std::cout << "to Euler angles:" << std::endl;
        //std::cout << ea << std::endl << std::endl;

        //printf("x1:%f \n",transform.getOrigin().x());

        pcl::PointCloud<pcl::PointXYZ>::Ptr myCloud (new pcl::PointCloud<pcl::PointXYZ>);
        //printf("x2:%f \n",transform.getOrigin().x());
        pcl::fromROSMsg(*msg,*myCloud);

        pcl::io::savePCDFileASCII ("/home/tim/DataForTests/ScansOfLabyrinth/after_voxel_"+std::to_string(current_number_save)+".pcd", *myCloud);
        current_number_save++;
        std::cout << "saving number: "<< current_number_save << std::endl;

        pcl::transformPointCloud(*myCloud,*myCloud,shift,quat_rot);


//        sensor_msgs::PointCloud2 tmp;
//        pcl::toROSMsg(*myCloud, tmp);
//        tmp.header.frame_id = "map_ned";
//        tmp.header.stamp = ros::Time::now();
//        demoPublisher_.publish(tmp);
//        ros::Duration(0.1).sleep();

//        printf("current size Map before adding: %d \n",(*current_map).size());
//        printf("current size scan before adding: %d \n",(*myCloud).size());
//        int d = (*current_map).size();


        //*current_map+= *myCloud;
//        printf("current size Map after adding: %d \n",(*current_map).size());
        //printf("x4:%f \n",transform.getOrigin().x());
//        printf("current size Map: %d \n",(*current_map).size());
        //downsample the current map
        //pcl::PointCloud<pcl::PointXYZ> tmp = *current_map;
        //(*viewer).showCloud(current_map);

//        sensor_msgs::PointCloud2 cloud_msg_tmp;
//
//
//        pcl::toROSMsg(*current_map, cloud_msg_tmp);
//        cloud_msg_tmp.header.frame_id = "map_ned";
//        cloud_msg_tmp.header.stamp = ros::Time::now();
//        demoPublisher2_.publish(cloud_msg_tmp);



        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudvoxel_output (new pcl::PointCloud<pcl::PointXYZ>);
//        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudvoxel_output2 (new pcl::PointCloud<pcl::PointXYZ>);
//        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudvoxel_save (new pcl::PointCloud<pcl::PointXYZ>);
//        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudvoxel_input (new pcl::PointCloud<pcl::PointXYZ>);
//        *cloudvoxel_save = *current_map;

//        pcl::io::savePCDFileASCII ("before_voxel.pcd", *cloudvoxel_save);
//        pcl::io::loadPCDFile("/home/tim/catkin_ws/devel/lib/simulation_bluerov/before_voxel.pcd",*cloudvoxel_input);

//        printf("cloudvoxel_save number points %d \n",(*cloudvoxel_save).size());
//        printf("cloudvoxel_input number points %d \n",(*cloudvoxel_input).size());
        //cloudvoxel_save->is_dense = true;
//        printf("print dense save%d\n",cloudvoxel_save->is_dense);
//        printf("print dense input%d\n",cloudvoxel_input->is_dense);
        //downsample(current_map,cloudvoxel_output);
//        printf("cloudvoxel_output number points(with cloudvoxel_save) %d \n",(*cloudvoxel_output).size());
//        int tmp_d = (*cloudvoxel_output).size();

//        downsample(cloudvoxel_input,cloudvoxel_output2);
//        printf("cloudvoxel_output number points(with cloudvoxel_input) %d \n",(*cloudvoxel_output2).size());

        current_map = cloudvoxel_output;

//        if (d>tmp_d){
//            d=d-1;
//        }

//        printf("current size Map after voxel: %d \n",(*cloudvoxel_output).size());

        //(*current_map).viewer

        sensor_msgs::PointCloud2 cloud_msg;


        pcl::toROSMsg(*current_map, cloud_msg);
        cloud_msg.header.frame_id = "map_ned";
        cloud_msg.header.stamp = ros::Time::now();
        demoPublisher_.publish(cloud_msg);

//        if (d>(*current_map).size()){
//            d=d-1;
//
//        }
    }

private:
    ros::NodeHandle n_;
    ros::Publisher demoPublisher_ , demoPublisher2_;
    ros::Subscriber sub_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_map;
    tf::TransformListener tf_listener;
    int current_number_save = 0;

};//End of class SubscribeAndPublish





int main(int argc, char **argv)
{
    // setup ros for this node and get handle to ros system
    ros::init(argc, argv, "pmap_marker_publisher");
    ros::start();

    PCL_2_Marker tmp = PCL_2_Marker("marker/map","test");


    ros::spin();
    return 0;
}