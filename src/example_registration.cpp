
#include <scanRegistrationClass.h>
#include <ros/ros.h>
#include "sensor_msgs/PointCloud2.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include <pcl_conversions/pcl_conversions.h>
#include "keyFrame.h"

int
main (int argc, char** argv)
{
    ros::init(argc, argv, "exampleRegistration");
    ros::start();
    ros::NodeHandle n_;
    ros::Publisher demoPublisher_ , demoPublisher2_, demoPublisher3_;
    demoPublisher2_ = n_.advertise<sensor_msgs::PointCloud2>("first_scan",10);
    demoPublisher_ = n_.advertise<sensor_msgs::PointCloud2>("currentScanTransformed",10);
    demoPublisher3_ = n_.advertise<nav_msgs::Path>("positionOverTime",10);



    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFirstScan (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr lastScan (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr currentScan (new pcl::PointCloud<pcl::PointXYZ>);
    int i = 16;
    pcl::io::loadPCDFile("/home/tim/DataForTests/ScansOfLabyrinth/after_voxel_"+std::to_string(i-1)+".pcd",*cloudFirstScan);
    *currentScan=*cloudFirstScan;


    pcl::PointCloud<pcl::PointXYZ>::Ptr Final(new pcl::PointCloud<pcl::PointXYZ>);
    scanRegistrationClass registrationClass = scanRegistrationClass();
    Eigen::Matrix4f currentTransformation,completeTransformation;
    completeTransformation << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

    //registrationClass.addCloudToViewer(cloudFirstScan,"First cloud",1,0,0);

    sensor_msgs::PointCloud2 cloudMsg;
    pcl::toROSMsg(*cloudFirstScan,cloudMsg);
    cloudMsg.header.frame_id = "map_ned";


    pcl::PointCloud<pcl::PointXYZ> currentScanTransformed;

    nav_msgs::Path posOverTime;
    posOverTime.header.frame_id = "map_ned";
    for( ; i < 170; i=i+1 ) {
        *lastScan=*currentScan;
        pcl::io::loadPCDFile("/home/tim/DataForTests/ScansOfLabyrinth/after_voxel_"+std::to_string(i)+".pcd",*currentScan);
        currentTransformation=registrationClass.generalizedIcpRegistration(lastScan,currentScan,Final);
        completeTransformation=completeTransformation*currentTransformation;



        geometry_msgs::PoseStamped pos;
        Eigen::Quaternionf q(completeTransformation.inverse().block<3,3>(0,0));
        pos.pose.position.x = completeTransformation.inverse()(0,3);
        pos.pose.position.y = completeTransformation.inverse()(1,3);
        pos.pose.position.z = 0;
        pos.pose.orientation.x = q.x();
        pos.pose.orientation.y = q.y();
        pos.pose.orientation.z = q.z();
        pos.pose.orientation.w = q.w();

        posOverTime.poses.push_back(pos);
        demoPublisher3_.publish(posOverTime);



        pcl::transformPointCloud(*currentScan,currentScanTransformed,completeTransformation.inverse());

        sensor_msgs::PointCloud2 cloudMsg2;
        pcl::toROSMsg(currentScanTransformed,cloudMsg2);
        cloudMsg2.header.frame_id = "map_ned";
        demoPublisher_.publish(cloudMsg2);
        demoPublisher2_.publish(cloudMsg);
    }
    std::cout << "completeTransformation" << std::endl;
    std::cout << completeTransformation << std::endl;
    std::cout << "completeTransformation.inversed()" << std::endl;
    std::cout << completeTransformation.inverse() << std::endl;









    //registrationClass.addCloudToViewer(cloudFirstScan,"cloudFirstScan",1,0,0);
    //registrationClass.addCloudToViewer(currentScanTransformed,"currentScanTransformed",0,0,1);
    //registrationClass.addCloudToViewer(PositionOverTime,"PositionOverTime",1,1,1);

    //registrationClass.visualizeLoadedClouds(true);



    return (0);
}