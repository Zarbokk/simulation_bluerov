#include <stdio.h>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include "sensor_msgs/LaserScan.h"
#include <vector>


class LaserScan2PCL
{
public:
    LaserScan2PCL(const std::string& publishName, const std::string& subscribeName)
    {
        //Topic you want to publish "cloud_topic";
        demoPublisher_ = n_.advertise<pcl::PointCloud<pcl::PointXYZ> >(publishName,10);

        //Topic you want to subscribe
        sub_ = n_.subscribe(subscribeName, 1000, &LaserScan2PCL::callback,this);
    }

    void callback(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
        printf("test: %f\n",msg->range_max);
        pcl::PointCloud<pcl::PointXYZ> myCloud;
        double angle_increment = msg->angle_increment;
        int i = 0;
        while (msg->angle_min+angle_increment*i <= msg->angle_max)
        {
            pcl::PointXYZ newPoint;
            newPoint.x = cos(msg->angle_min+angle_increment*i)*msg->ranges[i];
            newPoint.y = sin(msg->angle_min+angle_increment*i)*msg->ranges[i];
            newPoint.z = 0;
            myCloud.points.push_back(newPoint);
            i++;
        }
        sensor_msgs::PointCloud2 cloud_msg;


        pcl::toROSMsg(myCloud, cloud_msg);
        cloud_msg.header.frame_id = "ping_sonar_link_gt";
        cloud_msg.header.stamp = ros::Time::now();
        demoPublisher_.publish(cloud_msg);
    }

private:
    ros::NodeHandle n_;
    ros::Publisher demoPublisher_;
    ros::Subscriber sub_;

};//End of class SubscribeAndPublish





int main(int argc, char **argv)
{
    // setup ros for this node and get handle to ros system
    ros::init(argc, argv, "pcl_sonar_publisher");
    ros::start();

    LaserScan2PCL tmp = LaserScan2PCL("test","rrbot/laser/scan");


    ros::spin();
    return 0;
}