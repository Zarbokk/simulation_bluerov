#include <stdio.h>
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Imu.h"
#include "gazebo_msgs/ModelStates.h"
#include "geometry_msgs/TwistStamped.h"
#include <tf/transform_listener.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <fstream>

class dataGenerationClass {
public:
    dataGenerationClass(std::string folderName) {
        //Topic you want to subscribe
        subPCL_ = n_.subscribe("sonar/full_scan", 1000, &dataGenerationClass::callbackPCL, this);
        subIMU_ = n_.subscribe("mavros/imu/data", 1000, &dataGenerationClass::callbackIMU, this);
        subVel_ = n_.subscribe("mavros/local_position/velocity_body", 1000, &dataGenerationClass::callbackvelocityBody,
                               this);
        subPos_ = n_.subscribe("gazebo/model_states", 1000, &dataGenerationClass::callbackPOSGT, this);
        this->folder = folderName;

        gtFile.open("/home/tim/DataForTests/" + this->folder + "/groundTruth.csv");
        gtFile << "keyframe,x,y,z,timestamp\n";
        gtFile.close();
        gtFile.open("/home/tim/DataForTests/" + this->folder + "/groundTruth.csv", std::fstream::app);

        bodyVelocityFile.open("/home/tim/DataForTests/" + this->folder + "/bodyVelocity.csv");
        bodyVelocityFile << "keyframe,x_vel,y_vel,z_vel,timestamp\n";
        bodyVelocityFile.close();
        bodyVelocityFile.open("/home/tim/DataForTests/" + this->folder + "/bodyVelocity.csv", std::fstream::app);

        angularVelocityFile.open("/home/tim/DataForTests/" + this->folder + "/angularVelocity.csv");
        angularVelocityFile << "keyframe,phi_vel,theta_vel,psi_vel,timestamp\n";
        angularVelocityFile.close();
        angularVelocityFile.open("/home/tim/DataForTests/" + this->folder + "/angularVelocity.csv", std::fstream::app);

    }

    void callbackPCL(const sensor_msgs::PointCloud2ConstPtr &msg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr myCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *myCloud);
        pcl::io::savePCDFileASCII(
                "/home/tim/DataForTests/" + this->folder + "/after_voxel_" + std::to_string(this->currentKeyFrame) +
                ".pcd",
                *myCloud);
        std::cout << "saving Keyframe number: " << this->currentKeyFrame << std::endl;
        //std::ofstream myfile;


        gtFile << this->currentKeyFrame << "," << this->currentGTPosition.x() << "," << this->currentGTPosition.y()
               << "," << this->currentGTPosition.z() << "," << msg->header.stamp << "\n";

        this->currentKeyFrame++;
    }

    void callbackIMU(const sensor_msgs::Imu &msg) {
        this->currentAngleVelocity.x() = msg.angular_velocity.x;
        this->currentAngleVelocity.y() = msg.angular_velocity.y;
        this->currentAngleVelocity.z() = msg.angular_velocity.z;
        angularVelocityFile << this->currentKeyFrame << "," << this->currentAngleVelocity.x() << ","
                            << this->currentAngleVelocity.y() << "," << this->currentAngleVelocity.z() << ","
                            << msg.header.stamp << "\n";
    }

    void callbackvelocityBody(const geometry_msgs::TwistStamped &msg) {
        this->currentBodyVelocity.x() = msg.twist.linear.x;
        this->currentBodyVelocity.y() = msg.twist.linear.y;
        this->currentBodyVelocity.z() = msg.twist.linear.z;
        bodyVelocityFile << this->currentKeyFrame << "," << this->currentBodyVelocity.x() << ","
                         << this->currentBodyVelocity.y() << "," << this->currentBodyVelocity.z() << ","
                         << msg.header.stamp << "\n";

    }

    void callbackPOSGT(const gazebo_msgs::ModelStates &msg) {
        int indexPose = -1;
        for (int i = 0; i < msg.name.size(); i++) {
            if (msg.name[i] == "uuv_bluerov2_heavy") {
                indexPose = i;
                break;
            }
        }
        this->currentGTPosition.x() = msg.pose[indexPose].position.x;
        this->currentGTPosition.y() = msg.pose[indexPose].position.y;
        this->currentGTPosition.z() = msg.pose[indexPose].position.z;
    }

private:
    ros::NodeHandle n_;
    ros::Publisher demoPublisher_, demoPublisher2_;
    ros::Subscriber subPCL_, subIMU_, subVel_, subPos_;
    //pcl::PointCloud<pcl::PointXYZ>::Ptr currentNoncorrectedPCL;
    tf::TransformListener tf_listener;
    int currentKeyFrame = 0;
    Eigen::Vector3f currentGTPosition;
    std::ofstream gtFile;
    Eigen::Vector3f currentBodyVelocity;
    std::ofstream bodyVelocityFile;
    Eigen::Vector3f currentAngleVelocity;
    std::ofstream angularVelocityFile;
    std::string folder;
};//End of class SubscribeAndPublish





int main(int argc, char **argv) {
    // setup ros for this node and get handle to ros system
    ros::init(argc, argv, "pmap_marker_publisher");
    ros::start();

    dataGenerationClass tmp = dataGenerationClass("tmp");


    ros::spin();
    return 0;
}


