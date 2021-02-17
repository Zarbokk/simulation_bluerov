//
// Created by tim on 12.02.21.
//
//#include <ros/ros.h>
#include <keyFrame.h>
#include <vector>
int main(int argc, char **argv)
{
    // setup ros for this node and get handle to ros system
    ros::init(argc, argv, "test_example_for_key_points");
    ros::start();

    std::vector<keyFrame> keyframeList;


    keyFrame newKeyFrame;
    pcl::PointCloud<pcl::PointXYZ> myCloud;
    for (int v=0; v<1000; ++v)
    {
        pcl::PointXYZ newPoint;
        newPoint.x = (rand() * 100.0) / RAND_MAX;
        newPoint.y = (rand() * 100.0) / RAND_MAX;
        newPoint.z = (rand() * 100.0) / RAND_MAX;
        myCloud.points.push_back(newPoint);
    }
    Eigen::Vector3f currentEstimatedPos;
    currentEstimatedPos[0]= 1;
    currentEstimatedPos[1] = 2;
    currentEstimatedPos[2] = 3;
    newKeyFrame.setPointCloud(myCloud.makeShared());
    newKeyFrame.setEstimatedPos(currentEstimatedPos);
    keyframeList.push_back(newKeyFrame);
    std::cout << keyframeList.size() << std::endl;




    keyFrame newKeyFrame2;
    pcl::PointCloud<pcl::PointXYZ> myCloud2;
    for (int v=0; v<1000; ++v)
    {
        pcl::PointXYZ newPoint;
        newPoint.x = 3+(rand() * 100.0) / RAND_MAX;
        newPoint.y = -4+(rand() * 100.0) / RAND_MAX;
        newPoint.z = 5+(rand() * 100.0) / RAND_MAX;
        myCloud2.points.push_back(newPoint);
    }
    newKeyFrame.setPointCloud(myCloud2.makeShared());
    newKeyFrame.setEstimatedPos(currentEstimatedPos);
    keyframeList.push_back(newKeyFrame);
    std::cout << keyframeList.size() << std::endl;


    ros::spin();
    return 0;
}