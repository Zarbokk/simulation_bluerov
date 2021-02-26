
#include <scanRegistrationClass.h>
#include <ros/ros.h>
#include "sensor_msgs/PointCloud2.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <graphSlamSaveStructure.h>


void visualizeCurrentGraph(graphSlamSaveStructure &graphSaved, ros::Publisher &publisherPath,
                           ros::Publisher &publisherCloud, ros::Publisher &publisherMarkerArray) {

    nav_msgs::Path posOverTime;
    posOverTime.header.frame_id = "map_ned";
    Eigen::Matrix4f currentTransformation, completeTransformation;
    pcl::PointCloud<pcl::PointXYZ> completeCloudWithPos;
    visualization_msgs::MarkerArray markerArray;
    int i = 0;
    for (const auto &keyFrameElement : keyframeList) {

        pcl::PointCloud<pcl::PointXYZ> currentScanTransformed;

        completeTransformation << 1, 0, 0, keyFrameElement.getEstimatedPos().x(),
                0, 1, 0, keyFrameElement.getEstimatedPos().y(),
                0, 0, 1, keyFrameElement.getEstimatedPos().z(),
                0, 0, 0, 1;//transformation missing currently
        Eigen::Matrix3f m(keyFrameElement.getEstimatedRotation());
        completeTransformation.block<3, 3>(0, 0) = m;
        pcl::transformPointCloud(*keyFrameElement.getPointCloud(), currentScanTransformed, completeTransformation);
        completeCloudWithPos += currentScanTransformed;


        geometry_msgs::PoseStamped pos;
        pos.pose.position.x = keyFrameElement.getEstimatedPos().x();
        pos.pose.position.y = keyFrameElement.getEstimatedPos().y();
        pos.pose.position.z = keyFrameElement.getEstimatedPos().z();
        pos.pose.orientation.x = keyFrameElement.getEstimatedRotation().x();
        pos.pose.orientation.y = keyFrameElement.getEstimatedRotation().y();
        pos.pose.orientation.z = keyFrameElement.getEstimatedRotation().z();
        pos.pose.orientation.w = keyFrameElement.getEstimatedRotation().w();

        posOverTime.poses.push_back(pos);

        visualization_msgs::Marker currentMarker;
        currentMarker.pose.position.x = keyFrameElement.getEstimatedPos().x();
        currentMarker.pose.position.y = keyFrameElement.getEstimatedPos().y();
        currentMarker.pose.position.z = keyFrameElement.getEstimatedPos().z();
        currentMarker.pose.orientation.w = 1;
        currentMarker.header.frame_id = "map_ned";
        currentMarker.scale.x = 0.1 * keyFrameElement.getCovarianzPos().x();
        currentMarker.scale.y = 0.1 * keyFrameElement.getCovarianzPos().y();
        currentMarker.scale.z = 0;
        currentMarker.color.r = 0;
        currentMarker.color.g = 1;
        currentMarker.color.b = 0;
        currentMarker.color.a = 0.1;
        currentMarker.lifetime.sec = 2;

        currentMarker.type = 2;
        currentMarker.id = i;
        i++;

        markerArray.markers.push_back(currentMarker);


    }


    publisherPath.publish(posOverTime);

    sensor_msgs::PointCloud2 cloudMsg2;
    pcl::toROSMsg(completeCloudWithPos, cloudMsg2);
    cloudMsg2.header.frame_id = "map_ned";
    publisherCloud.publish(cloudMsg2);

    publisherMarkerArray.publish(markerArray);

}

void detectLoopClosure(graphSlamSaveStructure &graphSaved,scanRegistrationClass registrationClass) {
    Eigen::Vector3f estimatedPosLastPoint = keyframeList.back().getEstimatedPos();
    Eigen::Vector3f estimatedCovarianz = keyframeList.back().getEstimatedPos();
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudLast = keyframeList.back().getPointCloud();
    Eigen::ArrayXXf dist;
    dist.resize(keyframeList.size() - 1, 1);
    for (int s = 0; s < keyframeList.size() - 1; s++) {
        dist.row(s) = (keyframeList[s].getEstimatedPos() - estimatedPosLastPoint).norm();
    }
    const int ignoreLastNLoopClosures = 4;
    std::vector<int> has2beChecked;
    if (dist.size()>ignoreLastNLoopClosures) {
        for (int i = 0; i < dist.size() - ignoreLastNLoopClosures; i++) {
            if ((dist(i,0) - 1 * estimatedCovarianz.norm())<0){
                has2beChecked.push_back(i);
            }
        }
        std::cout << "loop closure for:" << std::endl;
        for (const auto &has2beCheckedElemenet : has2beChecked) {
            double fitnessScore;
            Eigen::Matrix4f currentTransformation;
            currentTransformation = registrationClass.generalizedIcpRegistrationSimple(keyframeList[has2beCheckedElemenet].getPointCloud(), keyframeList.back().getPointCloud(), fitnessScore);

            if (fitnessScore<1){
                std::cout << "fitnessScore" << std::endl;
                std::cout << fitnessScore << std::endl;
                std::cout << "has2beCheckedElemenet" << std::endl;
                std::cout << has2beCheckedElemenet << std::endl;
                std::cout << "dist.size()" << std::endl;
                std::cout << dist.size() << std::endl;
            }


            //keyframeList[has2beChecked[0]].getPointCloud()

        }
    }
}


int
main(int argc, char **argv) {
    ros::init(argc, argv, "exampleRegistration");
    ros::start();
    ros::NodeHandle n_;
    ros::Publisher publisherKeyFrameClouds, publisherFirstScan, publisherPathOverTime, publisherMarkerArray;
    publisherFirstScan = n_.advertise<sensor_msgs::PointCloud2>("first_scan", 10);
    publisherKeyFrameClouds = n_.advertise<sensor_msgs::PointCloud2>("currentScanTransformed", 10);
    publisherPathOverTime = n_.advertise<nav_msgs::Path>("positionOverTime", 10);
    publisherMarkerArray = n_.advertise<visualization_msgs::MarkerArray>("covariance", 10);


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFirstScan(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr lastScan(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr currentScan(new pcl::PointCloud<pcl::PointXYZ>);
    int i = 16;
    pcl::io::loadPCDFile("/home/tim/DataForTests/ScansOfLabyrinth/after_voxel_" + std::to_string(i - 1) + ".pcd",
                         *cloudFirstScan);
    *currentScan = *cloudFirstScan;


    pcl::PointCloud<pcl::PointXYZ>::Ptr Final(new pcl::PointCloud<pcl::PointXYZ>);
    scanRegistrationClass registrationClass = scanRegistrationClass();
    Eigen::Matrix4f currentTransformation, completeTransformation;
    completeTransformation <<   1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

    //initialize first scan(for viewer only)
    sensor_msgs::PointCloud2 cloudMsg;
    pcl::toROSMsg(*cloudFirstScan, cloudMsg);
    cloudMsg.header.frame_id = "map_ned";


    std::vector<keyFrame> keyframeList;
    keyFrame currentFrame;
    currentFrame.setPointCloud(cloudFirstScan);
    Eigen::Vector3f currentEstPos, currentPosCov;
    currentEstPos.x() = 0;
    currentEstPos.y() = 0;
    currentEstPos.z() = 0;
    currentFrame.setEstimatedPos(currentEstPos);

    currentPosCov.x() = 0;
    currentPosCov.y() = 0;
    currentPosCov.z() = 0;
    currentFrame.setCovarianzPos(currentPosCov);
    Eigen::Quaternionf tmpRotation(completeTransformation.inverse().block<3, 3>(0, 0));
    currentFrame.setEstimatedRotation(tmpRotation);

    keyframeList.push_back(currentFrame);
    double fitnessScore;
    for (; i < 170; i = i + 1) {
        keyFrame currentFrameForLoop;
        *lastScan = *currentScan;
        pcl::io::loadPCDFile("/home/tim/DataForTests/ScansOfLabyrinth/after_voxel_" + std::to_string(i) + ".pcd",
                             *currentScan);
        currentTransformation = registrationClass.generalizedIcpRegistration(lastScan, currentScan, Final,
                                                                             fitnessScore);
        completeTransformation = completeTransformation * currentTransformation;


        geometry_msgs::PoseStamped pos;
        Eigen::Quaternionf q(completeTransformation.inverse().block<3, 3>(0, 0));

        currentEstPos.x() = completeTransformation.inverse()(0, 3);
        currentEstPos.y() = completeTransformation.inverse()(1, 3);
        currentEstPos.z() = 0;
        currentFrameForLoop.setEstimatedPos(currentEstPos);

        currentPosCov.x() = keyframeList[keyframeList.size() - 1].getCovarianzPos().x() + fitnessScore;
        currentPosCov.y() = keyframeList[keyframeList.size() - 1].getCovarianzPos().y() + fitnessScore;
        currentPosCov.z() = 0.0;
        currentFrameForLoop.setCovarianzPos(currentPosCov);

        currentFrameForLoop.setEstimatedRotation(q);
        currentFrameForLoop.setPointCloud(currentScan);

        keyframeList.push_back(currentFrameForLoop);

        visualizeKeyFrameList(keyframeList, publisherPathOverTime, publisherKeyFrameClouds, publisherMarkerArray);
        publisherFirstScan.publish(cloudMsg);//publish first scan always



        detectLoopClosure(keyframeList,registrationClass);//test loop closure
        std::cout << "###############################END OF LOOP###############################" << std::endl;
    }


    std::cout << "completeTransformation" << std::endl;
    std::cout << completeTransformation << std::endl;
    std::cout << "completeTransformation.inversed()" << std::endl;
    std::cout << completeTransformation.inverse() << std::endl;


    return (0);
}