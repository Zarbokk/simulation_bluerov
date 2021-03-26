//
// Created by tim on 26.03.21.
//

#include <scanRegistrationClass.h>
//#include <ros/ros.h>
//#include "sensor_msgs/PointCloud2.h"
//#include "nav_msgs/Path.h"
//#include "geometry_msgs/PoseStamped.h"
//#include <pcl_conversions/pcl_conversions.h>
//#include <visualization_msgs/Marker.h>
//#include <visualization_msgs/MarkerArray.h>
#include <graphSlamSaveStructure.h>
#include <slamToolsRos.h>
#include <random>

double scalingAngle = 0.1;
double scalingAllg = 0.25;
double sigmaScaling = 3;


bool detectLoopClosure(graphSlamSaveStructure &graphSaved, scanRegistrationClass registrationClass) {
    Eigen::Vector3f estimatedPosLastPoint = graphSaved.getVertexList().back().getPositionVertex();
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudLast = graphSaved.getVertexList().back().getPointCloud();
    Eigen::ArrayXXf dist;
    dist.resize(graphSaved.getVertexList().size() - 1, 1);
    for (int s = 0; s < graphSaved.getVertexList().size() - 1; s++) {
        //dist.row(s) = (graphSaved.getVertexList()[s].getPositionVertex() - estimatedPosLastPoint).norm();
        dist.row(s) = pow((estimatedPosLastPoint.x() - graphSaved.getVertexList()[s].getPositionVertex().x()), 2) /
                      pow((sigmaScaling * graphSaved.getVertexList()[s].getCovariancePosition().x()), 2) +
                      pow((estimatedPosLastPoint.y() - graphSaved.getVertexList()[s].getPositionVertex().y()), 2) /
                      pow((sigmaScaling * graphSaved.getVertexList()[s].getCovariancePosition().y()), 2);
    }
    const int ignoreLastNLoopClosures = 2;
    std::vector<int> has2beChecked;
    if (dist.size() > ignoreLastNLoopClosures) {
        for (int i = 0; i < dist.size() - ignoreLastNLoopClosures; i++) {
            if (dist(i, 0) < 1) {
                has2beChecked.push_back(i);
            }
        }

        std::cout << "Test loop closure for :" << has2beChecked.size() << std::endl;
        std::shuffle(has2beChecked.begin(), has2beChecked.end(), std::mt19937(std::random_device()()));
//        while(has2beChecked.size()>3){
//            has2beChecked.pop_back();//just check max 3
//        }
        int loopclosureNumber = 0;
        bool foundLoopClosure = false;
        for (const auto &has2beCheckedElemenet : has2beChecked) {
            double fitnessScore;
            Eigen::Matrix4f currentTransformation;
            currentTransformation = registrationClass.generalizedIcpRegistrationSimple(
                    graphSaved.getVertexList()[has2beCheckedElemenet].getPointCloud(),
                    graphSaved.getVertexList().back().getPointCloud(),
                    fitnessScore);
            fitnessScore = scalingAllg * sqrt(fitnessScore);
            if (fitnessScore < 0.1) {
                std::cout << "Found Loop Closure with fitnessScore: " << fitnessScore << std::endl;
                if (fitnessScore < 0.01) {
                    std::cout << "FitnessScore Very Low: " << fitnessScore << std::endl;
                    fitnessScore = 0.01;
                }
                Eigen::Vector3f currentPosDiff;
                Eigen::Quaternionf currentRotDiff(currentTransformation.inverse().block<3, 3>(0, 0));
                currentPosDiff.x() = currentTransformation.inverse()(0, 3);
                currentPosDiff.y() = currentTransformation.inverse()(1, 3);
                currentPosDiff.z() = 0;
                Eigen::Vector3f positionCovariance(fitnessScore, fitnessScore, 0);
                graphSaved.addEdge(has2beCheckedElemenet, (int) graphSaved.getVertexList().size() - 1, currentPosDiff,
                                   currentRotDiff, positionCovariance, (float) (0.1 * fitnessScore));
                foundLoopClosure = true;
                loopclosureNumber++;
                if (loopclosureNumber > 1) { break; }// break if multiple loop closures are found
            }
        }
        if (foundLoopClosure) {
            return true;
        }

    }
    return false;
}

void loadCSVFiles(std::vector<std::vector<measurement>> &groundTruthSorted,
                  std::vector<std::vector<measurement>> &angularVelocitySorted,
                  std::vector<std::vector<measurement>> &bodyVelocitySorted, std::string &folderExperiment) {


    std::ifstream fileGroundTruth("/home/tim/DataForTests/" + folderExperiment + "/groundTruth.csv");
    std::ifstream fileAngularVelocity("/home/tim/DataForTests/" + folderExperiment + "/angularVelocity.csv");
    std::ifstream fileBodyVelocity("/home/tim/DataForTests/" + folderExperiment + "/bodyVelocity.csv");

    std::vector<measurement> groundTruth = slamToolsRos::parseCSVFile(fileGroundTruth);
    std::vector<measurement> angularVelocity = slamToolsRos::parseCSVFile(fileAngularVelocity);
    std::vector<measurement> bodyVelocity = slamToolsRos::parseCSVFile(fileBodyVelocity);
    groundTruthSorted = slamToolsRos::sortToKeyframe(groundTruth);
    angularVelocitySorted = slamToolsRos::sortToKeyframe(angularVelocity);
    angularVelocitySorted.pop_back();
    bodyVelocitySorted = slamToolsRos::sortToKeyframe(bodyVelocity);
    bodyVelocitySorted.pop_back();

}

graphSlamSaveStructure initializeGraph(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFirstScan) {
    const int dimension = 3;
    graphSlamSaveStructure graphSaved(dimension);

    Eigen::Vector3f firstPosition(0, 0, 0);
    Eigen::AngleAxisf rotationVectorFirst(0.0f / 180.0f * 3.14159f, Eigen::Vector3f(0, 0, 1));
    Eigen::Quaternionf firstRotation(rotationVectorFirst.toRotationMatrix());
    Eigen::Vector3f covariancePos(0, 0, 0);
    graphSaved.addVertex(0, firstPosition, firstRotation, covariancePos, 0,
                         cloudFirstScan);//the first vertex sets 0 of the coordinate system

    std::deque<float> subgraphs{1, 3};
    graphSaved.initiallizeSubGraphs(subgraphs);

    return graphSaved;
}

int
main(int argc, char **argv) {
    std::string folderExperiment = "tmp";// folder of experiment
    ros::init(argc, argv, "slamLabyrinth01");
    ros::start();
    ros::NodeHandle n_;
    ros::Publisher publisherFirstScan, publisherKeyFrameClouds, publisherPathOverTime, publisherMarkerArray;
    publisherFirstScan = n_.advertise<sensor_msgs::PointCloud2>("first_scan", 10);
    publisherKeyFrameClouds = n_.advertise<sensor_msgs::PointCloud2>("currentScanTransformed", 10);
    publisherPathOverTime = n_.advertise<nav_msgs::Path>("positionOverTime", 10);
    publisherMarkerArray = n_.advertise<visualization_msgs::MarkerArray>("covariance", 10);

    //slamToolsRos toolsObject;
    std::vector<std::vector<measurement>> groundTruthSorted;
    std::vector<std::vector<measurement>> angularVelocitySorted;
    std::vector<std::vector<measurement>> bodyVelocitySorted;
    loadCSVFiles(groundTruthSorted, angularVelocitySorted, bodyVelocitySorted, folderExperiment);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFirstScan(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("/home/tim/DataForTests/"+folderExperiment+"/after_voxel_1.pcd",
                         *cloudFirstScan);
    //correct cloud Scan


    graphSlamSaveStructure graphSaved = initializeGraph(cloudFirstScan);
    sensor_msgs::PointCloud2 firstScanMsg;
    pcl::toROSMsg(*cloudFirstScan, firstScanMsg);
    firstScanMsg.header.frame_id = "map_ned";
    float lastTimeKeyFrame = groundTruthSorted[1][0].timeStamp;
    for (int currentKeyFrame = 2; currentKeyFrame < groundTruthSorted.size(); currentKeyFrame++) {
        float timeCurrentGroundTruth = groundTruthSorted[currentKeyFrame][0].timeStamp;
        //forward calculate pose(relative)(with velocities) add edges+vertexes
        for(int i = 0; i<bodyVelocitySorted[currentKeyFrame].size();i++){//calculate that for every second

        }

        for(int i = 0; i<angularVelocitySorted[currentKeyFrame].size();i++){//calculate that for every second

        }
        //re-map point cloud
        //make scan matching with last scan
        lastTimeKeyFrame = timeCurrentGroundTruth;
    }

    return (0);
}
