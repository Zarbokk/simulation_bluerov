//
// Created by tim on 26.03.21.
//

#include <slamToolsRos.h>

double scalingAngle = 0.05;
double scalingAllg = 0.25;
double sigmaScaling = 3;
double noiseVelocityIntigration = 0.1;

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

    Eigen::Vector3d firstPosition(0, 0, 0);
    Eigen::AngleAxisd rotationVectorFirst(0.0f / 180.0f * 3.14159f, Eigen::Vector3d(0, 0, 1));
    Eigen::Quaterniond firstRotation(rotationVectorFirst.toRotationMatrix());
    Eigen::Vector3d covariancePos(0, 0, 0);
    graphSaved.addVertex(0, firstPosition, firstRotation, covariancePos, 0,
                         cloudFirstScan,
                         graphSlamSaveStructure::INTEGRATED_POS_USAGE);//the first vertex sets 0 of the coordinate system

    std::deque<double> subgraphs{1, 3};
    graphSaved.initiallizeSubGraphs(subgraphs);

    return graphSaved;
}

int
main(int argc, char **argv) {
    std::string folderExperiment = "tmp";// folder of experiment
    ros::init(argc, argv, "slamLabyrinth01");
    ros::start();
    ros::NodeHandle n_;
    ros::Publisher publisherFirstScan,publisherAlignedScan,publisherFromScan,publisherToScan, publisherKeyFrameClouds, publisherPathOverTime, publisherMarkerArray, publisherPathOverTimeGT;
    publisherFromScan = n_.advertise<sensor_msgs::PointCloud2>("fromScan", 10);
    publisherToScan = n_.advertise<sensor_msgs::PointCloud2>("toScan", 10);
    publisherAlignedScan = n_.advertise<sensor_msgs::PointCloud2>("alignedScan", 10);
    publisherFirstScan = n_.advertise<sensor_msgs::PointCloud2>("first_scan", 10);
    publisherKeyFrameClouds = n_.advertise<sensor_msgs::PointCloud2>("currentScanTransformed", 10);
    publisherPathOverTime = n_.advertise<nav_msgs::Path>("positionOverTime", 10);
    publisherPathOverTimeGT = n_.advertise<nav_msgs::Path>("positionOverTimeGT", 10);
    publisherMarkerArray = n_.advertise<visualization_msgs::MarkerArray>("covariance", 10);
    scanRegistrationClass registrationClass = scanRegistrationClass();

    //slamToolsRos toolsObject;
    std::vector<std::vector<measurement>> groundTruthSorted;
    std::vector<std::vector<measurement>> angularVelocitySorted;
    std::vector<std::vector<measurement>> bodyVelocitySorted;
    loadCSVFiles(groundTruthSorted, angularVelocitySorted, bodyVelocitySorted, folderExperiment);

    pcl::PointCloud<pcl::PointXYZ>::Ptr currentScan(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr lastScan(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr Final(
            new pcl::PointCloud<pcl::PointXYZ>);//@TODOoutput of matching process should be deleted later
    pcl::io::loadPCDFile("/home/tim/DataForTests/" + folderExperiment + "/after_voxel_1.pcd",
                         *currentScan);

    //Matrices
    Eigen::Matrix4d currentTransformation;


    std::vector<vertex> posDiffOverTimeVertices;
    std::vector<edge> posDiffOverTimeEdges;

    double lastTimeKeyFrame = groundTruthSorted[0][0].timeStamp;//280
    double timeCurrentGroundTruth = groundTruthSorted[1][0].timeStamp;//290
    double fitnessScore;


    //graphSlamSaveStructure graphSaved = initializeGraph(currentScan);
    sensor_msgs::PointCloud2 firstScanMsg;
    pcl::toROSMsg(*currentScan, firstScanMsg);
    firstScanMsg.header.frame_id = "map_ned";
    //add first vertex
    graphSlamSaveStructure graphSaved(3);
    graphSaved.addVertex(0, Eigen::Vector3d(0, 0, 0), Eigen::Quaterniond(1, 0, 0, 0),
                         Eigen::Vector3d(0, 0, 0), 0, graphSlamSaveStructure::INTEGRATED_POS_USAGE);

    //first step
    slamToolsRos::calculatePositionOverTime(angularVelocitySorted[1], bodyVelocitySorted[1],
                                            posDiffOverTimeEdges, lastTimeKeyFrame, timeCurrentGroundTruth);


    //add vertex and so on to graphSaved
    for (auto &currentEdge : posDiffOverTimeEdges) {
        vertex lastVertex = graphSaved.getVertexList().back();
        Eigen::Matrix4d tmpTransformation = lastVertex.getTransformation();
        tmpTransformation = tmpTransformation * currentEdge.getTransformation();
        Eigen::Vector3d pos = tmpTransformation.block<3, 1>(0, 3);
        Eigen::Matrix3d rotM = tmpTransformation.block<3, 3>(0, 0);
        Eigen::Quaterniond rot(rotM);

        graphSaved.addVertex(lastVertex.getVertexNumber() + 1, pos, rot, lastVertex.getCovariancePosition(),
                             lastVertex.getCovarianceQuaternion(),graphSlamSaveStructure::INTEGRATED_POS_USAGE);
        graphSaved.addEdge(lastVertex.getVertexNumber(), lastVertex.getVertexNumber() + 1,
                           currentEdge.getPositionDifference(), currentEdge.getRotationDifference(),
                           Eigen::Vector3d(noiseVelocityIntigration, noiseVelocityIntigration, 0),
                           scalingAngle * noiseVelocityIntigration,graphSlamSaveStructure::INTEGRATED_POS_USAGE);
//        graphSaved.getVertexList().back().setTypeOfVertex(
//                graphSlamSaveStructure::INTEGRATED_POS_USAGE);//1 for vertex defined by dead reckoning
    }

    slamToolsRos::correctPointCloudByPosition(currentScan, posDiffOverTimeEdges, lastTimeKeyFrame);

    graphSaved.getVertexList().back().setPointCloud(currentScan);
    graphSaved.getVertexList().back().setTypeOfVertex(graphSlamSaveStructure::POINT_CLOUD_USAGE);

    //initialize hierachical slam
    std::deque<double> subgraphs{1, 3};
    graphSaved.initiallizeSubGraphs(subgraphs);

    for (int currentKeyFrame = 2; currentKeyFrame < groundTruthSorted.size(); currentKeyFrame++) {
        *lastScan = *currentScan;
        lastTimeKeyFrame = timeCurrentGroundTruth;
        timeCurrentGroundTruth = groundTruthSorted[currentKeyFrame][0].timeStamp;

        pcl::io::loadPCDFile(
                "/home/tim/DataForTests/" + folderExperiment + "/after_voxel_" + std::to_string(currentKeyFrame) +
                ".pcd",
                *currentScan);
        //forward calculate pose(relative)(with velocities) add edges+vertexes
        slamToolsRos::calculatePositionOverTime(angularVelocitySorted[currentKeyFrame],
                                                bodyVelocitySorted[currentKeyFrame],
                                                posDiffOverTimeEdges, lastTimeKeyFrame, timeCurrentGroundTruth);
        //sort in posDiffOverTime and calculate vertices to be added
        for (auto &currentEdge : posDiffOverTimeEdges) {
            vertex lastVertex = graphSaved.getVertexList().back();
            Eigen::Matrix4d tmpTransformation = lastVertex.getTransformation();
            tmpTransformation = tmpTransformation * currentEdge.getTransformation();
            Eigen::Vector3d pos = tmpTransformation.block<3, 1>(0, 3);
            Eigen::Matrix3d rotM = tmpTransformation.block<3, 3>(0, 0);
            Eigen::Quaterniond rot(rotM);
            graphSaved.addVertex(lastVertex.getVertexNumber() + 1, pos, rot, lastVertex.getCovariancePosition(),
                                 lastVertex.getCovarianceQuaternion(),graphSlamSaveStructure::INTEGRATED_POS_USAGE);
            graphSaved.addEdge(lastVertex.getVertexNumber(), lastVertex.getVertexNumber() + 1,
                               currentEdge.getPositionDifference(), currentEdge.getRotationDifference(),
                               Eigen::Vector3d(noiseVelocityIntigration, noiseVelocityIntigration, 0),
                               scalingAngle * noiseVelocityIntigration,graphSlamSaveStructure::INTEGRATED_POS_USAGE);
        }
        //save uncorrected scan
        pcl::PointCloud<pcl::PointXYZ>::Ptr currentScanRAW(new pcl::PointCloud<pcl::PointXYZ>);
        *currentScanRAW = *currentScan;
        //re-map point cloud
        slamToolsRos::correctPointCloudByPosition(currentScan, posDiffOverTimeEdges,
                                                  lastTimeKeyFrame);//has to be done with edges
        //Add current Scan to the graph additionally set point cloud usage true
        graphSaved.getVertexList().back().setPointCloud(currentScan);
        //graphSaved.getVertexList().back().setTypeOfVertex(graphSlamSaveStructure::POINT_CLOUD_USAGE);
        graphSaved.getVertexByIndex(graphSaved.getVertexList().size()-1)->setTypeOfVertex(graphSlamSaveStructure::POINT_CLOUD_USAGE);

        //make scan matching with last scan
        Eigen::Matrix4d initialGuessTransformation =
                graphSaved.getVertexList()[graphSaved.getVertexList().size() -
                                           9].getTransformation().inverse() *graphSaved.getVertexList().back().getTransformation();//@todo understand if 9 is correct

//        initialGuessTransformation << 1, 0, 0, 0,
//                        0, 1, 0, 0,
//                        0, 0, 1, 0,
//                        0, 0, 0, 1;
//        Eigen::Matrix4d tmpInverse = initialGuessTransformation.inverse();
//        initialGuessTransformation = tmpInverse;
        currentTransformation = registrationClass.generalizedIcpRegistration(currentScan,lastScan, Final,
                                                                             fitnessScore, initialGuessTransformation);
        std::cout << "current Fitness Score: " << sqrt(fitnessScore) << std::endl;

//        tmpInverse=currentTransformation.inverse();
//        currentTransformation=tmpInverse;

        sensor_msgs::PointCloud2 fromMsg;
        pcl::toROSMsg(*lastScan, fromMsg);
        fromMsg.header.frame_id = "map_ned";
        publisherFromScan.publish(fromMsg);

        sensor_msgs::PointCloud2 toMsg;
        pcl::toROSMsg(*currentScan, toMsg);
        toMsg.header.frame_id = "map_ned";
        publisherToScan.publish(toMsg);

        sensor_msgs::PointCloud2 alignedMsg;
        pcl::toROSMsg(*Final, alignedMsg);
        alignedMsg.header.frame_id = "map_ned";
        publisherAlignedScan.publish(alignedMsg);
        if(currentKeyFrame > -1){
            std::cout << "stop" << std::endl;

            for (int j = 0 ; j < 10 ; j++){
                *currentScan = *currentScanRAW;

    //            for (auto &currentEdge : posDiffOverTimeEdges) {//calculate current diff
    //                initialGuessTransformation = initialGuessTransformation*currentEdge.getTransformation();
    //            }
                Eigen::Matrix4d difference =initialGuessTransformation.inverse()*currentTransformation;
                //calculate 1/9th and change that //@TODO Rotation is missing
                for (auto &currentEdge : posDiffOverTimeEdges) {//calculate current diff
                    Eigen::Vector3d diffVector = currentEdge.getTransformation().block<3, 1>(0, 3)+0.5*difference.block<3, 1>(0, 3)/9.0;
                    currentEdge.setPositionDifference(diffVector);
                }

                //re-map point cloud
                slamToolsRos::correctPointCloudByPosition(currentScan, posDiffOverTimeEdges,
                                                          lastTimeKeyFrame);

                initialGuessTransformation << 1, 0, 0, 0,
                        0, 1, 0, 0,
                        0, 0, 1, 0,
                        0, 0, 0, 1;
                for (auto &currentEdge : posDiffOverTimeEdges) {//sum up transformations for initial guess
                    initialGuessTransformation = initialGuessTransformation*currentEdge.getTransformation();
                }

//                tmpInverse = initialGuessTransformation.inverse();
                currentTransformation = registrationClass.generalizedIcpRegistration(currentScan,lastScan, Final,
                                                                                     fitnessScore, initialGuessTransformation);
                std::cout << fitnessScore << std::endl;
//                tmpInverse=currentTransformation.inverse();
//                currentTransformation=tmpInverse;

                sensor_msgs::PointCloud2 fromMsg;
                pcl::toROSMsg(*lastScan, fromMsg);
                fromMsg.header.frame_id = "map_ned";
                publisherFromScan.publish(fromMsg);

                sensor_msgs::PointCloud2 toMsg;
                pcl::toROSMsg(*currentScan, toMsg);
                toMsg.header.frame_id = "map_ned";
                publisherToScan.publish(toMsg);

                sensor_msgs::PointCloud2 alignedMsg;
                pcl::toROSMsg(*Final, alignedMsg);
                alignedMsg.header.frame_id = "map_ned";
                publisherAlignedScan.publish(alignedMsg);

                std::cout << "done optimizing difference" << std::endl;
            }
            graphSaved.getVertexList().back().setPointCloud(currentScan);
        }
        //add edge for currentTransformation
        Eigen::Quaterniond qTMP(currentTransformation.block<3, 3>(0, 0));
        graphSaved.addEdge(graphSaved.getVertexList().size() - 9, graphSaved.getVertexList().size() - 1,
                           currentTransformation.block<3, 1>(0, 3), qTMP,
                           Eigen::Vector3d(sqrt(fitnessScore), sqrt(fitnessScore), 0),
                           (double) sqrt(fitnessScore),graphSlamSaveStructure::POINT_CLOUD_USAGE);//@TODO still not sure about size

        //add pointCloud to Vertex and change type
        graphSaved.getVertexList().back().setPointCloud(currentScan);
        graphSaved.getVertexList().back().setTypeOfVertex(graphSlamSaveStructure::POINT_CLOUD_USAGE);

        //watch for loop closure
        slamToolsRos::detectLoopClosure(graphSaved, registrationClass, sigmaScaling, 0.5);
        //optimization of graph
        graphSaved.optimizeGraphWithSlamTopDown(false);
        //std::vector<int> holdStill{0};
        bool calcEverythingAnew=false;//for debugging
        if(calcEverythingAnew){
            std::vector<int> holdStill{0};
            graphSaved.optimizeGraphWithSlam(false,holdStill);
        }
        //graphSaved.optimizeGraphWithSlam(false,holdStill);
        //graphSaved.calculateCovarianceInCloseProximity();
        //visualization of graph in ros

        slamToolsRos::visualizeCurrentGraph(graphSaved, publisherPathOverTime, publisherKeyFrameClouds,
                                            publisherMarkerArray, sigmaScaling, publisherPathOverTimeGT,
                                            groundTruthSorted);
    }
    std::vector<int> holdStill{0};
    graphSaved.optimizeGraphWithSlam(false,holdStill);
    slamToolsRos::visualizeCurrentGraph(graphSaved, publisherPathOverTime, publisherKeyFrameClouds,
                                        publisherMarkerArray, sigmaScaling, publisherPathOverTimeGT,
                                        groundTruthSorted);

    return (0);
}
