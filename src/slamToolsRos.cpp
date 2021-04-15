//
// Created by tim on 26.03.21.
//

#include "slamToolsRos.h"

void slamToolsRos::visualizeCurrentGraph(graphSlamSaveStructure &graphSaved, ros::Publisher &publisherPath,
                                         ros::Publisher &publisherCloud, ros::Publisher &publisherMarkerArray,
                                         double sigmaScaling, ros::Publisher &publisherPathGT,
                                         std::vector<std::vector<measurement>> &groundTruthSorted) {

    nav_msgs::Path posOverTime;
    posOverTime.header.frame_id = "map_ned";
    Eigen::Matrix4d currentTransformation, completeTransformation;
    pcl::PointCloud<pcl::PointXYZ> completeCloudWithPos;
    visualization_msgs::MarkerArray markerArray;
    int i = 0;
    std::vector<vertex> vertexList = graphSaved.getVertexList();
    for (const auto &vertexElement : vertexList) {

        pcl::PointCloud<pcl::PointXYZ> currentScanTransformed;
        completeTransformation << 1, 0, 0, vertexElement.getPositionVertex().x(),
                0, 1, 0, vertexElement.getPositionVertex().y(),
                0, 0, 1, vertexElement.getPositionVertex().z(),
                0, 0, 0, 1;//transformation missing currently
        Eigen::Matrix3d m(vertexElement.getRotationVertex().toRotationMatrix());
        completeTransformation.block<3, 3>(0, 0) = m;
        pcl::transformPointCloud(*vertexElement.getPointCloud(), currentScanTransformed, completeTransformation);
        completeCloudWithPos += currentScanTransformed;


        geometry_msgs::PoseStamped pos;
        pos.pose.position.x = vertexElement.getPositionVertex().x();
        pos.pose.position.y = vertexElement.getPositionVertex().y();
        pos.pose.position.z = vertexElement.getPositionVertex().z();
        pos.pose.orientation.x = vertexElement.getRotationVertex().x();
        pos.pose.orientation.y = vertexElement.getRotationVertex().y();
        pos.pose.orientation.z = vertexElement.getRotationVertex().z();
        pos.pose.orientation.w = vertexElement.getRotationVertex().w();

        posOverTime.poses.push_back(pos);

        visualization_msgs::Marker currentMarker;
        currentMarker.pose.position.x = pos.pose.position.x;
        currentMarker.pose.position.y = pos.pose.position.y;
        currentMarker.pose.position.z = pos.pose.position.z;
        currentMarker.pose.orientation.w = 1;
        currentMarker.header.frame_id = "map_ned";
        currentMarker.scale.x = sigmaScaling *
                                2 *
                                vertexElement.getCovariancePosition()[0];//missing covarianz since its in edge and only available after graph optimization
        currentMarker.scale.y = sigmaScaling *
                                2 *
                                vertexElement.getCovariancePosition()[1];//missing covarianz since its in edge and only available after graph optimization
        currentMarker.scale.z = 0;
        currentMarker.color.r = 0;
        currentMarker.color.g = 1;
        currentMarker.color.b = 0;
        currentMarker.color.a = 0.1;
        //currentMarker.lifetime.sec = 10;

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
    int numberOfKeyframes = (int)(graphSaved.getVertexList().size()/9)+1;
    //calculate path GT
    nav_msgs::Path posOverTimeGT;
    posOverTimeGT.header.frame_id = "map_ned";
    for(int i = 0 ; i <numberOfKeyframes;i++){
        for(auto &posList:groundTruthSorted[i]){
            geometry_msgs::PoseStamped pos;
            pos.pose.position.x = posList.y-groundTruthSorted[0][0].y;
            pos.pose.position.y = -(posList.x-groundTruthSorted[0][0].x);
            pos.pose.position.z = 0;
            pos.pose.orientation.x = 0;
            pos.pose.orientation.y = 0;
            pos.pose.orientation.z = 0;
            pos.pose.orientation.w = 1;
            posOverTimeGT.poses.push_back(pos);
        }

    }
    publisherPathGT.publish(posOverTimeGT);

}

std::vector<measurement> slamToolsRos::parseCSVFile(std::istream &stream) {
    std::vector<measurement> returnVector;

    std::string firstLine;
    std::getline(stream, firstLine);

    //std::stringstream          lineStream(line);
    std::string cell;


    for (std::string line; std::getline(stream, line);) {
        std::stringstream lineStream(line);
        std::vector<std::string> result;
        while (std::getline(lineStream, cell, ',')) {
            result.push_back(cell);
            //std::cout << cell << std::endl;
        }
        measurement tmpMeas{};
        tmpMeas.keyframe = std::stoi(result[0]);
        tmpMeas.x = std::stof(result[1]);
        tmpMeas.y = std::stof(result[2]);
        tmpMeas.z = std::stof(result[3]);
        tmpMeas.timeStamp = std::stof(result[4]);
        returnVector.push_back(tmpMeas);
    }
    return returnVector;
}

std::vector<std::vector<measurement>> slamToolsRos::sortToKeyframe(std::vector<measurement> &input) {
    int currentKeyframe = input[0].keyframe;
    std::vector<std::vector<measurement>> output;
    std::vector<measurement> tmp1;
    output.push_back(tmp1);
    for (auto currentMeasurement:input) {
        if (currentMeasurement.keyframe != currentKeyframe) {//new keyframe reached
            std::vector<measurement> tmp;
            currentKeyframe = currentMeasurement.keyframe;
            output.push_back(tmp);
            output[currentKeyframe].push_back(currentMeasurement);
        } else {
            output[currentKeyframe].push_back(currentMeasurement);
        }
    }
    return output;
}

void
slamToolsRos::correctPointCloudByPosition(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudScan, std::vector<edge> &posDiff,
                                          double timestepBeginning) {
    //this is assuming we are linear in time
    //resulting scan should be at end position means: point 0 is transformed by diff of pos 0 to n

    struct pclAngle {
        pcl::PointXYZ pointXyz;
        double angle{};
    };

    //calculate angle of every point
    std::vector<pclAngle> vectorOfPointsAngle;
    for (int i = 0; i < cloudScan->size(); i++) {//@TODO has to be corrected Last Point is always a positive angle and not under 2 pi
        double currentAngle = fmod(2 * (double) M_PI + atan2(cloudScan->points[i].y, cloudScan->points[i].x), 2 * M_PI);
        pclAngle tmpPCL;
        tmpPCL.pointXyz = cloudScan->points[i];
        tmpPCL.angle = currentAngle;
        vectorOfPointsAngle.push_back(tmpPCL);
    }
    //sort from 0 to 2pi
    std::sort(vectorOfPointsAngle.begin(), vectorOfPointsAngle.end(),
              [](const auto &i, const auto &j) { return i.angle < j.angle; });


    //calculate number of points transformations defined by edge list
    std::vector<Eigen::Matrix4d> listOfTransormations;
    double startTime = timestepBeginning;
    double endTime = posDiff.back().getTimeStamp();
    std::vector<double> timeStepsForCorrection = slamToolsRos::linspace(startTime, endTime, vectorOfPointsAngle.size());
    for (int i = 0; i < timeStepsForCorrection.size(); i++) {
        Eigen::Matrix4d currentTransformation;
        currentTransformation << 1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1;
        int j = (int) posDiff.size() - 1;
        while (timeStepsForCorrection[i] < posDiff[j].getTimeStamp()) {
            j--;
            if (j == 0) {
                break;
            }
        }
        if (j == 0) {
            if (timeStepsForCorrection[i] == startTime) {
                //add everything
                for (int k = j; k < posDiff.size(); k++) {
                    //calculate transformation of edge
                    currentTransformation *= posDiff[k].getTransformation();
                    //add to currentTransformation
                }
            } else {
                //interpolate from timeStepsForCorrection[i] to posDiff[j]

                double fromTimeStep = timeStepsForCorrection[i];
                double toTimeStep = posDiff[j].getTimeStamp();
                //interpolate posDiff[j] from to
                Eigen::Matrix4d firstTransformation = posDiff[j].getTransformation();
                double interpolationFactor =
                        (toTimeStep - fromTimeStep) / (posDiff[j].getTimeStamp() - startTime);
                firstTransformation.block<3, 1>(0, 3) = interpolationFactor * firstTransformation.block<3, 1>(0, 3);
                Eigen::Matrix3d rotationMatrix = firstTransformation.block<3, 3>(0, 0);
                Eigen::Vector3d eulerAngles = rotationMatrix.eulerAngles(0, 1, 2);
                eulerAngles = interpolationFactor * eulerAngles;

                Eigen::Matrix3d tmp;
                tmp = Eigen::AngleAxisd(eulerAngles[0], Eigen::Vector3d::UnitX()) *
                      Eigen::AngleAxisd(eulerAngles[1], Eigen::Vector3d::UnitY()) *
                      Eigen::AngleAxisd(eulerAngles[2], Eigen::Vector3d::UnitZ());
                firstTransformation.block<3, 3>(0, 0) = tmp;
                currentTransformation *= firstTransformation;
                //add rest
                for (int k = j + 1; k < posDiff.size(); k++) {
                    //calculate transformation of edge
                    currentTransformation *= posDiff[k].getTransformation();
                    //add to currentTransformation
                }
            }
        } else {
            //interpolate first step
            double fromTimeStep = timeStepsForCorrection[i];
            double toTimeStep = posDiff[j].getTimeStamp();
            //interpolate posDiff[j] from to
            Eigen::Matrix4d firstTransformation = posDiff[j].getTransformation();
            double interpolationFactor =
                    (toTimeStep - fromTimeStep) / (posDiff[j].getTimeStamp() - posDiff[j - 1].getTimeStamp());
            firstTransformation.block<3, 1>(0, 3) = interpolationFactor * firstTransformation.block<3, 1>(0, 3);
            Eigen::Matrix3d rotationMatrix = firstTransformation.block<3, 3>(0, 0);
            Eigen::Vector3d eulerAngles = rotationMatrix.eulerAngles(0, 1, 2);
            eulerAngles = interpolationFactor * eulerAngles;

            Eigen::Matrix3d tmp;
            tmp = Eigen::AngleAxisd(eulerAngles[0], Eigen::Vector3d::UnitX()) *
                  Eigen::AngleAxisd(eulerAngles[1], Eigen::Vector3d::UnitY()) *
                  Eigen::AngleAxisd(eulerAngles[2], Eigen::Vector3d::UnitZ());
            firstTransformation.block<3, 3>(0, 0) = tmp;
            currentTransformation *= firstTransformation;
            for (int k = j + 1; k < posDiff.size(); k++) {
                currentTransformation *= posDiff[k].getTransformation();
            }
        }
        listOfTransormations.push_back(currentTransformation);
    }


    //correct points by transformation
    pcl::PointCloud<pcl::PointXYZ> cloud;
    for (int i = 0; i < vectorOfPointsAngle.size(); i++) {
        pcl::PointXYZ point;
        point = vectorOfPointsAngle[i].pointXyz;
        Eigen::Vector4d currentPoint(point.x, point.y, point.z, 1);
        currentPoint = listOfTransormations[i].inverse() * currentPoint;
        point.x = currentPoint.x();
        point.y = currentPoint.y();
        point.z = currentPoint.z();
        //fill point
        cloud.push_back(point);
    }
    *cloudScan = cloud;
}

std::vector<double> slamToolsRos::linspace(double start_in, double end_in, int num_in) {

    std::vector<double> linspaced;

    double start = start_in;
    double end = end_in;
    auto num = (double) num_in;

    if (num == 0) { return linspaced; }
    if (num == 1) {
        linspaced.push_back(start);
        return linspaced;
    }

    double delta = (end - start) / (num - 1);//stepSize

    for (int i = 0; i < num - 1; ++i) {
        linspaced.push_back(start + delta * i);
    }
    linspaced.push_back(end); // I want to ensure that start and end
    // are exactly the same as the input
    return linspaced;
}

void slamToolsRos::calculatePositionOverTime(std::vector<measurement> &angularVelocityList,
                                             std::vector<measurement> &bodyVelocityList,
                                             std::vector<edge> &posOverTimeEdge,
                                             double lastTimeStamp,
                                             double currentTimeStamp) {//last then current
    posOverTimeEdge.clear();
    std::vector<double> timeSteps = slamToolsRos::linspace(lastTimeStamp, currentTimeStamp, 10);
    std::vector<double> angularX;
    std::vector<double> angularY;
    std::vector<double> angularZ;
    for (int i = 1;
         i < timeSteps.size(); i++) {//calculate that between lastTimeKeyFrame and timeCurrentGroundTruth
        std::vector<measurement> measurementsOfInterest;
        for (int j = 0; j < angularVelocityList.size(); j++) {
            if (timeSteps[i - 1] <= angularVelocityList[j].timeStamp &&
                timeSteps[i] > angularVelocityList[j].timeStamp) {
                measurementsOfInterest.push_back(angularVelocityList[j]);
            }
        }

        double integratorX = 0;
        double integratorY = 0;
        double integratorZ = 0;
        for (int j = 0; j < measurementsOfInterest.size(); j++) {
            if (j == angularVelocityList.size() - 1) {
                integratorX += angularVelocityList[j].x * (timeSteps[i] - angularVelocityList[j - 1].timeStamp);
                integratorY += angularVelocityList[j].y * (timeSteps[i] - angularVelocityList[j - 1].timeStamp);
                integratorZ += angularVelocityList[j].z * (timeSteps[i] - angularVelocityList[j - 1].timeStamp);
            } else {
                if (j == 0) {
                    integratorX += angularVelocityList[j].x * (angularVelocityList[j].timeStamp - timeSteps[i - 1]);
                    integratorY += angularVelocityList[j].y * (angularVelocityList[j].timeStamp - timeSteps[i - 1]);
                    integratorZ += angularVelocityList[j].z * (angularVelocityList[j].timeStamp - timeSteps[i - 1]);
                } else {
                    integratorX += (angularVelocityList[j].x + angularVelocityList[j - 1].x) / 2 *
                                   (angularVelocityList[j].timeStamp - angularVelocityList[j - 1].timeStamp);
                    integratorY += (angularVelocityList[j].y + angularVelocityList[j - 1].y) / 2 *
                                   (angularVelocityList[j].timeStamp - angularVelocityList[j - 1].timeStamp);
                    integratorZ += (angularVelocityList[j].z + angularVelocityList[j - 1].z) / 2 *
                                   (angularVelocityList[j].timeStamp - angularVelocityList[j - 1].timeStamp);
                }
            }
        }
        angularX.push_back(integratorX);
        angularY.push_back(integratorY);
        angularZ.push_back(integratorZ);
    }


    std::vector<double> linearX;
    std::vector<double> linearY;
    std::vector<double> linearZ;
    for (int i = 1;
         i < timeSteps.size(); i++) {//calculate that between lastTimeKeyFrame and timeCurrentGroundTruth
        std::vector<measurement> measurementsOfInterest;
        for (int j = 0; j < bodyVelocityList.size(); j++) {
            if (timeSteps[i - 1] <= bodyVelocityList[j].timeStamp &&
                timeSteps[i] > bodyVelocityList[j].timeStamp) {
                measurementsOfInterest.push_back(bodyVelocityList[j]);
            }
        }

        double integratorX = 0;
        double integratorY = 0;
        double integratorZ = 0;
        for (int j = 0; j < measurementsOfInterest.size(); j++) {
            if (j == bodyVelocityList.size() - 1) {
                integratorX += bodyVelocityList[j].x * (timeSteps[i] - bodyVelocityList[j - 1].timeStamp);
                integratorY += bodyVelocityList[j].y * (timeSteps[i] - bodyVelocityList[j - 1].timeStamp);
                integratorZ += bodyVelocityList[j].z * (timeSteps[i] - bodyVelocityList[j - 1].timeStamp);
            } else {
                if (j == 0) {
                    integratorX += bodyVelocityList[j].x * (bodyVelocityList[j].timeStamp - timeSteps[i - 1]);
                    integratorY += bodyVelocityList[j].y * (bodyVelocityList[j].timeStamp - timeSteps[i - 1]);
                    integratorZ += bodyVelocityList[j].z * (bodyVelocityList[j].timeStamp - timeSteps[i - 1]);
                } else {
                    integratorX += (bodyVelocityList[j].x + bodyVelocityList[j - 1].x) / 2 *
                                   (bodyVelocityList[j].timeStamp - bodyVelocityList[j - 1].timeStamp);
                    integratorY += (bodyVelocityList[j].y + bodyVelocityList[j - 1].y) / 2 *
                                   (bodyVelocityList[j].timeStamp - bodyVelocityList[j - 1].timeStamp);
                    integratorZ += (bodyVelocityList[j].z + bodyVelocityList[j - 1].z) / 2 *
                                   (bodyVelocityList[j].timeStamp - bodyVelocityList[j - 1].timeStamp);
                }
            }
        }
        linearX.push_back(integratorX);
        linearY.push_back(integratorY);
        linearZ.push_back(integratorZ);
    }



    //std::vector<vertex> &posOverTimeVertex,
    //std::vector<edge> &posOverTimeEdge,
    double scalingOfWrongData = 0.3;//@TODO just a test scaling of POS Diff
    for (int i = 0; i < timeSteps.size() - 1; i++) {
        Eigen::Vector3d posDiff(linearX[i], linearY[i], 0);//linear Z missing
        Eigen::Quaterniond rotDiff = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX())//should be added somewhen(6DOF)
                                     * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())//should be added somewhen(6DOF)
                                     * Eigen::AngleAxisd(-scalingOfWrongData*angularZ[i], Eigen::Vector3d::UnitZ());//@TODO test for right mapping
        Eigen::Vector3d covariancePos(0, 0, 0);
        edge currentEdge(0, 0, -scalingOfWrongData*posDiff, rotDiff, covariancePos, 0, 3,graphSlamSaveStructure::INTEGRATED_POS_USAGE);
        currentEdge.setTimeStamp(timeSteps[i + 1]);
        posOverTimeEdge.push_back(currentEdge);
    }
}

bool slamToolsRos::detectLoopClosure(graphSlamSaveStructure &graphSaved, scanRegistrationClass &registrationClass,
                                     double sigmaScaling, double cutoffFitnessOnDetect) {
    Eigen::Vector3d estimatedPosLastPoint = graphSaved.getVertexList().back().getPositionVertex();
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
    const int ignoreLastNLoopClosures = 1;
    std::vector<int> has2beChecked;
    if (dist.size() > ignoreLastNLoopClosures) {
        for (int i = 0; i < dist.size() - ignoreLastNLoopClosures; i++) {
            if (dist(i, 0) < 1 && graphSaved.getVertexList()[i].getTypeOfVertex() == graphSlamSaveStructure::POINT_CLOUD_USAGE) {
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
            Eigen::Matrix4d currentTransformation,guess;
            guess = graphSaved.getVertexList().back().getTransformation().inverse() *
                    graphSaved.getVertexList()[has2beCheckedElemenet].getTransformation();
            currentTransformation = registrationClass.generalizedIcpRegistrationSimple(
                    graphSaved.getVertexList()[has2beCheckedElemenet].getPointCloud(),
                    graphSaved.getVertexList().back().getPointCloud(),
                    fitnessScore,guess);
            fitnessScore = sqrt(fitnessScore);
            if (fitnessScore < cutoffFitnessOnDetect) {//@TODO was 0.1
                std::cout << "Found Loop Closure with fitnessScore: " << fitnessScore << std::endl;
                if (fitnessScore < 0.01) {
                    std::cout << "FitnessScore Very Low: " << fitnessScore << std::endl;
                    fitnessScore = 0.01;
                }
                Eigen::Vector3d currentPosDiff;
                Eigen::Quaterniond currentRotDiff(currentTransformation.inverse().block<3, 3>(0, 0));
                currentPosDiff.x() = currentTransformation.inverse()(0, 3);
                currentPosDiff.y() = currentTransformation.inverse()(1, 3);
                currentPosDiff.z() = 0;
                Eigen::Vector3d positionCovariance(fitnessScore, fitnessScore, 0);
                graphSaved.addEdge(has2beCheckedElemenet, (int) graphSaved.getVertexList().size() - 1, currentPosDiff,
                                   currentRotDiff, positionCovariance, (double) (0.1 * fitnessScore),graphSlamSaveStructure::INTEGRATED_POS_USAGE);
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