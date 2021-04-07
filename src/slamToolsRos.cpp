//
// Created by tim on 26.03.21.
//

#include "slamToolsRos.h"

void slamToolsRos::visualizeCurrentGraph(graphSlamSaveStructure &graphSaved, ros::Publisher &publisherPath,
                                         ros::Publisher &publisherCloud, ros::Publisher &publisherMarkerArray,
                                         float sigmaScaling, ros::Publisher &publisherPathGT,
                                         std::vector<std::vector<measurement>> &groundTruthSorted) {

    nav_msgs::Path posOverTime;
    posOverTime.header.frame_id = "map_ned";
    Eigen::Matrix4f currentTransformation, completeTransformation;
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
        Eigen::Matrix3f m(vertexElement.getRotationVertex().toRotationMatrix());
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
                                          float timestepBeginning) {
    //this is assuming we are linear in time
    //resulting scan should be at end position means: point 0 is transformed by diff of pos 0 to n

    struct pclAngle {
        pcl::PointXYZ pointXyz;
        float angle{};
    };

    //calculate angle of every point
    std::vector<pclAngle> vectorOfPointsAngle;
    for (int i = 0; i < cloudScan->size(); i++) {
        float currentAngle = fmod(2 * (float) M_PI + atan2(cloudScan->points[i].y, cloudScan->points[i].x), 2 * M_PI);
        pclAngle tmpPCL;
        tmpPCL.pointXyz = cloudScan->points[i];
        tmpPCL.angle = currentAngle;
        vectorOfPointsAngle.push_back(tmpPCL);
    }
    //sort from 0 to 2pi
    std::sort(vectorOfPointsAngle.begin(), vectorOfPointsAngle.end(),
              [](const auto &i, const auto &j) { return i.angle < j.angle; });


    //calculate number of points transformations defined by edge list
    std::vector<Eigen::Matrix4f> listOfTransormations;
    float startTime = timestepBeginning;
    float endTime = posDiff.back().getTimeStamp();
    std::vector<float> timeStepsForCorrection = slamToolsRos::linspace(startTime, endTime, vectorOfPointsAngle.size());
    for (int i = 0; i < timeStepsForCorrection.size(); i++) {
        Eigen::Matrix4f currentTransformation;
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

                float fromTimeStep = timeStepsForCorrection[i];
                float toTimeStep = posDiff[j].getTimeStamp();
                //interpolate posDiff[j] from to
                Eigen::Matrix4f firstTransformation = posDiff[j].getTransformation();
                float interpolationFactor =
                        (toTimeStep - fromTimeStep) / (posDiff[j].getTimeStamp() - startTime);
                firstTransformation.block<3, 1>(0, 3) = interpolationFactor * firstTransformation.block<3, 1>(0, 3);
                Eigen::Matrix3f rotationMatrix = firstTransformation.block<3, 3>(0, 0);
                Eigen::Vector3f eulerAngles = rotationMatrix.eulerAngles(0, 1, 2);
                eulerAngles = interpolationFactor * eulerAngles;

                Eigen::Matrix3f tmp;
                tmp = Eigen::AngleAxisf(eulerAngles[0], Eigen::Vector3f::UnitX()) *
                      Eigen::AngleAxisf(eulerAngles[1], Eigen::Vector3f::UnitY()) *
                      Eigen::AngleAxisf(eulerAngles[2], Eigen::Vector3f::UnitZ());
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
            float fromTimeStep = timeStepsForCorrection[i];
            float toTimeStep = posDiff[j].getTimeStamp();
            //interpolate posDiff[j] from to
            Eigen::Matrix4f firstTransformation = posDiff[j].getTransformation();
            float interpolationFactor =
                    (toTimeStep - fromTimeStep) / (posDiff[j].getTimeStamp() - posDiff[j - 1].getTimeStamp());
            firstTransformation.block<3, 1>(0, 3) = interpolationFactor * firstTransformation.block<3, 1>(0, 3);
            Eigen::Matrix3f rotationMatrix = firstTransformation.block<3, 3>(0, 0);
            Eigen::Vector3f eulerAngles = rotationMatrix.eulerAngles(0, 1, 2);
            eulerAngles = interpolationFactor * eulerAngles;

            Eigen::Matrix3f tmp;
            tmp = Eigen::AngleAxisf(eulerAngles[0], Eigen::Vector3f::UnitX()) *
                  Eigen::AngleAxisf(eulerAngles[1], Eigen::Vector3f::UnitY()) *
                  Eigen::AngleAxisf(eulerAngles[2], Eigen::Vector3f::UnitZ());
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
        Eigen::Vector4f currentPoint(point.x, point.y, point.z, 1);
        currentPoint = listOfTransormations[i].inverse() * currentPoint;
        point.x = currentPoint.x();
        point.y = currentPoint.y();
        point.z = currentPoint.z();
        //fill point
        cloud.push_back(point);
    }
    *cloudScan = cloud;
}

std::vector<float> slamToolsRos::linspace(float start_in, float end_in, int num_in) {

    std::vector<float> linspaced;

    float start = start_in;
    float end = end_in;
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
                                             float lastTimeStamp,
                                             float currentTimeStamp) {//last then current
    posOverTimeEdge.clear();
    std::vector<float> timeSteps = slamToolsRos::linspace(lastTimeStamp, currentTimeStamp, 10);
    std::vector<float> angularX;
    std::vector<float> angularY;
    std::vector<float> angularZ;
    for (int i = 1;
         i < timeSteps.size(); i++) {//calculate that between lastTimeKeyFrame and timeCurrentGroundTruth
        std::vector<measurement> measurementsOfInterest;
        for (int j = 0; j < angularVelocityList.size(); j++) {
            if (timeSteps[i - 1] <= angularVelocityList[j].timeStamp &&
                timeSteps[i] > angularVelocityList[j].timeStamp) {
                measurementsOfInterest.push_back(angularVelocityList[j]);
            }
        }

        float integratorX = 0;
        float integratorY = 0;
        float integratorZ = 0;
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


    std::vector<float> linearX;
    std::vector<float> linearY;
    std::vector<float> linearZ;
    for (int i = 1;
         i < timeSteps.size(); i++) {//calculate that between lastTimeKeyFrame and timeCurrentGroundTruth
        std::vector<measurement> measurementsOfInterest;
        for (int j = 0; j < bodyVelocityList.size(); j++) {
            if (timeSteps[i - 1] <= bodyVelocityList[j].timeStamp &&
                timeSteps[i] > bodyVelocityList[j].timeStamp) {
                measurementsOfInterest.push_back(bodyVelocityList[j]);
            }
        }

        float integratorX = 0;
        float integratorY = 0;
        float integratorZ = 0;
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
    float scalingOfWrongData = 0.4;//@TODO just a test scaling of POS Diff
    for (int i = 0; i < timeSteps.size() - 1; i++) {
        Eigen::Vector3f posDiff(linearX[i], linearY[i], 0);//linear Z missing
        Eigen::Quaternionf rotDiff = Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX())//should be added somewhen(6DOF)
                                     * Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY())//should be added somewhen(6DOF)
                                     * Eigen::AngleAxisf(-scalingOfWrongData*angularZ[i], Eigen::Vector3f::UnitZ());//@TODO test for right mapping
        Eigen::Vector3f covariancePos(0, 0, 0);
        edge currentEdge(0, 0, -scalingOfWrongData*posDiff, rotDiff, covariancePos, 0, 3,graphSlamSaveStructure::INTEGRATED_POS_USAGE);
        currentEdge.setTimeStamp(timeSteps[i + 1]);
        posOverTimeEdge.push_back(currentEdge);
    }
}

bool slamToolsRos::detectLoopClosure(graphSlamSaveStructure &graphSaved, scanRegistrationClass &registrationClass,
                                     double sigmaScaling, double scalingAllg) {
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
            Eigen::Matrix4f currentTransformation;
            currentTransformation = registrationClass.generalizedIcpRegistrationSimple(
                    graphSaved.getVertexList()[has2beCheckedElemenet].getPointCloud(),
                    graphSaved.getVertexList().back().getPointCloud(),
                    fitnessScore);
            fitnessScore = sqrt(fitnessScore);
            if (fitnessScore < scalingAllg * 4 * 1) {//@TODO was 0.1
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
                                   currentRotDiff, positionCovariance, (float) (0.1 * fitnessScore),graphSlamSaveStructure::INTEGRATED_POS_USAGE);
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