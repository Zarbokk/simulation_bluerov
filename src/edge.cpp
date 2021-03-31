//
// Created by tim on 23.02.21.
//

#include "edge.h"

int edge::getFromVertex() const {
    return edge::fromVertex;
}

void edge::setFromVertex(int fromVertex) {
    edge::fromVertex = fromVertex;
}

int edge::getToVertex() const {
    return edge::toVertex;
}

void edge::setToVertex(int toVertex) {
    edge::toVertex = toVertex;
}

const pcl::PointCloud<pcl::PointXYZ>::Ptr &edge::getPointCloud() const {
    return edge::pointCloud;
}

void edge::setPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloud) {
    *edge::pointCloud = *pointCloud;
}

Eigen::Vector3f edge::getCovariancePosition() const {
    return covariancePosition;
}

void edge::setEdge(edge &edgeToCopy){
    this->fromVertex = edgeToCopy.getFromVertex();
    this->toVertex = edgeToCopy.getToVertex();
    this->covariancePosition = edgeToCopy.getCovariancePosition();
    this->covarianceQuaternion = edgeToCopy.getCovarianceQuaternion();
    this->hasPointCloud = edgeToCopy.getHasPointCloud();
    this->positionDifference = edgeToCopy.getPositionDifference();
    this->rotationDifference = edgeToCopy.getRotationDifference();
    this->pointCloud = edgeToCopy.getPointCloud();

}

void edge::setCovariancePosition(Eigen::Vector3f covariancePosition) {
    edge::covariancePosition = covariancePosition;
}

float edge::getCovarianceQuaternion() const {
    return covarianceQuaternion;
}

void edge::setCovarianceQuaternion(float covarianceQuaternion) {
    edge::covarianceQuaternion = covarianceQuaternion;
}

const Eigen::Vector3f &edge::getPositionDifference() const {
    return positionDifference;
}

void edge::setPositionDifference(const Eigen::Vector3f &positionDifference) {
    edge::positionDifference = positionDifference;
}

const Eigen::Quaternionf &edge::getRotationDifference() const {
    return rotationDifference;
}

void edge::setRotationDifference(const Eigen::Quaternionf &rotationDifference) {
    edge::rotationDifference = rotationDifference;
}

bool edge::getHasPointCloud() const {
    return hasPointCloud;
}

void edge::setHasPointCloud(bool hasPointCloud) {
    edge::hasPointCloud = hasPointCloud;
}

bool edge::isHasPointCloud() const {
    return hasPointCloud;
}

int edge::getTypeOfEdge() const {
    return typeOfEdge;
}

void edge::setTypeOfEdge(int typeOfEdge) {// 0=pointCloud    %%%%%%%%%   1 = integratedPosDiff
    edge::typeOfEdge = typeOfEdge;
}

float edge::getTimeStamp() const {
    return timeStamp;
}

void edge::setTimeStamp(float timeStamp) {
    edge::timeStamp = timeStamp;
}

Eigen::Matrix4f edge::getTransformation(){
    Eigen::Matrix4f transformation;

    transformation << 1, 0, 0, this->positionDifference.x(),
            0, 1, 0, this->positionDifference.y(),
            0, 0, 1, this->positionDifference.z(),
            0, 0, 0, 1;//transformation missing currently
    Eigen::Matrix3f m(this->rotationDifference.toRotationMatrix());
    transformation.block<3, 3>(0, 0) = m;
    return transformation;
}