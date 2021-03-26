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
