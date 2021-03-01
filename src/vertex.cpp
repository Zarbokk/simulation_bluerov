//
// Created by tim on 23.02.21.
//

#include "vertex.h"

int vertex::getVertexNumber() const {
    return vertex::vertexNumber;
}

void vertex::setVertexNumber(int vertexNumber) {
    vertex::vertexNumber = vertexNumber;
}

const Eigen::Vector3f &vertex::getPositionVertex() const {
    return positionVertex;
}

void vertex::setPositionVertex(const Eigen::Vector3f &positionVertex) {
    vertex::positionVertex = positionVertex;
}

const Eigen::Quaternionf &vertex::getRotationVertex() const {
    return rotationVertex;
}

void vertex::setRotationVertex(const Eigen::Quaternionf &rotationVertex) {
    vertex::rotationVertex = rotationVertex;
}

const pcl::PointCloud<pcl::PointXYZ>::Ptr &vertex::getPointCloud() const {
    return vertex::pointCloud;
}

void vertex::setPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloud) {
    *vertex::pointCloud = *pointCloud;
}

float vertex::getCovariancePosition() const {
    return covariancePosition;
}

void vertex::setCovariancePosition(float covariancePosition) {
    vertex::covariancePosition = covariancePosition;
}

float vertex::getCovarianceQuaternion() const {
    return covarianceQuaternion;
}

void vertex::setCovarianceQuaternion(float covarianceQuaternion) {
    vertex::covarianceQuaternion = covarianceQuaternion;
}
