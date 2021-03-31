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

Eigen::Vector3f vertex::getCovariancePosition() const {
    return covariancePosition;
}

void vertex::setCovariancePosition(Eigen::Vector3f covariancePosition) {
    vertex::covariancePosition = covariancePosition;
}

float vertex::getCovarianceQuaternion() const {
    return covarianceQuaternion;
}

void vertex::setCovarianceQuaternion(float covarianceQuaternion) {
    vertex::covarianceQuaternion = covarianceQuaternion;
}

Eigen::Matrix4f vertex::getTransformation(){
    Eigen::Matrix4f transformation;

    transformation << 1, 0, 0, this->positionVertex.x(),
            0, 1, 0, this->positionVertex.y(),
            0, 0, 1, this->positionVertex.z(),
            0, 0, 0, 1;//transformation missing currently
    Eigen::Matrix3f m(this->rotationVertex.toRotationMatrix());
    transformation.block<3, 3>(0, 0) = m;
    return transformation;
}

int vertex::getTypeOfVertex() const {
    return typeOfVertex;
}

void vertex::setTypeOfVertex(int typeOfVertex) {// 0=pointCloud    %%%%%%%%%   1 = integratedPosDiff
    vertex::typeOfVertex = typeOfVertex;
}

float vertex::getTimeStamp() const {
    return timeStamp;
}

void vertex::setTimeStamp(float timeStamp) {
    vertex::timeStamp = timeStamp;
}
