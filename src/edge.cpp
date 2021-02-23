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

std::vector<float> edge::getInformationMeasurement() const {
    return edge::informationMeasurement;
}

void edge::setInformationMeasurement(std::vector<float> &informationMeasurement) {
    edge::informationMeasurement = informationMeasurement;
}

std::vector<float> edge::getMeasurementDifference() const {
    return edge::measurementDifference;
}

void edge::setMeasurementDifference(std::vector<float> &measurementDifference) {
    edge::measurementDifference = measurementDifference;
}

const pcl::PointCloud<pcl::PointXYZ>::Ptr &edge::getPointCloud() const {
    return edge::pointCloud;
}

void edge::setPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloud) {
    edge::pointCloud = pointCloud;
}
