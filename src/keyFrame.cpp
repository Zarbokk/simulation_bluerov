//
// Created by tim on 12.02.21.
//

#include "../include/keyFrame.h"

const pcl::PointCloud<pcl::PointXYZ>::Ptr &keyFrame::getPointCloud() const {
    return pointCloud;
}

void keyFrame::setPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloud) {
    keyFrame::pointCloud = pointCloud;
}

const Eigen::Vector3f &keyFrame::getEstimatedPos() const {
    return estimatedPos;
}

void keyFrame::setEstimatedPos(const Eigen::Vector3f &estimatedPos) {
    keyFrame::estimatedPos = estimatedPos;
}

const Eigen::Quaternionf &keyFrame::getEstimatedRotation() const {
    return estimatedRotation;
}

void keyFrame::setEstimatedRotation(const Eigen::Quaternionf &estimatedRotation) {
    keyFrame::estimatedRotation = estimatedRotation;
}

const Eigen::Vector3f &keyFrame::getCovarianzPos() const {
    return covarianzPos;
}

void keyFrame::setCovarianzPos(const Eigen::Vector3f &covarianzPos) {
    keyFrame::covarianzPos = covarianzPos;
}

const Eigen::Vector3f &keyFrame::getCovarianzRpy() const {
    return covarianzRPY;
}

void keyFrame::setCovarianzRpy(const Eigen::Vector3f &covarianzRpy) {
    covarianzRPY = covarianzRpy;
}

const ros::Time &keyFrame::getTimeKeyframe() const {
    return timeKeyframe;
}

void keyFrame::setTimeKeyframe(const ros::Time &timeKeyframe) {
    keyFrame::timeKeyframe = timeKeyframe;
}
