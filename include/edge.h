//
// Created by tim on 23.02.21.
//

#ifndef SIMULATION_BLUEROV_EDGE_H
#define SIMULATION_BLUEROV_EDGE_H

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

class edge {
public:
    edge(const int fromVertex, const int toVertex, const Eigen::Vector3f &positionDifference,
         const Eigen::Quaternionf &rotationDifference, const Eigen::Vector3f covariancePosition, const float covarianceQuaternion,
         int degreeOfFreedom) {
        if (degreeOfFreedom == 3) {
            edge::fromVertex = fromVertex;
            edge::toVertex = toVertex;
            edge::positionDifference = positionDifference;
            edge::rotationDifference = rotationDifference;

            edge::covariancePosition = covariancePosition;
            edge::covarianceQuaternion = covarianceQuaternion;

        } else {
            std::cout << "not yet implemented DOF 6" << std::endl;
            std::exit(-1);
        }
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZ>);
        edge::pointCloud = tmp;
    }

    edge(const int fromVertex, const int toVertex, const Eigen::Vector3f &positionDifference,
         const Eigen::Quaternionf &rotationDifference, const Eigen::Vector3f covariancePosition, const float covarianceQuaternion,
         int degreeOfFreedom,pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloud) {
        if (degreeOfFreedom == 3) {
            edge::fromVertex = fromVertex;
            edge::toVertex = toVertex;
            edge::positionDifference = positionDifference;
            edge::rotationDifference = rotationDifference;

            edge::covariancePosition = covariancePosition;
            edge::covarianceQuaternion = covarianceQuaternion;

        } else {
            std::cout << "not yet implemented DOF 6" << std::endl;
            std::exit(-1);
        }
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZ>);
        edge::pointCloud = tmp;
        setPointCloud(pointCloud);
    }

    void setEdge(edge &edgeToCopy);

    Eigen::Vector3f getCovariancePosition() const;

    void setCovariancePosition(Eigen::Vector3f covariancePosition);

    float getCovarianceQuaternion() const;

    void setCovarianceQuaternion(float covarianceQuaternion);

    const Eigen::Vector3f &getPositionDifference() const;

    void setPositionDifference(const Eigen::Vector3f &positionDifference);

    const Eigen::Quaternionf &getRotationDifference() const;

    void setRotationDifference(const Eigen::Quaternionf &rotationDifference);

    int getFromVertex() const;

    void setFromVertex(int fromVertex);

    int getToVertex() const;

    void setToVertex(int toVertex);

    const pcl::PointCloud<pcl::PointXYZ>::Ptr &getPointCloud() const;

    void setPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloud);


private:
    int fromVertex;
    int toVertex;
    Eigen::Vector3f covariancePosition;//estimated covarianze for this measurement in x y z
    float covarianceQuaternion;//estimated covarianze for this measurement in q = w x y z (rotation)
    bool hasPointCloud;
public:
    bool getHasPointCloud() const;

    void setHasPointCloud(bool hasPointCloud);

private:
    Eigen::Vector3f positionDifference;
    Eigen::Quaternionf rotationDifference;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud;//measurement
    int typeOfEdge;// 0=pointCloud    %%%%%%%%%   1 = integratedPosDiff
    float timeStamp;
public:
    bool isHasPointCloud() const;

    int getTypeOfEdge() const;

    void setTypeOfEdge(int typeOfEdge);

    float getTimeStamp() const;

    void setTimeStamp(float timeStamp);

    Eigen::Matrix4f getTransformation();
};


#endif //SIMULATION_BLUEROV_EDGE_H
