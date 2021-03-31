//
// Created by tim on 23.02.21.
//

#ifndef SIMULATION_BLUEROV_VETREX_H
#define SIMULATION_BLUEROV_VETREX_H
//#include <vector>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

class vertex {

public:
    vertex(int vertexNumber, const Eigen::Vector3f &positionVertex, const Eigen::Quaternionf &rotationVertex,
           int degreeOfFreedom, const Eigen::Vector3f &covariancePosition, const float covarianceQuaternion) {
        vertex::vertexNumber = vertexNumber;
        vertex::positionVertex = positionVertex;
        vertex::rotationVertex = rotationVertex;
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
        vertex::pointCloud = tmp;
        vertex::covariancePosition = covariancePosition;
        vertex::covarianceQuaternion = covarianceQuaternion;
    }

    vertex(int vertexNumber, const Eigen::Vector3f &positionVertex, const Eigen::Quaternionf &rotationVertex,
           int degreeOfFreedom, pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloud,
           const Eigen::Vector3f &covariancePosition,
           const float covarianceQuaternion) {
        vertex::vertexNumber = vertexNumber;
        vertex::positionVertex = positionVertex;
        vertex::rotationVertex = rotationVertex;
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
        vertex::pointCloud = tmp;
        vertex::covariancePosition = covariancePosition;
        vertex::covarianceQuaternion = covarianceQuaternion;
        setPointCloud(pointCloud);
    }

    int getVertexNumber() const;

    void setVertexNumber(int vertexNumber);

    const Eigen::Vector3f &getPositionVertex() const;

    void setPositionVertex(const Eigen::Vector3f &positionVertex);

    const Eigen::Quaternionf &getRotationVertex() const;

    void setRotationVertex(const Eigen::Quaternionf &rotationVertex);

    const pcl::PointCloud<pcl::PointXYZ>::Ptr &getPointCloud() const;

    void setPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloud);

    Eigen::Vector3f getCovariancePosition() const;

    void setCovariancePosition(Eigen::Vector3f covariancePosition);

    float getCovarianceQuaternion() const;

    void setCovarianceQuaternion(float covarianceQuaternion);

    Eigen::Matrix4f getTransformation();

    int getTypeOfVertex() const;

    void setTypeOfVertex(int typeOfVertex);

    float getTimeStamp() const;

    void setTimeStamp(float timeStamp);

private:
    int vertexNumber;
    Eigen::Vector3f positionVertex;// position w.r.t. Initiial Starting Position
    Eigen::Quaternionf rotationVertex;// rotation w.r.t. Initial Starting Rotation
    Eigen::Vector3f covariancePosition;
    float covarianceQuaternion;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud;//measurement by edge from this vertex to previous vertex
    int typeOfVertex;// 0=pointCloud    %%%%%%%%%   1 = integratedPosDiff

    float timeStamp;
};


#endif //SIMULATION_BLUEROV_VETREX_H
