//
// Created by tim on 23.02.21.
//
#include "edge.h"
#include "vertex.h"

#ifndef SIMULATION_BLUEROV_GRAPHSLAMSAVESTRUCTURE_H
#define SIMULATION_BLUEROV_GRAPHSLAMSAVESTRUCTURE_H


class graphSlamSaveStructure {
public:
    graphSlamSaveStructure(int degreeOfFreedom) {
        if (degreeOfFreedom == 3) {
            graphSlamSaveStructure::degreeOfFreedom = degreeOfFreedom;
            graphSlamSaveStructure::numberOfEdges = 0;
            graphSlamSaveStructure::numberOfVertex = 0;

        } else {
            std::cout << "not yet implemented DOF 6" << std::endl;
            std::exit(-1);
        }
    }

    void addEdge(const int fromVertex, const int toVertex, const Eigen::Vector3f &positionDifference,
                 const Eigen::Quaternionf &rotationDifference, const float covariancePosition,
                 const float covarianceQuaternion);

    void addEdge(const int fromVertex, const int toVertex, const Eigen::Vector3f &positionDifference,
                 const Eigen::Quaternionf &rotationDifference, const float covariancePosition,
                 const float covarianceQuaternion, pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloud);

    void addVertex(int vertexNumber, const Eigen::Vector3f &positionVertex, Eigen::Quaternionf &rotationVertex);

    Eigen::MatrixXf getInformationMatrix();

    Eigen::MatrixXf getJacobianMatrix();

    Eigen::MatrixXf getErrorMatrix();

    void printCurrentState();

    void addToState(std::vector<Eigen::Vector3f> &positionDifferenceVector,std::vector<Eigen::Quaternionf> &rotationDifferenceVector);
    void addToState(Eigen::MatrixXf &vectorToAdd);

private:
    int degreeOfFreedom;//3 for [x y alpha] or 6 for [x y z alpha beta gamma]
    int numberOfEdges;
    int numberOfVertex;
    std::vector<edge> edgeList;
    std::vector<vertex> vertexList;
};


#endif //SIMULATION_BLUEROV_GRAPHSLAMSAVESTRUCTURE_H
