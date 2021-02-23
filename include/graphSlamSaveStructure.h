//
// Created by tim on 23.02.21.
//
#include "edge.h"
#include "vertex.h"
#ifndef SIMULATION_BLUEROV_GRAPHSLAMSAVESTRUCTURE_H
#define SIMULATION_BLUEROV_GRAPHSLAMSAVESTRUCTURE_H


class graphSlamSaveStructure {
public:
    graphSlamSaveStructure(int stateDimension) {
        graphSlamSaveStructure::stateDimension = stateDimension;
        graphSlamSaveStructure::numberOfEdges = 0;
        graphSlamSaveStructure::numberOfVertex = 0;
    }
    void addEdge(int fromVertex, int toVertex, std::vector<float> &informationMeasurement, std::vector<float> &measurementDifference);
    void addEdge(int fromVertex, int toVertex, std::vector<float> &informationMeasurement, std::vector<float> &measurementDifference,pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloud);
    void addVertex(int vertexNumber, std::vector<float> &estimatedState);
    Eigen::MatrixXf getInformationMatrix();
    Eigen::MatrixXf getConectivityMatrix();
    Eigen::MatrixXf getJacobianMatrix();
    Eigen::MatrixXf getErrorMatrix();
    void printCurrentState();
    void addToState(Eigen::MatrixXf &vectorToAdd);

private:
    int stateDimension;
    int numberOfEdges;
    int numberOfVertex;
    std::vector<edge> edgeList;
    std::vector<vertex> vertexList;
};


#endif //SIMULATION_BLUEROV_GRAPHSLAMSAVESTRUCTURE_H
