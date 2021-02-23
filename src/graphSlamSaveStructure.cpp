//
// Created by tim on 23.02.21.
//

#include "graphSlamSaveStructure.h"

void graphSlamSaveStructure::addEdge(int fromVertex, int toVertex, std::vector<float> &informationMeasurement,
                                     std::vector<float> &measurementDifference) {
    edge edgeToAdd(fromVertex, toVertex, informationMeasurement, measurementDifference,
                   graphSlamSaveStructure::stateDimension);
    graphSlamSaveStructure::numberOfEdges += 1;
    graphSlamSaveStructure::edgeList.push_back(edgeToAdd);

}

void graphSlamSaveStructure::addEdge(int fromVertex, int toVertex, std::vector<float> &informationMeasurement,
                                     std::vector<float> &measurementDifference, pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloud) {
    edge edgeToAdd(fromVertex, toVertex, informationMeasurement, measurementDifference,
                   graphSlamSaveStructure::stateDimension,pointCloud);
    graphSlamSaveStructure::numberOfEdges += 1;
    graphSlamSaveStructure::edgeList.push_back(edgeToAdd);

}

void graphSlamSaveStructure::addVertex(int vertexNumber, std::vector<float> &estimatedState) {

    vertex vertexToAdd(vertexNumber, estimatedState, graphSlamSaveStructure::stateDimension);
    graphSlamSaveStructure::numberOfVertex += 1;
    graphSlamSaveStructure::vertexList.push_back(vertexToAdd);

}

Eigen::MatrixXf graphSlamSaveStructure::getInformationMatrix() {
    Eigen::MatrixXf informationMatrix = Eigen::MatrixXf::Zero(
            graphSlamSaveStructure::numberOfEdges * graphSlamSaveStructure::stateDimension + graphSlamSaveStructure::stateDimension,
            graphSlamSaveStructure::numberOfEdges * graphSlamSaveStructure::stateDimension + graphSlamSaveStructure::stateDimension);

    for (int j = 0; j < graphSlamSaveStructure::stateDimension; j++) {
        informationMatrix(j, j) = 1;//first indece
    }
    for (int i = 1; i <= graphSlamSaveStructure::numberOfEdges; i++) {
        for (int j = 0; j < graphSlamSaveStructure::stateDimension; j++) {
            informationMatrix(i * graphSlamSaveStructure::stateDimension + j, i * graphSlamSaveStructure::stateDimension + j) = graphSlamSaveStructure::edgeList[i -
                                                                                                                1].getInformationMeasurement()[j];
        }
    }
    return informationMatrix;
}

Eigen::MatrixXf graphSlamSaveStructure::getConectivityMatrix() {
    Eigen::MatrixXf connectivityMatrix = Eigen::MatrixXf::Zero(
            graphSlamSaveStructure::numberOfEdges * graphSlamSaveStructure::stateDimension + graphSlamSaveStructure::stateDimension,
            graphSlamSaveStructure::numberOfVertex * graphSlamSaveStructure::stateDimension );
    for (int i = 0; i < graphSlamSaveStructure::stateDimension ; i++) {
        for (int j = 0; j < graphSlamSaveStructure::stateDimension ; j++) {
            connectivityMatrix(i, i) = -1;
        }
    }

//    connectivityMatrix <<   -1, 0, 0,
//                            1, -1, 0,
//                            0, 1, -1;
    for (int i = 0; i < graphSlamSaveStructure::numberOfEdges; i++) {
        for (int j = 0; j < graphSlamSaveStructure::stateDimension ; j++) {
            connectivityMatrix(i * graphSlamSaveStructure::stateDimension  + graphSlamSaveStructure::stateDimension  + j,
                               graphSlamSaveStructure::stateDimension  * graphSlamSaveStructure::edgeList[i].getFromVertex() + j) = 1;
            connectivityMatrix(i * graphSlamSaveStructure::stateDimension  + graphSlamSaveStructure::stateDimension  + j,
                               graphSlamSaveStructure::stateDimension  * graphSlamSaveStructure::edgeList[i].getToVertex() + j) = -1;
        }
    }
//    std::cout << "connectivityMatrix" << std::endl;
//    std::cout << connectivityMatrix << std::endl;
    return connectivityMatrix;
}

Eigen::MatrixXf graphSlamSaveStructure::getJacobianMatrix() {
    Eigen::MatrixXf jacobianMatrix = Eigen::MatrixXf::Zero(graphSlamSaveStructure::numberOfVertex, graphSlamSaveStructure::numberOfVertex);
//    jacobianMatrix << -1, 0, 0,
//                        1, -1, 0,
//                        0, 1, -1;
    jacobianMatrix = getConectivityMatrix();
    return jacobianMatrix;
}

Eigen::MatrixXf graphSlamSaveStructure::getErrorMatrix() {

    Eigen::MatrixXf stateTMP;
    stateTMP.resize(graphSlamSaveStructure::numberOfVertex * graphSlamSaveStructure::stateDimension , 1);
    for (int i = 0; i < graphSlamSaveStructure::numberOfVertex; i++) {
        for (int j = 0; j < graphSlamSaveStructure::stateDimension ; j++) {
            stateTMP(i * graphSlamSaveStructure::stateDimension  + j, 0) = graphSlamSaveStructure::vertexList[i].getStateVertex()[j];
        }
    }

    Eigen::MatrixXf error = Eigen::MatrixXf::Zero(
            graphSlamSaveStructure::numberOfEdges * graphSlamSaveStructure::stateDimension  + graphSlamSaveStructure::stateDimension , 1);//add an additional size for x0
    for (int i = 0; i < graphSlamSaveStructure::numberOfEdges; i++) {
        for (int j = 0; j < graphSlamSaveStructure::stateDimension ; j++) {
            error(i * graphSlamSaveStructure::stateDimension  + j + graphSlamSaveStructure::stateDimension ,
                  0) = graphSlamSaveStructure::edgeList[i].getMeasurementDifference()[j];
        }
    }
    error = error + getConectivityMatrix() * stateTMP;
    return error;
}

void graphSlamSaveStructure::printCurrentState() {
    std::cout << "current State:" << std::endl;
    for (int i = 0; i < graphSlamSaveStructure::numberOfVertex; i++) {
        for (int j = 0; j < graphSlamSaveStructure::stateDimension; j++) {
            std::cout << graphSlamSaveStructure::vertexList[i].getStateVertex()[j] << std::endl;
        }
    }
}

void graphSlamSaveStructure::addToState(Eigen::MatrixXf &vectorToAdd) {


    for (int i = 0; i < graphSlamSaveStructure::numberOfVertex; i++) {
        std::vector<float> currentState = graphSlamSaveStructure::vertexList[i].getStateVertex();
        for (int j = 0; j < graphSlamSaveStructure::stateDimension; j++) {
            currentState[j]+= (float) vectorToAdd(i * graphSlamSaveStructure::stateDimension + j, 0);
        }
        graphSlamSaveStructure::vertexList[i].setStateVertex(currentState);
    }
}
