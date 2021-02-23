//
// Created by tim on 22.02.21.
//
#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <iostream>

class edge {
public:
    edge(int fromNode, int toNode, float *informationMeasurement, float *measurementDifference, int dimension) {
        edge::fromNode = fromNode;
        edge::toNode = toNode;
        edge::dimension = dimension;
        edge::informationMeasurement = new float[edge::dimension];
        edge::measurementDifference = new float[edge::dimension];
        for (int i = 0; i < edge::dimension; i++) {
            edge::informationMeasurement[i] = informationMeasurement[i];
            edge::measurementDifference[i] = measurementDifference[i];
        }
    }

    int fromNode;
    int toNode;
    int dimension;
    float *informationMeasurement;
    float *measurementDifference;
private:

};

class node {
public:
    node(int nodeNumber, int dimension) {
        node::dimension = dimension;
        node::nodeNumber = nodeNumber;
        node::stateNode = new float[node::dimension];
        node::informationState = new float[node::dimension];
    }

    int nodeNumber;
    int dimension;
    float *stateNode;
    float *informationState;
private:

};

Eigen::MatrixXf getInformationMatrix(std::vector<node> &listOfNodes, std::vector<edge> &listOfEdges) {
    Eigen::MatrixXf informationMatrix = Eigen::MatrixXf::Zero(
            listOfEdges.size() * listOfEdges[0].dimension + listOfEdges[0].dimension,
            listOfEdges.size() * listOfEdges[0].dimension + listOfEdges[0].dimension);

    for (int j = 0; j < listOfEdges[0].dimension; j++) {
        informationMatrix(j, j) = 1;//first indece
    }
    for (int i = 1; i <= listOfEdges.size(); i++) {
        for (int j = 0; j < listOfEdges[0].dimension; j++) {
            informationMatrix(i * listOfEdges[0].dimension + j, i * listOfEdges[0].dimension + j) = listOfEdges[i -
                                                                                                                1].informationMeasurement[j];
        }
    }
    return informationMatrix;
}


Eigen::MatrixXf getConectivityMatrix(std::vector<node> &listOfNodes, std::vector<edge> &listOfEdges) {
    Eigen::MatrixXf connectivityMatrix = Eigen::MatrixXf::Zero(
            listOfEdges.size() * listOfEdges[0].dimension + listOfEdges[0].dimension,
            listOfNodes.size() * listOfNodes[0].dimension);
    for (int i = 0; i < listOfNodes[0].dimension; i++) {
        for (int j = 0; j < listOfNodes[i].dimension; j++) {
            connectivityMatrix(i, i) = -1;
        }
    }

//    connectivityMatrix <<   -1, 0, 0,
//                            1, -1, 0,
//                            0, 1, -1;
    for (int i = 0; i < listOfEdges.size(); i++) {
        for (int j = 0; j < listOfNodes[i].dimension; j++) {
            connectivityMatrix(i * listOfNodes[i].dimension + listOfNodes[i].dimension + j,
                               listOfNodes[i].dimension * listOfEdges[i].fromNode + j) = 1;
            connectivityMatrix(i * listOfNodes[i].dimension + listOfNodes[i].dimension + j,
                               listOfNodes[i].dimension * listOfEdges[i].toNode + j) = -1;
        }
    }
    std::cout << "connectivityMatrix" << std::endl;
    std::cout << connectivityMatrix << std::endl;
    return connectivityMatrix;
}

Eigen::MatrixXf getJacobianMatrix(std::vector<node> &listOfNodes, std::vector<edge> &listOfEdges) {
    Eigen::MatrixXf jacobianMatrix = Eigen::MatrixXf::Zero(listOfNodes.size(), listOfNodes.size());
//    jacobianMatrix << -1, 0, 0,
//                        1, -1, 0,
//                        0, 1, -1;
    jacobianMatrix = getConectivityMatrix(listOfNodes, listOfEdges);
    return jacobianMatrix;
}

Eigen::MatrixXf getErrorMatrix(std::vector<node> &listOfNodes, std::vector<edge> &listOfEdges) {

    Eigen::MatrixXf stateTMP;
    stateTMP.resize(listOfNodes.size() * listOfNodes[0].dimension, 1);
    for (int i = 0; i < listOfNodes.size(); i++) {
        for (int j = 0; j < listOfNodes[i].dimension; j++) {
            stateTMP(i * listOfNodes[i].dimension + j, 0) = listOfNodes[i].stateNode[j];
        }
    }

    Eigen::MatrixXf error = Eigen::MatrixXf::Zero(
            listOfEdges.size() * listOfNodes[0].dimension + listOfNodes[0].dimension, 1);//add an additional size for x0
    for (int i = 0; i < listOfEdges.size(); i++) {
        for (int j = 0; j < listOfNodes[i].dimension; j++) {
            error(i * listOfNodes[i].dimension + j + listOfNodes[i].dimension,
                  0) = listOfEdges[i].measurementDifference[j];
        }
    }
    error = error + getConectivityMatrix(listOfNodes, listOfEdges) * stateTMP;
    return error;
}

void addToState(std::vector<node> &listOfNodes, Eigen::MatrixXf &vectorToAdd) {
    for (int i = 0; i < listOfNodes.size(); i++) {
        for (int j = 0; j < listOfNodes[i].dimension; j++) {
            listOfNodes[i].stateNode[j] += (float) vectorToAdd(i * listOfNodes[i].dimension + j, 0);
        }
    }
}

void printCurrentState(std::vector<node> &listOfNodes) {
    std::cout << "current State:" << std::endl;
    for (int i = 0; i < listOfNodes.size(); i++) {
        for (int j = 0; j < listOfNodes[i].dimension; j++) {
            std::cout << listOfNodes[i].stateNode[j] << std::endl;
        }
    }
}

int
main(int argc, char **argv) {

    std::vector<node> listNodes;
    std::vector<edge> listEdge;
    const int dimension = 2;
    for (int i = 0; i < 4; i++) {
        node tmpNode(i, dimension);
        float *stateNode = new float[dimension];
        stateNode[0] = ((float) i + 1.0f) / 2.0f - 0.5f;
        stateNode[1] = ((float) -i + 1.0f) / 2.0f - 0.5f;
        tmpNode.stateNode = stateNode;
        tmpNode.dimension = dimension;
        listNodes.push_back(tmpNode);
    }
    std::cout << "\n" << std::endl;

    float *informationMeasurement1 = new float[dimension];
    float *measurementDifference1 = new float[dimension];
    informationMeasurement1[0] = 1;
    informationMeasurement1[1] = 1;
    measurementDifference1[0] = 1;//x value
    measurementDifference1[1] = 1;//y value
    edge tmpEdge1(0, 1, informationMeasurement1, measurementDifference1, dimension);
    listEdge.push_back(tmpEdge1);

    float *informationMeasurement2 = new float[dimension];
    float *measurementDifference2 = new float[dimension];
    informationMeasurement2[0] = 1;
    informationMeasurement2[1] = 1;
    measurementDifference2[0] = 1;//x value
    measurementDifference2[1] = -0.5;//y value
    edge tmpEdge2(1, 2, informationMeasurement2, measurementDifference2, dimension);
    listEdge.push_back(tmpEdge2);

    float *informationMeasurement3 = new float[dimension];
    float *measurementDifference3 = new float[dimension];
    informationMeasurement3[0] = 1;
    informationMeasurement3[1] = 1;
    measurementDifference3[0] = 2;
    measurementDifference3[1] = 1;
    edge tmpEdge3(0, 3, informationMeasurement3, measurementDifference3, dimension);
    listEdge.push_back(tmpEdge3);


    Eigen::MatrixXf errorMatrix;
    Eigen::MatrixXf informationMatrix;
    Eigen::MatrixXf jacobianMatrix;
    Eigen::MatrixXf bMatrix;
    Eigen::MatrixXf hMatrix;
    Eigen::MatrixXf vectorToAdd;
    const float gainVector = 0.5;


    std::cout << "informationMatrix:" << std::endl;
    std::cout << informationMatrix << std::endl;
    for (int i = 0; i < 50; i++) {
        printCurrentState(listNodes);

        informationMatrix = getInformationMatrix(listNodes, listEdge);
        errorMatrix = getErrorMatrix(listNodes, listEdge);
        jacobianMatrix = getJacobianMatrix(listNodes, listEdge);
        bMatrix = (errorMatrix.transpose() * informationMatrix * jacobianMatrix).transpose();
        hMatrix = jacobianMatrix.transpose() * informationMatrix * jacobianMatrix;
        vectorToAdd = gainVector * hMatrix.colPivHouseholderQr().solve(-bMatrix);
        addToState(listNodes, vectorToAdd);

        std::cout << "errorMatrix:" << std::endl;
        std::cout << errorMatrix << std::endl;
        std::cout << "jacobianMatrix:" << std::endl;
        std::cout << jacobianMatrix << std::endl;
        std::cout << "bMatrix:" << std::endl;
        std::cout << bMatrix << std::endl;
        std::cout << "hMatrix:" << std::endl;
        std::cout << hMatrix << std::endl;
        std::cout << "solve Hx=-b:" << std::endl;
        std::cout << vectorToAdd << std::endl;

    }
    printCurrentState(listNodes);
    return (0);
}

