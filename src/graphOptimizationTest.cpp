//
// Created by tim on 22.02.21.
//
#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <iostream>
class edge {
    public:
        edge(int fromNode,int toNode,float *informationMeasurement,float *measurementDifference, int dimension)
        {
            edge::fromNode = fromNode;
            edge::toNode = toNode;
            edge::informationMeasurement = new float [dimension];
            edge::measurementDifference = new float [dimension];
            for (int i = 0 ; i<dimension;i++){
                edge::informationMeasurement[i]=informationMeasurement[i];
                edge::measurementDifference[i]=measurementDifference[i];
            }
        }

        int fromNode;
        int toNode;
        float *informationMeasurement;
        float *measurementDifference;
    private:

};

class node {
public:
    node(int nodeNumber,int dimension)
    {
        node::nodeNumber = nodeNumber;
        node::stateNode = new float [dimension];
        node::informationState = new float [dimension];
    }

    int nodeNumber;
    float *stateNode;
    float *informationState;
private:

};

Eigen::MatrixXf getInformationMatrix(std::vector<node> &listOfNodes,std::vector<edge> &listOfEdges){
    Eigen::MatrixXf informationMatrix;
    informationMatrix.resize(listOfEdges.size()+1, listOfEdges.size()+1);
    informationMatrix <<    1, 0, 0,
                            0, 1, 0,
                            0, 0, 1;
    return informationMatrix;
}

Eigen::MatrixXf getJacobianMatrix(std::vector<node> &listOfNodes,std::vector<edge> &listOfEdges){
    Eigen::MatrixXf jacobianMatrix;
    jacobianMatrix.resize(listOfNodes.size(), listOfNodes.size());
    jacobianMatrix <<   -1, 0, 0,
                        1, -1, 0,
                        0, 1, -1;
    return jacobianMatrix;
}

Eigen::MatrixXf getConectivityMatrix(std::vector<node> &listOfNodes,std::vector<edge> &listOfEdges){
    Eigen::MatrixXf connectivityMatrix;
    connectivityMatrix.resize(listOfNodes.size(), listOfNodes.size());
    connectivityMatrix <<   -1, 0, 0,
                            1, -1, 0,
                            0, 1, -1;
    return connectivityMatrix;
}


Eigen::MatrixXf getErrorMatrix(std::vector<node> &listOfNodes,std::vector<edge> &listOfEdges){

    int sizeAllNodes = listOfNodes.size();
    Eigen::MatrixXf stateTMP;
    stateTMP.resize(sizeAllNodes, 1);
    for (int i = 0 ; i < sizeAllNodes;i++){
        stateTMP(i,0)=listOfNodes[i].stateNode[0];
    }

    int sizeAllEdges = listOfEdges.size();
    Eigen::MatrixXf error;
    error.resize(sizeAllEdges+1, 1);//add an additional size for x0
    error(0,0)=0.0f;
    for (int i = 0 ; i < sizeAllEdges;i++){
        error(i+1,0)=listOfEdges[i].measurementDifference[0];
    }
    error=error+getConectivityMatrix(listOfNodes,listOfEdges)*stateTMP;
    return error;
}

void addToState(std::vector<node> &listOfNodes,Eigen::MatrixXf &vectorToAdd){
    for (int i = 0 ; i < vectorToAdd.size();i++){
        listOfNodes[i].stateNode[0] +=(float)vectorToAdd(i,0);
    }
}
void printCurrentState(std::vector<node> &listOfNodes){
    std::cout << "current State:"<< std::endl;
    for (int i = 0 ; i < listOfNodes.size();i++){
        std::cout << listOfNodes[i].stateNode[0]<< std::endl;
    }
}

int
main(int argc, char **argv) {

    std::vector<node> listNodes;
    std::vector<edge> listEdge;
    for (int i =0 ; i<3 ; i++){
        node tmpNode(i,1);
        float *stateNode = new float[1];
        stateNode[0]=((float)i+1.0f)/2.0f-0.5f;
        tmpNode.stateNode = stateNode;
        listNodes.push_back(tmpNode);
    }
    std::cout << "\n" << std::endl;
    float *informationMeasurement1 = new float[1];
    float *measurementDifference1 = new float[1];
    informationMeasurement1[0]=1;
    measurementDifference1[0]=1;
    edge tmpEdge1(0,1,informationMeasurement1,measurementDifference1,1);
    listEdge.push_back(tmpEdge1);

    float *informationMeasurement2 = new float[1];
    float *measurementDifference2 = new float[1];
    informationMeasurement2[0]=1;
    measurementDifference2[0]=2;
    edge tmpEdge2(0,1,informationMeasurement2,measurementDifference2,1);
    listEdge.push_back(tmpEdge2);

    Eigen::MatrixXf errorMatrix;
    Eigen::MatrixXf informationMatrix;
    Eigen::MatrixXf jacobianMatrix;
    Eigen::MatrixXf bMatrix;
    Eigen::MatrixXf hMatrix;
    Eigen::MatrixXf vectorToAdd;
    const float gainVector=0.2;
    for (int i = 0; i<30;i++){
        printCurrentState(listNodes);


        errorMatrix=getErrorMatrix(listNodes,listEdge);
        informationMatrix = getInformationMatrix(listNodes,listEdge);
        jacobianMatrix = getJacobianMatrix(listNodes,listEdge);
        bMatrix=(errorMatrix.transpose()*informationMatrix*jacobianMatrix).transpose();
        hMatrix=jacobianMatrix.transpose()*informationMatrix*jacobianMatrix;
        vectorToAdd= gainVector*hMatrix.colPivHouseholderQr().solve(-bMatrix);
        addToState(listNodes,vectorToAdd);
//        std::cout << "errorMatrix:" << std::endl;
//        std::cout << errorMatrix << std::endl;
//        std::cout << "informationMatrix:" << std::endl;
//        std::cout << informationMatrix << std::endl;
//        std::cout << "jacobianMatrix:" << std::endl;
//        std::cout << jacobianMatrix << std::endl;
//        std::cout << "bMatrix:" << std::endl;
//        std::cout << bMatrix << std::endl;
//        std::cout << "hMatrix:" << std::endl;
//        std::cout << hMatrix << std::endl;
//        std::cout << "solve Hx=-b:" << std::endl;
//        std::cout << vectorToAdd << std::endl;

    }
    printCurrentState(listNodes);
    return (0);
}

