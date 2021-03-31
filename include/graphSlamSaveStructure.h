//
// Created by tim on 23.02.21.
//
#include "edge.h"
#include "vertex.h"
#include<Eigen/SparseCholesky>

#ifndef SIMULATION_BLUEROV_GRAPHSLAMSAVESTRUCTURE_H
#define SIMULATION_BLUEROV_GRAPHSLAMSAVESTRUCTURE_H


class graphSlamSaveStructure {
public:
    graphSlamSaveStructure(int degreeOfFreedom) {
        if (degreeOfFreedom == 3) {
            graphSlamSaveStructure::degreeOfFreedom = degreeOfFreedom;
            graphSlamSaveStructure::numberOfEdges = 0;
            graphSlamSaveStructure::numberOfVertex = 0;
            hasHierachicalGraph = false;
        } else {
            std::cout << "not yet implemented DOF 6" << std::endl;
            std::exit(-1);
        }
    }

    void addEdge(const int fromVertex, const int toVertex, const Eigen::Vector3f &positionDifference,
                 const Eigen::Quaternionf &rotationDifference, const Eigen::Vector3f covariancePosition,
                 const float covarianceQuaternion);

    void addEdge(const int fromVertex, const int toVertex, const Eigen::Vector3f &positionDifference,
                 const Eigen::Quaternionf &rotationDifference, const Eigen::Vector3f covariancePosition,
                 const float covarianceQuaternion, pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloud);

    void addVertex(int vertexNumber, const Eigen::Vector3f &positionVertex, const Eigen::Quaternionf &rotationVertex,
                   const Eigen::Vector3f &covariancePosition, const float covarianceQuaternion);

    void addVertex(int vertexNumber, const Eigen::Vector3f &positionVertex, const Eigen::Quaternionf &rotationVertex,
                   const Eigen::Vector3f &covariancePosition, const float covarianceQuaternion,
                   pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloud);

    Eigen::SparseMatrix<float> getInformationMatrix();

    Eigen::SparseMatrix<float> getJacobianMatrix();

    Eigen::MatrixXf getErrorMatrix();

    void printCurrentState();

    void printCurrentStateGeneralInformation();

    void addToEveryState(std::vector<Eigen::Vector3f> &positionDifferenceVector,
                         std::vector<Eigen::Quaternionf> &rotationDifferenceVector);

    void addToState(Eigen::MatrixXf &vectorToAdd);//from optimization

    vertex getVertexByIndex(int i);

    std::vector<vertex> getVertexList();

    void optimizeGraphWithSlam(bool verbose, std::vector<int> &holdStill);

    void optimizeGraphWithSlamTopDown(bool verbose);

    void initiallizeSubGraphs(std::deque<float> cellSizes);//cell sizes in order (first 1m cecond 4 m and so on)

    void createHierachicalGraph(float cellSizeDes);

    edge getEdgeBetweenNodes(int fromVertex, int toVertex, std::vector<int> &holdStill);

    Eigen::MatrixXf transformStateDiffToAddVector(std::vector<vertex> &stateBeforeOptimization,
                                                  std::vector<vertex> &stateAfterOptimization) const;

    graphSlamSaveStructure getSubGraph();

    bool createSubGraphBetweenCell(int vertexIndexFrom, int vertexIndexTo, graphSlamSaveStructure &currentSubGraph);

    void calculateCovarianceInCloseProximity();// calculate the subgraph for the last hierachical graph, and connected cells. then calculate covariances.

    static const int POINT_CLOUD_USAGE = 0;
    static const int INTEGRATED_POS_USAGE = 1;
private:

    void removeRowColumn(Eigen::SparseMatrix<float> &matrix, int rowToRemove) const;

    void lookupTableCreation(float minDistanceForNewCell);

    void
    getListofConnectedVertexAndEdges(std::vector<int> &vertexIndicesofICell, std::vector<edge> &listOfContainingEdges,
                                     std::vector<int> &listOfConnectedVertexes);

    int getCellOfVertexIndex(int vertexIndex);

    static bool checkIfElementsOfVectorAreEqual(std::vector<int> &i0, std::vector<int> &i1);

    static std::vector<int> joinTwoLists(std::vector<int> &i0, std::vector<int> &i1);

    bool checkIfDirectConnectionExists(int vertexIndex0,int vertexIndex1);

    int degreeOfFreedom;//3 for [x y alpha] or 6 for [x y z alpha beta gamma]
    int numberOfEdges;
    int numberOfVertex;
    std::vector<edge> edgeList;
    std::vector<vertex> vertexList;
    bool hasHierachicalGraph;
    float cellSize;
    std::vector<std::vector<int>> lookUpTableCell;// [i][j] i = cell j=vertex (cell of sub graph and vertex of graph)
    graphSlamSaveStructure *hierachicalGraph;
};


#endif //SIMULATION_BLUEROV_GRAPHSLAMSAVESTRUCTURE_H
