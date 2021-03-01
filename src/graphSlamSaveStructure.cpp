//
// Created by tim on 23.02.21.
//

#include "graphSlamSaveStructure.h"

void
graphSlamSaveStructure::addEdge(const int fromVertex, const int toVertex, const Eigen::Vector3f &positionDifference,
                                const Eigen::Quaternionf &rotationDifference, const float covariancePosition,
                                const float covarianceQuaternion) {
    edge edgeToAdd(fromVertex, toVertex, positionDifference, rotationDifference, covariancePosition,
                   covarianceQuaternion,
                   graphSlamSaveStructure::degreeOfFreedom);
    graphSlamSaveStructure::numberOfEdges += 1;
    graphSlamSaveStructure::edgeList.push_back(edgeToAdd);

}

void
graphSlamSaveStructure::addEdge(const int fromVertex, const int toVertex, const Eigen::Vector3f &positionDifference,
                                const Eigen::Quaternionf &rotationDifference, const float covariancePosition,
                                const float covarianceQuaternion, pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloud) {
    edge edgeToAdd(fromVertex, toVertex, positionDifference, rotationDifference, covariancePosition,
                   covarianceQuaternion,
                   graphSlamSaveStructure::degreeOfFreedom, pointCloud);
    graphSlamSaveStructure::numberOfEdges += 1;
    graphSlamSaveStructure::edgeList.push_back(edgeToAdd);

}

void graphSlamSaveStructure::addVertex(int vertexNumber, const Eigen::Vector3f &positionVertex,
                                       Eigen::Quaternionf &rotationVertex,
                                       float covariancePosition, float covarianceQuaternion) {

    vertex vertexToAdd(vertexNumber, positionVertex, rotationVertex, graphSlamSaveStructure::degreeOfFreedom,
                       covariancePosition, covarianceQuaternion);
    graphSlamSaveStructure::numberOfVertex += 1;
    graphSlamSaveStructure::vertexList.push_back(vertexToAdd);

}

void graphSlamSaveStructure::addVertex(int vertexNumber, const Eigen::Vector3f &positionVertex,
                                       Eigen::Quaternionf &rotationVertex,
                                       float covariancePosition, float covarianceQuaternion,
                                       pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloud) {

    vertex vertexToAdd(vertexNumber, positionVertex, rotationVertex, graphSlamSaveStructure::degreeOfFreedom,
                       pointCloud, covariancePosition, covarianceQuaternion);
    graphSlamSaveStructure::numberOfVertex += 1;
    graphSlamSaveStructure::vertexList.push_back(vertexToAdd);


}

Eigen::MatrixXf graphSlamSaveStructure::getInformationMatrix() {
    Eigen::MatrixXf informationMatrix = Eigen::MatrixXf::Zero(
            graphSlamSaveStructure::numberOfEdges * graphSlamSaveStructure::degreeOfFreedom +
            graphSlamSaveStructure::degreeOfFreedom,
            graphSlamSaveStructure::numberOfEdges * graphSlamSaveStructure::degreeOfFreedom +
            graphSlamSaveStructure::degreeOfFreedom);

    for (int j = 0; j < graphSlamSaveStructure::degreeOfFreedom; j++) {
        informationMatrix(j, j) = 1000000;//first indece
    }
    for (int i = 1; i <= graphSlamSaveStructure::numberOfEdges; i++) {
        informationMatrix(i * graphSlamSaveStructure::degreeOfFreedom + 0,
                          i * graphSlamSaveStructure::degreeOfFreedom + 0) =
                1 / graphSlamSaveStructure::edgeList[i - 1].getCovariancePosition();

        informationMatrix(i * graphSlamSaveStructure::degreeOfFreedom + 1,
                          i * graphSlamSaveStructure::degreeOfFreedom + 1) =
                1 / graphSlamSaveStructure::edgeList[i - 1].getCovariancePosition();

        informationMatrix(i * graphSlamSaveStructure::degreeOfFreedom + 2,
                          i * graphSlamSaveStructure::degreeOfFreedom + 2) =
                1 / graphSlamSaveStructure::edgeList[i - 1].getCovarianceQuaternion();
    }
    return informationMatrix;
}


Eigen::MatrixXf graphSlamSaveStructure::getJacobianMatrix() {
    Eigen::MatrixXf jacobianMatrix = Eigen::MatrixXf::Zero(
            graphSlamSaveStructure::numberOfEdges * graphSlamSaveStructure::degreeOfFreedom +
            graphSlamSaveStructure::degreeOfFreedom,
            graphSlamSaveStructure::numberOfVertex * graphSlamSaveStructure::degreeOfFreedom);

    for (int i = 0; i < graphSlamSaveStructure::degreeOfFreedom; i++) {
        jacobianMatrix(i, i) = 1;
    }

    for (int i = 0; i < graphSlamSaveStructure::numberOfEdges; i++) {

        int fromVertex = graphSlamSaveStructure::edgeList[i].getFromVertex();
        int toVertex = graphSlamSaveStructure::edgeList[i].getToVertex();
        Eigen::Vector3f eulerAnglesRotation = graphSlamSaveStructure::vertexList[fromVertex].getRotationVertex().toRotationMatrix().eulerAngles(
                0, 1, 2);
        float alpha1 = eulerAnglesRotation[2];
        float x1 = graphSlamSaveStructure::vertexList[fromVertex].getPositionVertex().x();
        float y1 = graphSlamSaveStructure::vertexList[fromVertex].getPositionVertex().y();

        float x2 = graphSlamSaveStructure::vertexList[toVertex].getPositionVertex().x();
        float y2 = graphSlamSaveStructure::vertexList[toVertex].getPositionVertex().y();


        //first row

        jacobianMatrix(i * graphSlamSaveStructure::degreeOfFreedom + graphSlamSaveStructure::degreeOfFreedom + 0,
                       fromVertex * graphSlamSaveStructure::degreeOfFreedom + 0) = -cos(
                alpha1);//first collumn of fromVertex
        jacobianMatrix(i * graphSlamSaveStructure::degreeOfFreedom + graphSlamSaveStructure::degreeOfFreedom + 0,
                       fromVertex * graphSlamSaveStructure::degreeOfFreedom + 1) = -sin(
                alpha1);//second collumn of fromVertex
        jacobianMatrix(i * graphSlamSaveStructure::degreeOfFreedom + graphSlamSaveStructure::degreeOfFreedom + 0,
                       fromVertex * graphSlamSaveStructure::degreeOfFreedom + 2) =
                -sin(alpha1) * (x2 - x1) + cos(alpha1) * (y2 - y1);//third collumn of fromVertex

        jacobianMatrix(i * graphSlamSaveStructure::degreeOfFreedom + graphSlamSaveStructure::degreeOfFreedom + 0,
                       toVertex * graphSlamSaveStructure::degreeOfFreedom + 0) = cos(alpha1);//first collumn of toVertex
        jacobianMatrix(i * graphSlamSaveStructure::degreeOfFreedom + graphSlamSaveStructure::degreeOfFreedom + 0,
                       toVertex * graphSlamSaveStructure::degreeOfFreedom + 1) = sin(
                alpha1);//second collumn of toVertex
        jacobianMatrix(i * graphSlamSaveStructure::degreeOfFreedom + graphSlamSaveStructure::degreeOfFreedom + 0,
                       toVertex * graphSlamSaveStructure::degreeOfFreedom + 2) = 0;//third collumn of toVertex


        //second row

        jacobianMatrix(i * graphSlamSaveStructure::degreeOfFreedom + graphSlamSaveStructure::degreeOfFreedom + 1,
                       fromVertex * graphSlamSaveStructure::degreeOfFreedom + 0) = sin(
                alpha1);//first collumn of fromVertex
        jacobianMatrix(i * graphSlamSaveStructure::degreeOfFreedom + graphSlamSaveStructure::degreeOfFreedom + 1,
                       fromVertex * graphSlamSaveStructure::degreeOfFreedom + 1) = -cos(
                alpha1);//second collumn of fromVertex
        jacobianMatrix(i * graphSlamSaveStructure::degreeOfFreedom + graphSlamSaveStructure::degreeOfFreedom + 1,
                       fromVertex * graphSlamSaveStructure::degreeOfFreedom + 2) =
                -cos(alpha1) * (x2 - x1) - sin(alpha1) * (y2 - y1);//third collumn of fromVertex

        jacobianMatrix(i * graphSlamSaveStructure::degreeOfFreedom + graphSlamSaveStructure::degreeOfFreedom + 1,
                       toVertex * graphSlamSaveStructure::degreeOfFreedom + 0) = -sin(
                alpha1);//first collumn of toVertex
        jacobianMatrix(i * graphSlamSaveStructure::degreeOfFreedom + graphSlamSaveStructure::degreeOfFreedom + 1,
                       toVertex * graphSlamSaveStructure::degreeOfFreedom + 1) = cos(
                alpha1);//second collumn of toVertex
        jacobianMatrix(i * graphSlamSaveStructure::degreeOfFreedom + graphSlamSaveStructure::degreeOfFreedom + 1,
                       toVertex * graphSlamSaveStructure::degreeOfFreedom + 2) = 0;//third collumn of toVertex


        //third row

        jacobianMatrix(i * graphSlamSaveStructure::degreeOfFreedom + graphSlamSaveStructure::degreeOfFreedom + 2,
                       fromVertex * graphSlamSaveStructure::degreeOfFreedom + 0) = 0;//first collumn of fromVertex
        jacobianMatrix(i * graphSlamSaveStructure::degreeOfFreedom + graphSlamSaveStructure::degreeOfFreedom + 2,
                       fromVertex * graphSlamSaveStructure::degreeOfFreedom + 1) = 0;//second collumn of fromVertex
        jacobianMatrix(i * graphSlamSaveStructure::degreeOfFreedom + graphSlamSaveStructure::degreeOfFreedom + 2,
                       fromVertex * graphSlamSaveStructure::degreeOfFreedom + 2) = -1;//third collumn of fromVertex

        jacobianMatrix(i * graphSlamSaveStructure::degreeOfFreedom + graphSlamSaveStructure::degreeOfFreedom + 2,
                       toVertex * graphSlamSaveStructure::degreeOfFreedom + 0) = 0;//first collumn of toVertex
        jacobianMatrix(i * graphSlamSaveStructure::degreeOfFreedom + graphSlamSaveStructure::degreeOfFreedom + 2,
                       toVertex * graphSlamSaveStructure::degreeOfFreedom + 1) = 0;//second collumn of toVertex
        jacobianMatrix(i * graphSlamSaveStructure::degreeOfFreedom + graphSlamSaveStructure::degreeOfFreedom + 2,
                       toVertex * graphSlamSaveStructure::degreeOfFreedom + 2) = 1;//third collumn of toVertex

    }

    return -jacobianMatrix;
}

float angleDiff(float first, float second) {//first-second
    return atan2(sin(first - second), cos(first - second));
}

Eigen::MatrixXf graphSlamSaveStructure::getErrorMatrix() {

    //this is for DOF 3
    Eigen::MatrixXf error = Eigen::MatrixXf::Zero(
            graphSlamSaveStructure::numberOfEdges * graphSlamSaveStructure::degreeOfFreedom +
            graphSlamSaveStructure::degreeOfFreedom, 1);//add an additional size for x0
    for (int i = 0; i < graphSlamSaveStructure::numberOfEdges; i++) {

        int fromVertex = graphSlamSaveStructure::edgeList[i].getFromVertex();
        int toVertex = graphSlamSaveStructure::edgeList[i].getToVertex();

        Eigen::Vector3f eulerAnglesRotation = graphSlamSaveStructure::vertexList[fromVertex].getRotationVertex().toRotationMatrix().eulerAngles(
                0, 1, 2);
        Eigen::AngleAxisf rotation_vector1(-eulerAnglesRotation(2), Eigen::Vector3f(0, 0, 1));
        Eigen::Matrix2f currentRotation = rotation_vector1.matrix().block<2, 2>(0, 0);


        Eigen::Vector2f differenceBeforeRotation =
                graphSlamSaveStructure::vertexList[toVertex].getPositionVertex().block<2, 1>(0, 0) -
                graphSlamSaveStructure::vertexList[fromVertex].getPositionVertex().block<2, 1>(0, 0);


        error.block<2, 1>(i * graphSlamSaveStructure::degreeOfFreedom + graphSlamSaveStructure::degreeOfFreedom, 0) =
                graphSlamSaveStructure::edgeList[i].getPositionDifference().block<2, 1>(0, 0) -
                currentRotation * differenceBeforeRotation;
        //calculate angle difference

        float angleDiffVertex = angleDiff(
                graphSlamSaveStructure::vertexList[toVertex].getRotationVertex().toRotationMatrix().eulerAngles(0, 1,
                                                                                                                2)[2],
                eulerAnglesRotation(2));

        float angleErrorDiff = angleDiff(
                graphSlamSaveStructure::edgeList[i].getRotationDifference().toRotationMatrix().eulerAngles(0, 1, 2)[2],
                angleDiffVertex);

        error(i * graphSlamSaveStructure::degreeOfFreedom + graphSlamSaveStructure::degreeOfFreedom + 2,
              0) = angleErrorDiff;
    }
    return error;
}

void graphSlamSaveStructure::printCurrentState() {
    std::cout << "current State:" << std::endl;
    for (int i = 0; i < graphSlamSaveStructure::numberOfVertex; i++) {
        std::cout << "State:" << i << std::endl;
        //std::cout << "Pos:" << std::endl;
        //std::cout << graphSlamSaveStructure::vertexList[i].getPositionVertex() << std::endl;
        std::cout << "Rot:" << std::endl;
        std::cout <<
                  graphSlamSaveStructure::vertexList[i].getRotationVertex().toRotationMatrix().eulerAngles(0, 1, 2)[2] *
                  180 / M_PI << std::endl;
    }
}

void graphSlamSaveStructure::printCurrentStateGeneralInformation() {
    std::cout << "Number of Vertex:" << graphSlamSaveStructure::numberOfVertex << std::endl;
    for (int i = 0; i < graphSlamSaveStructure::numberOfVertex; i++) {
        std::cout << "Vertex Nr. " << i << " Covariance "<<graphSlamSaveStructure::vertexList[i].getCovariancePosition()<< std::endl;
    }

    std::cout << "Number of Edges:" << graphSlamSaveStructure::numberOfVertex << std::endl;
    for (int i = 0; i < graphSlamSaveStructure::numberOfEdges; i++) {
        std::cout << "Edge from " << graphSlamSaveStructure::edgeList[i].getFromVertex() << " to "
                  << graphSlamSaveStructure::edgeList[i].getToVertex() << " covariance Position "
                  << graphSlamSaveStructure::edgeList[i].getCovariancePosition() << std::endl;
    }

}

void graphSlamSaveStructure::addToState(std::vector<Eigen::Vector3f> &positionDifferenceVector,
                                        std::vector<Eigen::Quaternionf> &rotationDifferenceVector) {


    for (int i = 0; i < graphSlamSaveStructure::numberOfVertex; i++) {
        Eigen::Vector3f currentStatePos = graphSlamSaveStructure::vertexList[i].getPositionVertex();
        Eigen::Quaternionf currentStateRotation = graphSlamSaveStructure::vertexList[i].getRotationVertex();
//        Eigen::Vector3f eulerAnglesRotation = graphSlamSaveStructure::vertexList[i].getRotationVertex().toRotationMatrix().eulerAngles(
//                0, 1, 2);
//        float alpha1 = eulerAnglesRotation[2];


        graphSlamSaveStructure::vertexList[i].setPositionVertex(currentStatePos + positionDifferenceVector[i]);

        graphSlamSaveStructure::vertexList[i].setRotationVertex(currentStateRotation * rotationDifferenceVector[i]);
    }
}

void graphSlamSaveStructure::addToState(Eigen::MatrixXf &vectorToAdd) {


    for (int i = 0; i < graphSlamSaveStructure::numberOfVertex; i++) {
        float x = vectorToAdd(i * graphSlamSaveStructure::degreeOfFreedom + 0);
        float y = vectorToAdd(i * graphSlamSaveStructure::degreeOfFreedom + 1);
        float alpha = vectorToAdd(i * graphSlamSaveStructure::degreeOfFreedom + 2);

        Eigen::Vector3f currentStatePos = graphSlamSaveStructure::vertexList[i].getPositionVertex();
        Eigen::Quaternionf currentStateRotation = graphSlamSaveStructure::vertexList[i].getRotationVertex();


        Eigen::Vector3f addedStatePos(x, y, 0);
        Eigen::AngleAxisf rotation_vector1(alpha, Eigen::Vector3f(0, 0, 1));
        Eigen::Quaternionf addedRotation(rotation_vector1);


        graphSlamSaveStructure::vertexList[i].setPositionVertex(currentStatePos + addedStatePos);

        graphSlamSaveStructure::vertexList[i].setRotationVertex(currentStateRotation * addedRotation);
    }
}

vertex graphSlamSaveStructure::getVertexByIndex(const int i) {
    if (i >= graphSlamSaveStructure::numberOfVertex) {
        std::cout << "access to a vertex that doesnt exist" << std::endl;
        std::exit(-1);
    }
    return graphSlamSaveStructure::vertexList[i];
}

std::vector<vertex> graphSlamSaveStructure::getVertexList() {
    return graphSlamSaveStructure::vertexList;
}

void graphSlamSaveStructure::optimizeGraphWithSlam(){
    Eigen::MatrixXf errorMatrix;
    Eigen::MatrixXf informationMatrix;
    Eigen::MatrixXf jacobianMatrix;
    Eigen::MatrixXf bMatrix;
    Eigen::MatrixXf hMatrix;
    Eigen::MatrixXf vectorToAdd;
    const float gainVector = 0.5;

    for (int i = 0; i < 20; i++) {

        informationMatrix = graphSlamSaveStructure::getInformationMatrix();
        errorMatrix = graphSlamSaveStructure::getErrorMatrix();
        jacobianMatrix = graphSlamSaveStructure::getJacobianMatrix();


        bMatrix = (errorMatrix.transpose() * informationMatrix * jacobianMatrix).transpose();
        hMatrix = jacobianMatrix.transpose() * informationMatrix * jacobianMatrix;
        vectorToAdd = gainVector * hMatrix.colPivHouseholderQr().solve(-bMatrix);

        graphSlamSaveStructure::addToState(vectorToAdd);
        std::cout << "errorMatrix:" << std::endl;
        std::cout << errorMatrix << std::endl;
        std::cout << "informationMatrix:" << std::endl;
        std::cout << informationMatrix << std::endl;
        std::cout << "jacobianMatrix:" << std::endl;
        std::cout << jacobianMatrix << std::endl;
        std::cout << "bMatrix:" << std::endl;
        std::cout << bMatrix << std::endl;
        std::cout << "hMatrix:" << std::endl;
        std::cout << hMatrix << std::endl;
        std::cout << "Covariance Matrix:" << std::endl;
        std::cout << hMatrix.diagonal().asDiagonal().toDenseMatrix().inverse().diagonal() << std::endl;
        std::cout << "solve Hx=-b:" << std::endl;
        std::cout << vectorToAdd << std::endl;
        std::cout << "complete error: "<< errorMatrix.norm() << std::endl;
    }
}




