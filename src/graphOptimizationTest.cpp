//
// Created by tim on 22.02.21.
//
#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <iostream>
#include "graphSlamSaveStructure.h"







int
main(int argc, char **argv) {
    const int dimension = 2;
    graphSlamSaveStructure graphSaved(dimension);


    for (int i = 0; i < 3; i++) {
        std::vector<float> stateNode;
        stateNode.push_back(((float) i + 1.0f) / 2.0f - 0.5f);
        stateNode.push_back(((float) -i + 1.0f) / 2.0f - 0.5f);
        graphSaved.addVertex(i,stateNode);
    }

    std::vector<float> informationMeasurement1 ;
    std::vector<float> measurementDifference1 ;
    informationMeasurement1.push_back(1);
    informationMeasurement1.push_back(1);
    measurementDifference1.push_back(1);//x value
    measurementDifference1.push_back(1);//y value

    graphSaved.addEdge(0, 1, informationMeasurement1, measurementDifference1);

    std::vector<float> informationMeasurement2 ;
    std::vector<float> measurementDifference2 ;
    informationMeasurement2.push_back(1);
    informationMeasurement2.push_back(1);
    measurementDifference2.push_back(1);//x value
    measurementDifference2.push_back(-0.5);//y value

    graphSaved.addEdge(1, 2, measurementDifference2, measurementDifference2);

    std::vector<float> informationMeasurement3 ;
    std::vector<float> measurementDifference3 ;
    informationMeasurement3.push_back(1);
    informationMeasurement3.push_back(1);
    measurementDifference3.push_back(2.5);//x value
    measurementDifference3.push_back(1);//y value

    graphSaved.addEdge(0, 2, informationMeasurement3, measurementDifference3);


    Eigen::MatrixXf errorMatrix;
    Eigen::MatrixXf informationMatrix;
    Eigen::MatrixXf jacobianMatrix;
    Eigen::MatrixXf bMatrix;
    Eigen::MatrixXf hMatrix;
    Eigen::MatrixXf vectorToAdd;
    const float gainVector = 0.5;

    for (int i = 0; i < 50; i++) {
        graphSaved.printCurrentState();

        informationMatrix = graphSaved.getInformationMatrix();
        errorMatrix = graphSaved.getErrorMatrix();
        jacobianMatrix = graphSaved.getJacobianMatrix();

        bMatrix = (errorMatrix.transpose() * informationMatrix * jacobianMatrix).transpose();
        hMatrix = jacobianMatrix.transpose() * informationMatrix * jacobianMatrix;
        vectorToAdd = gainVector * hMatrix.colPivHouseholderQr().solve(-bMatrix);

        graphSaved.addToState(vectorToAdd);

        std::cout << "errorMatrix:" << std::endl;
        std::cout << errorMatrix << std::endl;
//        std::cout << "jacobianMatrix:" << std::endl;
//        std::cout << jacobianMatrix << std::endl;
        std::cout << "bMatrix:" << std::endl;
        std::cout << bMatrix << std::endl;
        std::cout << "hMatrix:" << std::endl;
        std::cout << hMatrix << std::endl;
        std::cout << "solve Hx=-b:" << std::endl;
        std::cout << vectorToAdd << std::endl;

    }
    graphSaved.printCurrentState();
    return (0);
}

