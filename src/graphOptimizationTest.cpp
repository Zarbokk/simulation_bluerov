//
// Created by tim on 22.02.21.
//
#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <iostream>
#include "graphSlamSaveStructure.h"


void createExampleVertex(graphSlamSaveStructure &graphSaved) {
    Eigen::Vector3f positionVertex0(0, 0, 0);
    Eigen::AngleAxisf rotation_vector0(0.0f, Eigen::Vector3f(0, 0, 1));
    Eigen::Quaternionf rotationVertex0(rotation_vector0);
    graphSaved.addVertex(0, positionVertex0, rotationVertex0);

    Eigen::Vector3f positionVertex1(1, 1, 0);
    Eigen::AngleAxisf rotation_vector1(-90.0f / 180.0f * 3.14159f, Eigen::Vector3f(0, 0, 1));
    Eigen::Quaternionf rotationVertex1(rotation_vector1.toRotationMatrix());
    graphSaved.addVertex(1, positionVertex1, rotationVertex1);

    Eigen::Vector3f positionVertex2(0, 0, 0);
    Eigen::AngleAxisf rotation_vector2(-35.0f / 180.0f * 3.14159f, Eigen::Vector3f(0, 0, 1));
    Eigen::Quaternionf rotationVertex2(rotation_vector2.toRotationMatrix());
    graphSaved.addVertex(2, positionVertex2, rotationVertex2);

    Eigen::Vector3f positionVertex3(0, 0, 0);
    Eigen::AngleAxisf rotation_vector3(100.0f / 180.0f * 3.14159f, Eigen::Vector3f(0, 0, 1));
    Eigen::Quaternionf rotationVertex3(rotation_vector3.toRotationMatrix());
    graphSaved.addVertex(3, positionVertex3, rotationVertex3);

    graphSaved.addVertex(4, positionVertex0, rotationVertex0);

}

void createExampleEdge(graphSlamSaveStructure &graphSaved) {


    Eigen::Vector3f positionDifference0(8, 10, 0);
    Eigen::AngleAxisf rotation_vector0(-90.0f / 180.0f * 3.14159f, Eigen::Vector3f(0, 0, 1));
    Eigen::Quaternionf rotationDifference0(rotation_vector0.toRotationMatrix());
    float covariancePosition0 = 1;
    float covarianceQuaternion0 = 1;
    graphSaved.addEdge(0, 1, positionDifference0, rotationDifference0, covariancePosition0, covarianceQuaternion0);

    Eigen::Vector3f positionDifference1(4, 9, 0);
    Eigen::AngleAxisf rotation_vector1(55.0f / 180.0f * 3.14159f, Eigen::Vector3f(0, 0, 1));
    Eigen::Quaternionf rotationDifference1(rotation_vector1.toRotationMatrix());
    float covariancePosition1 = 1;
    float covarianceQuaternion1 = 1;
    graphSaved.addEdge(1, 2, positionDifference1, rotationDifference1, covariancePosition1, covarianceQuaternion1);

    Eigen::Vector3f positionDifference2(0, 12.4, 0);
    Eigen::AngleAxisf rotation_vector2(145.0f / 180.0f * 3.14159f, Eigen::Vector3f(0, 0, 1));
    Eigen::Quaternionf rotationDifference2(rotation_vector2.toRotationMatrix());
    float covariancePosition2 = 1;
    float covarianceQuaternion2 = 1;
    graphSaved.addEdge(2, 3, positionDifference2, rotationDifference2, covariancePosition2, covarianceQuaternion2);

    Eigen::Vector3f positionDifference3(2, -9, 0);
    Eigen::AngleAxisf rotation_vector3(90.0f / 180.0f * 3.14159f, Eigen::Vector3f(0, 0, 1));
    Eigen::Quaternionf rotationDifference3(rotation_vector3.toRotationMatrix());
    float covariancePosition3 = 1;
    float covarianceQuaternion3 = 1;
    graphSaved.addEdge(3, 4, positionDifference3, rotationDifference3, covariancePosition3, covarianceQuaternion3);


    Eigen::Vector3f positionDifference4(-7, 14, 0);
    Eigen::AngleAxisf rotation_vector4(180.0f / 180.0f * 3.14159f, Eigen::Vector3f(0, 0, 1));
    Eigen::Quaternionf rotationDifference4(rotation_vector4.toRotationMatrix());
    float covariancePosition4 = 1;
    float covarianceQuaternion4 = 1;
    //graphSaved.addEdge(1, 3, positionDifference4, rotationDifference4, covariancePosition4, covarianceQuaternion4);

}


int
main(int argc, char **argv) {
    const int dimension = 3;
    graphSlamSaveStructure graphSaved(dimension);


    createExampleVertex(graphSaved);

    createExampleEdge(graphSaved);


    Eigen::MatrixXf errorMatrix;
    Eigen::MatrixXf informationMatrix;
    Eigen::MatrixXf jacobianMatrix;
    Eigen::MatrixXf bMatrix;
    Eigen::MatrixXf hMatrix;
    Eigen::MatrixXf vectorToAdd;
    const float gainVector = 0.5;

    for (int i = 0; i < 20; i++) {
//        graphSaved.printCurrentState();

        informationMatrix = graphSaved.getInformationMatrix();
        errorMatrix = graphSaved.getErrorMatrix();
        jacobianMatrix = graphSaved.getJacobianMatrix();



//        std::cout << "errorMatrix:" << std::endl;
//        std::cout << errorMatrix << std::endl;
//        std::cout << "informationMatrix:" << std::endl;
//        std::cout << informationMatrix << std::endl;
//        std::cout << "jacobianMatrix:" << std::endl;
//        std::cout << jacobianMatrix << std::endl;



        bMatrix = (errorMatrix.transpose() * informationMatrix * jacobianMatrix).transpose();
        hMatrix = jacobianMatrix.transpose() * informationMatrix * jacobianMatrix;
        vectorToAdd = gainVector * hMatrix.colPivHouseholderQr().solve(-bMatrix);

        graphSaved.addToState(vectorToAdd);

//        std::cout << "bMatrix:" << std::endl;
//        std::cout << bMatrix << std::endl;
//        std::cout << "hMatrix:" << std::endl;
//        std::cout << hMatrix << std::endl;
//        std::cout << "solve Hx=-b:" << std::endl;
//        std::cout << vectorToAdd << std::endl;
        std::cout << "complete error: "<< errorMatrix.norm() << std::endl;
    }

    graphSaved.printCurrentState();
    std::cout << "complete error: "<< errorMatrix.norm() << std::endl;

    return (0);
}

