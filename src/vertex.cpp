//
// Created by tim on 23.02.21.
//

#include "vertex.h"

int vertex::getVertexNumber() const {
    return vertex::vertexNumber;
}

void vertex::setVertexNumber(int vertexNumber) {
    vertex::vertexNumber = vertexNumber;
}

const Eigen::Vector3f &vertex::getPositionVertex() const {
    return positionVertex;
}

void vertex::setPositionVertex(const Eigen::Vector3f &positionVertex) {
    vertex::positionVertex = positionVertex;
}

const Eigen::Quaternionf &vertex::getRotationVertex() const {
    return rotationVertex;
}

void vertex::setRotationVertex(const Eigen::Quaternionf &rotationVertex) {
    vertex::rotationVertex = rotationVertex;
}


