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

std::vector<float> vertex::getStateVertex() const {
    return vertex::stateVertex;
}

void vertex::setStateVertex(std::vector<float> &stateVertex) {
    vertex::stateVertex = stateVertex;
}
