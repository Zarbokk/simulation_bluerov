//
// Created by tim on 23.02.21.
//

#ifndef SIMULATION_BLUEROV_VETREX_H
#define SIMULATION_BLUEROV_VETREX_H
#include <vector>

class vertex {
public:
    vertex(int vertexNumber,const std::vector<float> stateVertex, int dimension) {
        vertex::vertexNumber = vertexNumber;
        vertex::stateVertex = stateVertex;
    }

    int getVertexNumber() const;

    void setVertexNumber(int vertexNumber);

    std::vector<float> getStateVertex() const;

    void setStateVertex(std::vector<float> &stateVertex);




private:
    int vertexNumber;
    std::vector<float> stateVertex;
    //float *informationState; // potentionally useful for estimated accuracy
};


#endif //SIMULATION_BLUEROV_VETREX_H
