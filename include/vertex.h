//
// Created by tim on 23.02.21.
//

#ifndef SIMULATION_BLUEROV_VETREX_H
#define SIMULATION_BLUEROV_VETREX_H
//#include <vector>
#include <Eigen/Geometry>

class vertex {
public:
    vertex(int vertexNumber, const Eigen::Vector3f &positionVertex, Eigen::Quaternionf &rotationVertex, int degreeOfFreedom) {
        vertex::vertexNumber = vertexNumber;
        vertex::positionVertex = positionVertex;
        vertex::rotationVertex = rotationVertex;
    }

    int getVertexNumber() const;

    void setVertexNumber(int vertexNumber);

    const Eigen::Vector3f &getPositionVertex() const;

    void setPositionVertex(const Eigen::Vector3f &positionVertex);

    const Eigen::Quaternionf &getRotationVertex() const;

    void setRotationVertex(const Eigen::Quaternionf &rotationVertex);


private:
    int vertexNumber;
    Eigen::Vector3f positionVertex;// position w.r.t. Initiial Starting Position
    Eigen::Quaternionf rotationVertex;// rotation w.r.t. Initial Starting Rotation

    //float *informationState; // potentionally useful for estimated accuracy
};


#endif //SIMULATION_BLUEROV_VETREX_H
