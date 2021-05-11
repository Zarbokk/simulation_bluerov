//
// Created by tim on 11.05.21.
//

#include "hilbertMap.h"


void hilbertMap::createRandomMap() {
    this->currentMap.clear();
    for(int i = 0 ; i<this->numberOfPointsForEachDimension;i++){
        std::vector<double> tmpVector;
        this->currentMap.push_back(tmpVector);
        for(int j = 0 ; j<this->numberOfPointsForEachDimension;j++){
            double occupancy = 0.5;
            this->currentMap[i].push_back(occupancy);
        }
    }
}

double hilbertMap::getDiscritisationSize() const {
    return discritisationSize;
}

void hilbertMap::setDiscritisationSize(double discritisationSize) {
    hilbertMap::discritisationSize = discritisationSize;
}

int hilbertMap::getNumberOfPointsForEachDimension() const {
    return numberOfPointsForEachDimension;
}

void hilbertMap::setNumberOfPointsForEachDimension(int numberOfPointsForEachDimension) {
    hilbertMap::numberOfPointsForEachDimension = numberOfPointsForEachDimension;
}

const std::vector<std::vector<double>> &hilbertMap::getCurrentMap() const {
    return currentMap;
}

void hilbertMap::setCurrentMap(const std::vector<std::vector<double>> &currentMap) {
    hilbertMap::currentMap = currentMap;
}

