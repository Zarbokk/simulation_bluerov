//
// Created by tim on 11.05.21.
//

#ifndef SIMULATION_BLUEROV_HILBERTMAP_H
#define SIMULATION_BLUEROV_HILBERTMAP_H
#include "vector"

class hilbertMap {
    public:
        hilbertMap(int numberOfPointsForEachDimension, double discritisationSize) {
            this->discritisationSize = discritisationSize;
            this->numberOfPointsForEachDimension = numberOfPointsForEachDimension;
        }
        void createRandomMap();

    private:
        double discritisationSize;
public:
    double getDiscritisationSize() const;

    void setDiscritisationSize(double discritisationSize);

    int getNumberOfPointsForEachDimension() const;

    void setNumberOfPointsForEachDimension(int numberOfPointsForEachDimension);

    const std::vector<std::vector<double>> &getCurrentMap() const;

    void setCurrentMap(const std::vector<std::vector<double>> &currentMap);

private:
    int numberOfPointsForEachDimension;
        std::vector<std::vector<double>> currentMap;
};


#endif //SIMULATION_BLUEROV_HILBERTMAP_H
