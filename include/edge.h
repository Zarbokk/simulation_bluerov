//
// Created by tim on 23.02.21.
//

#ifndef SIMULATION_BLUEROV_EDGE_H
#define SIMULATION_BLUEROV_EDGE_H
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

class edge {
public:
    edge(int fromVertex, int toVertex, const std::vector<float> &informationMeasurement, const std::vector<float> &measurementDifference, int dimension) {
        edge::fromVertex = fromVertex;
        edge::toVertex = toVertex;
        edge::informationMeasurement = informationMeasurement;
        edge::measurementDifference = measurementDifference;
    }
    edge(int fromVertex, int toVertex, const std::vector<float> &informationMeasurement, const std::vector<float> &measurementDifference, int dimension,pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloud) {
        edge::fromVertex = fromVertex;
        edge::toVertex = toVertex;
        edge::informationMeasurement = informationMeasurement;
        edge::measurementDifference = measurementDifference;
        for (int i = 0; i < dimension; i++) {
            edge::informationMeasurement[i] = informationMeasurement[i];
            edge::measurementDifference[i] = measurementDifference[i];
        }
        edge::pointCloud=pointCloud;
    }

    int getFromVertex() const;

    void setFromVertex(int fromVertex);

    int getToVertex() const;

    void setToVertex(int toVertex);

    std::vector<float> getInformationMeasurement() const;

    void setInformationMeasurement(std::vector<float> &informationMeasurement);

    std::vector<float> getMeasurementDifference() const;

    void setMeasurementDifference(std::vector<float> &measurementDifference);

    const pcl::PointCloud<pcl::PointXYZ>::Ptr &getPointCloud() const;

    void setPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloud);


private:
    int fromVertex;
    int toVertex;
    std::vector<float> informationMeasurement;//estimated accuracy
    std::vector<float> measurementDifference;//estimated difference between nodes "state-diff"
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud;//measurement

};


#endif //SIMULATION_BLUEROV_EDGE_H
