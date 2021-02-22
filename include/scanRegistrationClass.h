//
// Created by tim on 16.02.21.
//
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/projection_matrix.h>
#ifndef SIMULATION_BLUEROV_SCANREGISTRATIONCLASS_H
#define SIMULATION_BLUEROV_SCANREGISTRATIONCLASS_H


class scanRegistrationClass {
public:
    scanRegistrationClass()
    {

    }
    Eigen::Matrix4f generalizedIcpRegistrationSimple(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudFirstScan,
                                                     const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudSecondScan,
                                               double &fitnessScore);

    Eigen::Matrix4f generalizedIcpRegistration(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudFirstScan,
                                               const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudSecondScan,
                                    pcl::PointCloud<pcl::PointXYZ>::Ptr &Final,
                                    double &fitnessScore);



    Eigen::Matrix4f icpRegistration(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudFirstScan,
                                    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudSecondScan,
                         pcl::PointCloud<pcl::PointXYZ> &Final);
private:


};


#endif //SIMULATION_BLUEROV_SCANREGISTRATIONCLASS_H
