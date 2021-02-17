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
        pcl::visualization::PCLVisualizer::Ptr tmp_viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
        viewer = tmp_viewer;
        viewer->addCoordinateSystem (1.0);
        viewer->initCameraParameters ();

        viewer->setBackgroundColor (0, 0, 0);
    }
    void visualizeLoadedClouds(bool forLoop = false);
    void addCloudToViewer(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudToVisualize, const char *nameCloud,
                          const int &r,const int &g ,const int &b);
    void addCloudToViewer(pcl::PointCloud<pcl::PointXYZ> &cloudToVisualize, const char *nameCloud,
                          const int &r,const int &g ,const int &b);
    void removeCloudFromViewer(const char *nameCloud);

    Eigen::Matrix4f generalizedIcpRegistration(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudFirstScan,
                                    pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudSecondScan,
                                    pcl::PointCloud<pcl::PointXYZ>::Ptr &Final);

    Eigen::Matrix4f icpRegistration(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudFirstScan,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudSecondScan,
                         pcl::PointCloud<pcl::PointXYZ> &Final);
private:
    pcl::visualization::PCLVisualizer::Ptr viewer;

};


#endif //SIMULATION_BLUEROV_SCANREGISTRATIONCLASS_H
