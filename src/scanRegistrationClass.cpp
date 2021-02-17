//
// Created by tim on 16.02.21.
//

#include "scanRegistrationClass.h"

Eigen::Matrix4f scanRegistrationClass::generalizedIcpRegistration(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudFirstScan,
                                                       pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudSecondScan,
                                                       pcl::PointCloud<pcl::PointXYZ>::Ptr &Final){

    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ> gicp;

    gicp.setInputSource(cloudFirstScan);
    gicp.setInputTarget(cloudSecondScan);
    //gicp.setSourceCovariances(source_covariances);
    //gicp.setTargetCovariances(target_covariances);
    gicp.setMaxCorrespondenceDistance(5);
    //gicp.setMaximumIterations(1);
    Eigen::Matrix4f guess;
    guess << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;


    gicp.align(*Final,guess);
    std::cout << "has converged:" << gicp.hasConverged() << " score: " <<
              gicp.getFitnessScore() << std::endl;
    //std::cout << gicp.getFinalTransformation() << std::endl;

    return gicp.getFinalTransformation();
}

Eigen::Matrix4f scanRegistrationClass::icpRegistration(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudFirstScan,
                                            pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudSecondScan,
                                            pcl::PointCloud<pcl::PointXYZ> &Final){
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloudFirstScan);
    icp.setInputTarget(cloudSecondScan);
    icp.align(Final);
    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
              icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;
    return icp.getFinalTransformation();
}

void scanRegistrationClass::visualizeLoadedClouds(bool forLoop){
    if (!forLoop){
        while (!scanRegistrationClass::viewer->wasStopped ())
        {
            scanRegistrationClass::viewer->spinOnce (100);
        }
    }else{
            scanRegistrationClass::viewer->spin();
    }
}

void scanRegistrationClass::addCloudToViewer(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudToVisualize, const char *nameCloud,
                                             const int &r,const int &g ,const int &b){

    scanRegistrationClass::viewer->addPointCloud<pcl::PointXYZ> (cloudToVisualize, nameCloud);
    scanRegistrationClass::viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, nameCloud);
    scanRegistrationClass::viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, r,g,b, nameCloud);
}

void scanRegistrationClass::addCloudToViewer(pcl::PointCloud<pcl::PointXYZ> &cloudToVisualize, const char *nameCloud,
                      const int &r,const int &g ,const int &b){
    scanRegistrationClass::viewer->addPointCloud<pcl::PointXYZ> (cloudToVisualize.makeShared(), nameCloud);

    scanRegistrationClass::viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, nameCloud);
    scanRegistrationClass::viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, r,g,b, nameCloud);

}
void scanRegistrationClass::removeCloudFromViewer(const char *nameCloud){

    scanRegistrationClass::viewer->removePointCloud(nameCloud);
}

