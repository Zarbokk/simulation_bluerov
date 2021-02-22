//
// Created by tim on 16.02.21.
//

#include "scanRegistrationClass.h"

Eigen::Matrix4f scanRegistrationClass::generalizedIcpRegistration(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudFirstScan,
                                                                  const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudSecondScan,
                                                       pcl::PointCloud<pcl::PointXYZ>::Ptr &Final,
                                                       double &fitnessScore){

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
    fitnessScore = gicp.getFitnessScore();
    return gicp.getFinalTransformation();
}

Eigen::Matrix4f scanRegistrationClass::generalizedIcpRegistrationSimple(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudFirstScan,
                                                                  const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudSecondScan,
                                                                  double &fitnessScore){
    pcl::PointCloud<pcl::PointXYZ>::Ptr Final(new pcl::PointCloud<pcl::PointXYZ>);
    return scanRegistrationClass::generalizedIcpRegistration(cloudFirstScan, cloudSecondScan, Final,
                                           fitnessScore);
}




Eigen::Matrix4f scanRegistrationClass::icpRegistration(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudFirstScan,
                                                       const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudSecondScan,
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

