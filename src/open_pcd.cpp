

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>



void downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_input, pcl::PointCloud<pcl::PointXYZ>::Ptr &output) {
    //sor.setSaveLeafLayout(true);
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_input);
    sor.setLeafSize(0.10f, 0.10f, 0.10f);
    sor.filter(*output);
    //display or do something else with output
}

int
main (int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudinput (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("/home/tim/catkin_ws/devel/lib/simulation_bluerov/before_voxel.pcd",*cloudinput);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudoutput (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("/home/tim/catkin_ws/devel/lib/simulation_bluerov/after_voxel.pcd",*cloudoutput);

    printf("current size Map before voxel: %d \n",(*cloudinput).size());
    printf("current size Map after voxel: %d \n",(*cloudoutput).size());


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudvoxel_output (new pcl::PointCloud<pcl::PointXYZ>);


    downsample(cloudinput,cloudvoxel_output);

    cout << "test results:" << (*cloudinput).size() <<  ":" << (*cloudvoxel_output).size() <<endl;

//    current size Map: 18628
//    current size Map after voxel: 7937

    pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
    viewer.setBackgroundColor(0,0,0);
    //viewer.addPointCloud (cloudinput,"body");// note that before it was showCloud
    viewer.addPointCloud (cloudvoxel_output,"head");// note that before it was showCloud
    //viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "body");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "head");
    viewer.spin();

    return (0);
}
