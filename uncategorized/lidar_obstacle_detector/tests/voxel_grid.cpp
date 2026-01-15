#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/pcl_base.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

void simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
    // Create the visualizer object
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    // Add the point cloud to the viewer
    viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();

    // Spin while the visualizer is not closed
    viewer->spin();
}

int main() {
    pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
    pcl::PCLPointCloud2::Ptr filtered_cloud(new pcl::PCLPointCloud2());

    // Fill the cloud data
    pcl::PCDReader reader;
    reader.read("../table_scene_lms400.pcd", *cloud);

    std::cout<<"Point cloud before filetering, width: "<<cloud->width
    <<" height: "<<cloud->height << " data points (" << pcl::getFieldsList (*cloud) << ")." << std::endl;

    // Create filtering olbect
    pcl::VoxelGrid<pcl::PCLPointCloud2> voxel_grid;
    voxel_grid.setInputCloud(cloud);
    voxel_grid.setLeafSize(0.01f, 0.01f, 0.01f);
    voxel_grid.filter(*filtered_cloud);

    std::cout<<"Point cloud after filetering, width: "<<filtered_cloud->width
    <<" height: "<<filtered_cloud->height << " data points (" << pcl::getFieldsList (*filtered_cloud) << ")." << std::endl;

    // Write data
    pcl::PCDWriter writer;
    writer.write("table_scene_lms400_down_sampled.pcd", *filtered_cloud, 
    Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);

    pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");

    pcl::PointCloud<pcl::PointXYZ>::Ptr view_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2 (*filtered_cloud, *view_cloud); //* convert from PCLPointCloud2 to pcl::PointCloud<T>

    simpleVis(view_cloud);
    return 0;
}