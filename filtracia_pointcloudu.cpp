#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter_indices.h> // for pcl::removeNaNFromPointCloud
#include <pcl/segmentation/region_growing.h>

int main()
{
    // Read the initial point cloud
    pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
    pcl::PCDReader reader;
    reader.read("test_pcd.pcd", *cloud);

    // Step 1: Voxel Grid Downsampling
    pcl::PCLPointCloud2::Ptr cloud_downsampled(new pcl::PCLPointCloud2());
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.5f, 0.5f, 0.5f);
    sor.filter(*cloud_downsampled);
    pcl::PCDWriter writer1;
    writer1.write("test_pcd_downsampled.pcd", *cloud_downsampled, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), false);

    // Step 2: Statistical Outlier Removal
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*cloud_downsampled, *cloud_filtered);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_inliers(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor2;
    sor2.setInputCloud(cloud_filtered);
    sor2.setMeanK(50);
    sor2.setStddevMulThresh(1.0);
    sor2.filter(*cloud_inliers);
    pcl::PCDWriter writer2;
    writer2.write<pcl::PointXYZ>("test_pcd_downsampled_inliers.pcd", *cloud_inliers, false);

    // Step 3: Region Growing Segmentation
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_region_growing(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("test_pcd_downsampled_inliers.pcd", *cloud_region_growing) == -1)
    {
        std::cout << "Cloud reading failed." << std::endl;
        return (-1);
    }
	//normal estimation
    pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(cloud_region_growing);
    normal_estimator.setKSearch(50);
    normal_estimator.compute(*normals);

    pcl::IndicesPtr indices(new std::vector<int>);
    pcl::removeNaNFromPointCloud(*cloud_region_growing, *indices);
	
	//region grow segmentation
    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    reg.setMinClusterSize(50);
    reg.setMaxClusterSize(1000000);
    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(30);
    reg.setInputCloud(cloud_region_growing);
    reg.setIndices(indices);
    reg.setInputNormals(normals);
    reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
    reg.setCurvatureThreshold(1.0);

    std::vector<pcl::PointIndices> clusters;
    reg.extract(clusters);

    std::cout << "Number of clusters is equal to " << clusters.size() << std::endl;
    std::cout << "First cluster has " << clusters[0].indices.size() << " points." << std::endl;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
    pcl::visualization::CloudViewer viewer("Cluster viewer");
    viewer.showCloud(colored_cloud);
    while (!viewer.wasStopped())
    {
    }

    return 0;
}
