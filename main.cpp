#include <iostream>
#include <vector>
#include <pcl/filters/filter_indices.h> // for pcl::removeNaNFromPointCloud
#include <pcl/filters/passthrough.h>
#include <ros/ros.h>

//Standard ROS Headers
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>

//PCL basic headers
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

//PCL filters
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>

//PCL Segmentation
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/region_growing.h>

//PCL other headers
#include <pcl/surface/convex_hull.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/centroid.h>
#include "pcl_ros/transforms.h"

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

float nx,ny,nz;                          //Normal vector

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
    sor.setLeafSize(0.1f, 0.1f, 0.1f);
    sor.filter(*cloud_downsampled);
    pcl::PCDWriter writer1;
    writer1.write("test_pcd_downsampled.pcd", *cloud_downsampled, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), false);

	// std::vector<int> mapping;
    // pcl::removeNaNFromPointCloud(*cloud_downsampled, *cloud_downsampled, mapping);

    // // Step 2: Statistical Outlier Removal
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::fromPCLPointCloud2(*cloud_downsampled, *cloud_filtered);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_inliers(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor2;
    // sor2.setInputCloud(cloud_filtered);
    // sor2.setMeanK(50);
    // sor2.setStddevMulThresh(1.0);
    // sor2.filter(*cloud_inliers);
    // pcl::PCDWriter writer2;
    // writer2.write<pcl::PointXYZ>("test_pcd_downsampled_inliers.pcd", *cloud_inliers, false);

    // Step 3: Region Growing Segmentation
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_region_growing(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("test_pcd_downsampled.pcd", *cloud_region_growing) == -1) //test_pcd_downsampled_inliers.pcd
    {
        std::cout << "Cloud reading failed." << std::endl;
        return (-1);
    }


	//crop 
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud_region_growing);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(/* min_x */ -250, /* max_x */ 250);
    pass.filter(*cloud_region_growing); // Crop along x-axis
    pass.setInputCloud(cloud_region_growing);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(/* min_y */ -250, /* max_y */ 250);
    pass.filter(*cloud_region_growing); // Crop along y-axis
    pass.setInputCloud(cloud_region_growing);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(/* min_z */ -10000, /* max_z */ 10000); // Adjust as needed
    pass.filter(*cloud_region_growing); // Crop along z-axis

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

    if(clusters.size() > 0)
    {
        int currentClusterNum = 1;
        geometry_msgs::Pose single_object_pose;                //Current object pose
        std::vector<geometry_msgs::Pose> object_poses;         //Vector of object poses

        geometry_msgs::Vector3 single_object_normal;           //Current object normal
        std::vector<geometry_msgs::Vector3> object_normals;    //Vector of object normals

        int max_z_coordinate_object_index = 0;
        float max_z_coordinate = 0;
	
	for (std::vector<pcl::PointIndices>::const_iterator i = clusters.begin(); i != clusters.end(); ++i)
        {
          std::cout << "Processing cluster n." << currentClusterNum << "..." << std::endl;
          PointCloudT::Ptr cluster(new PointCloudT);
          for (std::vector<int>::const_iterator point = i->indices.begin(); point != i->indices.end(); point++)
              cluster->points.push_back(cloud_region_growing->points[*point]); //cloud_filtered
          cluster->width = cluster->points.size();
          cluster->height = 1;
          cluster->is_dense = true;

          PointCloudT::Ptr cluster_filtered(new PointCloudT);
          PointCloudT::Ptr cluster_plane_raw(new PointCloudT);


          //Cluster plane segmentation
          //-------------------------------------
          pcl::SACSegmentation<PointT> seg;
          pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
          pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);

          seg.setOptimizeCoefficients(true);
          seg.setModelType(pcl::SACMODEL_PLANE);
          seg.setMethodType(pcl::SAC_RANSAC);
          seg.setMaxIterations(100); //plane_max_iter
          seg.setDistanceThreshold(0.02); // plane_dist_thresh
          
		  seg.setInputCloud (cluster);
          seg.segment(*inliers, *coefficients);

          //Segment the largest planar komponent from cluster
          seg.setInputCloud (cluster);
          seg.segment(*inliers, *coefficients);

          //Extract the planar inliers from the input cloud
          pcl::ExtractIndices<PointT> extract;
          extract.setInputCloud(cluster);
          extract.setIndices(inliers);
          extract.setNegative(false);

          //Get the points associated with the planar surface
          extract.filter(*cluster_plane_raw);
          std::cout << "Planar cloud size: " << cluster_plane_raw->points.size() << std::endl;

          //Statistical outlier removal
          //--------------------------------------
          PointCloudT::Ptr cluster_plane(new PointCloudT);
          pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
          sor.setInputCloud (cluster_plane_raw);
          sor.setMeanK (50);
          sor.setStddevMulThresh (1.0);
          sor.filter (*cluster_plane);

          PointCloudT::Ptr convexHull (new PointCloudT);
          pcl::ConvexHull<pcl::PointXYZ> hull;
          std::vector<pcl::Vertices> polygons_alpha;
          hull.setComputeAreaVolume(true);
          hull.setInputCloud(cluster_plane);
          hull.reconstruct(*convexHull,polygons_alpha);

          std::vector <int> vertex_index;
          float curvature;

          for(int k = 0; k < convexHull->width; k++)
             vertex_index.push_back(k);

          pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
          ne.computePointNormal(*convexHull, vertex_index, nx,ny,nz, curvature);

          Eigen::Vector4f object_centroid;
          pcl::compute3DCentroid(*cluster_plane, object_centroid);


          //Detect and save the index of highest "z" centroid coordinate
          if(object_centroid[2] > max_z_coordinate)
          {
             max_z_coordinate = object_centroid[2];
             max_z_coordinate_object_index = currentClusterNum - 1;
          }

          float rx, ry, rz;
          rx = -((M_PI/2)-acos(ny));
          ry = ((M_PI/2)-acos(nx));
          rz = acos(nz);

          tf::Quaternion object_orientation;
          object_orientation.setRPY(rx,ry,0);

          //Save current object pose
          single_object_pose.position.x = object_centroid[0];
          single_object_pose.position.y = object_centroid[1];
          single_object_pose.position.z = object_centroid[2];

          single_object_pose.orientation.x = object_orientation.getX();
          single_object_pose.orientation.y = object_orientation.getY();
          single_object_pose.orientation.z = object_orientation.getZ();
          single_object_pose.orientation.w = object_orientation.getW();

          object_poses.push_back(single_object_pose);

          //Save current object normals
          single_object_normal.x = nx;
          single_object_normal.y = ny;
          single_object_normal.z = nz;

          object_normals.push_back(single_object_normal);

		}
		}	

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
    pcl::visualization::CloudViewer viewer("Cluster viewer");
    viewer.showCloud(colored_cloud);
    while (!viewer.wasStopped())
    {
    }

    return 0;
}
