/*
	欧几里得簇提取
*/
#include "stdafx.h"
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h> // 用于最后的显示提取的结果
#include <boost/thread/thread.hpp>

int main_Euclidean(int argc, char** argv)
//int main(int argc, char** argv)
{
	// Read in the cloud data
	pcl::PCDReader reader;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>), cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
	reader.read("D:/MaTong.pcd", *cloud);
	std::cout << "PointCloud before filtering has: " << cloud->points.size() << " data points." << std::endl; //*

	// Create the filtering object: downsample the dataset using a leaf size of 1cm（创建筛选对象：使用1cm的叶大小对数据集进行缩小采样）
	pcl::VoxelGrid<pcl::PointXYZ> vg;  //体素栅格下采样对象
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	vg.setInputCloud(cloud);   // 原始点云
	vg.setLeafSize(0.01f, 0.01f, 0.01f);  // 设置采样体素大小
	vg.filter(*cloud_filtered);    //保存
	std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size() << " data points." << std::endl; //*
	
	// Create the segmentation object for the planar model and set all the parameters
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PCDWriter writer;
	seg.setOptimizeCoefficients(true);   //设置对估计模型优化
	seg.setModelType(pcl::SACMODEL_PLANE);//设置分割模型为平面
	seg.setMethodType(pcl::SAC_RANSAC); //使用RANSAC鲁棒估计器来获得平面系数
	seg.setMaxIterations(100);   //设置迭代的最大次数100
	seg.setDistanceThreshold(0.02); //从每个内点到模型不大于2cm施加距离阈值

	int i = 0, nr_points = (int)cloud_filtered->points.size();
	while (cloud_filtered->points.size() > 0.3 * nr_points)
	{
		// Segment the largest planar component from the remaining cloud
		seg.setInputCloud(cloud_filtered);
		seg.segment(*inliers, *coefficients);
		if (inliers->indices.size() == 0)
		{
			std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
			break;
		}

		// Extract the planar inliers from the input cloud（从输入云中提取平面内点）
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud(cloud_filtered);
		extract.setIndices(inliers);
		extract.setNegative(false);

		// Get the points associated with the planar surface
		extract.filter(*cloud_plane);
		std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points." << std::endl;

		// Remove the planar inliers, extract the rest（ 移去平面局内点，提取剩余点云）
		extract.setNegative(true);
		extract.filter(*cloud_f);
		*cloud_filtered = *cloud_f;
	}

	// Creating the KdTree object for the search method of the extraction（为提取算法的搜索方法创建一个KdTree对象。）
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud_filtered);

	std::vector<pcl::PointIndices> cluster_indices; //包含vector <int>中的实际索引信息。每个检测到的集群的索引都保存在此处
	//cluster_indices是一个向量，
	//其中包含每个检测到的集群的一个PointIndices实例。
	//所以 cluster_indices [0]包含点云中第一个簇的所有索引。

	//
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(0.02); // 2cm //设置近邻搜索的搜索半径为2cm
	ec.setMinClusterSize(100);   //设置一个聚类需要的最少点数目为100
	ec.setMaxClusterSize(25000);  //设置一个聚类需要的最大点数目为20000
	ec.setSearchMethod(tree);   //设置点云的搜索机制
	ec.setInputCloud(cloud_filtered);
	ec.extract(cluster_indices);  //从点云中提取聚类，并将点云索引保存在cluster_indices中
	/*
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(0.8); //设置近邻搜索的搜索半径为2cm
	ec.setMinClusterSize(10);//设置一个聚类需要的最少点数目为100
	ec.setMaxClusterSize(2500);//设置一个聚类需要的最大点数目为20000
	ec.setSearchMethod(tree2);//设置点云的搜索机制
	ec.setInputCloud(cloud_objects);
	ec.extract(cluster_indices);//从点云中提取聚类，并将点云索引保存在cluster_indices中
	*/

	/*为了从点云索引向量中分割出每个聚类，必须迭代访问点云索引，每次创建一个新的点云数据集，并且将所有当前聚类的点写入到点云数据集中。*/
	//迭代访问点云索引cluster_indices，直到分割出所有聚类
	//int j = 0;
	//for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	//{
	//	
	//	for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
	//		cloud_cluster->points.push_back(cloud_filtered->points[*pit]); //*
	//	cloud_cluster->width = cloud_cluster->points.size();
	//	cloud_cluster->height = 1;
	//	cloud_cluster->is_dense = true;

	//	std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << std::endl;
	//	std::stringstream ss;
	//	ss << "cloud_cluster_" << j << ".pcd";
	//	writer.write<pcl::PointXYZ>(ss.str(), *cloud_cluster, false); //*
	//	j++;
	//}

	//显示提取的结果
	//读取文件 	Original picture
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_orig(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer_orig->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud - Original picture");
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->addPointCloud<pcl::PointXYZ>(cloud_cluster, "sample cloud");
	while (!viewer_orig->wasStopped())
	{
		viewer_orig->spinOnce(100);
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
		//boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	system("pause");
	return (0);
}