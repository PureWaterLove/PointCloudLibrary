
#include "stdafx.h"
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/pcl_base.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/morphological_filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/segmentation/extract_clusters.h>
using namespace std;



int demo_fengepingmian() {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	// Fill in the cloud data 初始化cloud--------------------------------------------
	cloud->width = 15;
	cloud->height = 1;
	cloud->points.resize(cloud->width * cloud->height);

	// Generate the data
	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].z = 1.0;
	}

	// Set a few outliers
	cloud->points[0].z = 2.0;
	cloud->points[3].z = -2.0;
	cloud->points[6].z = 4.0;

	std::cerr << "Point cloud data: " << cloud->points.size() << " points" << std::endl;
	for (size_t i = 0; i < cloud->points.size(); ++i)
		std::cerr << "    " << cloud->points[i].x << " "
		<< cloud->points[i].y << " "
		<< cloud->points[i].z << std::endl;
	//开始分割-------------------------------------------------------------------------------
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.01);

	seg.setInputCloud(cloud);
	seg.segment(*inliers, *coefficients);

	if (inliers->indices.size() == 0)
	{
		PCL_ERROR("Could not estimate a planar model for the given dataset.");
		return (-1);
	}

	std::cerr << "Model coefficients: " << coefficients->values[0] << " "
		<< coefficients->values[1] << " "
		<< coefficients->values[2] << " "
		<< coefficients->values[3] << std::endl;

	std::cerr << "Model inliers: " << inliers->indices.size() << std::endl;
	for (size_t i = 0; i < inliers->indices.size(); ++i)
		std::cerr << inliers->indices[i] << "    " << cloud->points[inliers->indices[i]].x << " "
		<< cloud->points[inliers->indices[i]].y << " "
		<< cloud->points[inliers->indices[i]].z << std::endl;
	system("pause");
}
///用于显示PointXYZ
boost::shared_ptr<pcl::visualization::PCLVisualizer>
simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ>(cloud, "project_inliners cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	//viewer->addCoordinateSystem (1.0, "global");
	viewer->initCameraParameters();
	return (viewer);
}
///-------------------------------------------------------
int demo_tiqupingmian(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected,bool negative) {
	
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

	pcl::SACSegmentation<pcl::PointXYZ> seg;  //创建分割对象
	seg.setOptimizeCoefficients(true);        //设置对估计的模型参数进行优化处理
	seg.setModelType(pcl::SACMODEL_PLANE);    //设置分割模型类别
	seg.setMethodType(pcl::SAC_RANSAC);       //设置用哪个随机参数估计方法
	seg.setMaxIterations(1000);                //设置最大迭代次数
	seg.setDistanceThreshold(0.01);            //设置判断是否为模型内点的距离阈值
	seg.setInputCloud(cloud);
	seg.segment(*inliers, *coefficients);

	pcl::ExtractIndices<pcl::PointXYZ> extract; //创建点云提取对象
	extract.setInputCloud(cloud);      //设置输入点云
	extract.setIndices(inliers);                 //设置分割后的内点为需要提取的点集
	extract.setNegative(negative);                  //设置提取内点而非外点  ，false分割模型类别，true 提取分割模型类别 以外的点
	extract.filter(*cloud_projected);                    //提取输出存储到cloud_p
}

int main_fenge(int argc, char** argv)
//int main(int argc, char** argv)
{
	pcl::PCDReader reader;   //读取PCD
	pcl::PCDWriter writer;   //写出PCD
	pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;  //创建分割对象（使用曲面法线进行平面分割的类）
	
	pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);//创建分割时所需要的模型系数对象coefficients 
	pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices); //存储内点的点索引集合对象inliers   plane 平面
	pcl::ExtractIndices<pcl::PointXYZ> extract;  //创建点云提取对象

	//DataSet
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_vg_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_NaNs_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	//读取文件 
	reader.read("D:/MaTong.pcd", *cloud);
	std::cout << "PointCloud before filtering has: " << cloud->points.size() << " data points." << std::endl; //*

	
	// 下采样，体素叶子大小为0.01
	pcl::VoxelGrid<pcl::PointXYZ> vg;      //体素栅格下采样对象
	vg.setInputCloud(cloud);               // 原始点云
	vg.setLeafSize(0.005f, 0.005f, 0.005f);   // 设置采样体素大小
	vg.filter(*cloud_vg_filtered);         //保存
	std::cout << "PointCloud after filtering has: " << cloud_vg_filtered->points.size() << " data points." << std::endl;

	// Build a passthrough filter to remove spurious NaNs 建立一个直通过滤器以消除错误的NaNs
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloud_vg_filtered);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0, 1.5);
	pass.filter(*cloud_NaNs_filtered);//预处理后的全景

	// Estimate point normals
	ne.setSearchMethod(tree);
	ne.setInputCloud(cloud_NaNs_filtered);
	ne.setKSearch(50);
	ne.compute(*cloud_normals);

	// Create the segmentation object for the planar model and set all the parameters
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);  //设置模型类型
	seg.setNormalDistanceWeight(0.1);
	seg.setMethodType(pcl::SAC_RANSAC);  //设置随机采样一致性方法类型
	seg.setMaxIterations(100);        //设置最大迭代次数
	seg.setDistanceThreshold(0.05);  //设定距离阀值，距离阀值决定了点被认为是局内点是必须满足的条件，表示点到估计模型的距离最大值
	seg.setInputCloud(cloud_NaNs_filtered);
	seg.setInputNormals(cloud_normals);
	// Obtain the plane inliers and coefficients
	seg.segment(*inliers_plane, *coefficients_plane);  //引发分割实现，存储分割结果到点几何inliers及存储平面模型的系数coefficients
	std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;
	// Extract the planar inliers from the input cloud
	extract.setInputCloud(cloud_NaNs_filtered);
	extract.setIndices(inliers_plane);
	extract.setNegative(false);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
	extract.filter(*cloud_plane);//得到 平面
	std::cerr << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points." << std::endl;

	// Remove the planar inliers, extract the rest
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_objects(new pcl::PointCloud<pcl::PointXYZ>());
	extract.setNegative(true);
	extract.filter(*cloud_objects);//得到了等待分割的前景图
	//再进行欧式分割，获得目标点云集合--------------------------------------------------------------------------------
	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZ>);
	
	tree2->setInputCloud(cloud_objects); //创建点云索引向量，用于存储实际的点云信息

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(0.8); //设置近邻搜索的搜索半径为2cm
	ec.setMinClusterSize(10);//设置一个聚类需要的最少点数目为100
	ec.setMaxClusterSize(2500);//设置一个聚类需要的最大点数目为20000
	ec.setSearchMethod(tree2);//设置点云的搜索机制
	ec.setInputCloud(cloud_objects);
	ec.extract(cluster_indices);//从点云中提取聚类，并将点云索引保存在cluster_indices中

	/*为了从点云索引向量中分割出每个聚类，必须迭代访问点云索引，每次创建一个新的点云数据集，并且将所有当前聚类的点写入到点云数据集中。*/
	//迭代访问点云索引cluster_indices，直到分割出所有聚类
	//int j = 0;
	//for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	//{
	//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
	//	//创建新的点云数据集cloud_cluster，将所有当前聚类写入到点云数据集中
	//	for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
	//	{
	//		cloud_cluster->points.push_back(cloud_objects->points[*pit]); //*
	//	}
	//	cloud_cluster->width = cloud_cluster->points.size();
	//	cloud_cluster->height = 1;
	//	cloud_cluster->is_dense = true;
	//	std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << std::endl;
	//	std::stringstream ss;
	//	ss << "D:/cloud_cluster_jiaoshui" << j << ".pcd";
	//	writer.write<pcl::PointXYZ>(ss.str(), *cloud_cluster, false); //*
	//	j++;
	//}
	
	//显示提取的结果
	//读取文件 	
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	viewer = simpleVis(cloud_objects);
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	return (0);
}