/*
	圆柱模型分割
*/
#include "stdafx.h"
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h> // 用于最后的显示提取的结果
#include <pcl/sample_consensus/method_types.h> //随机参数估计方法头文件
#include <pcl/sample_consensus/model_types.h>  //模型定义头文件
#include <pcl/segmentation/sac_segmentation.h> //基于采样一致性分割的类的头文件
#include <boost/thread/thread.hpp>

///-------------------------------------------------------

typedef pcl::PointXYZ PointT;

int main_CMS(int argc, char** argv) //Cylinder model segmentation
//int main(int argc, char** argv) 
{
	//1.定义变量
	// All the objects needed
	pcl::PCDReader reader;      //设置PCD文件读取对象
	pcl::PCDWriter writer;     //设置PCD文件写出对象
	pcl::PassThrough<PointT> pass;  //设置滤波器对象
	pcl::NormalEstimation<PointT, pcl::Normal> ne;  //设置法线估计对象
	pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;  //设置分割对象
	pcl::ExtractIndices<PointT> extract;   //设置点提取对象
	pcl::ExtractIndices<pcl::Normal> extract_normals;     //设置点提取对象
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());

	//2.设置数据集
	// Datasets
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<PointT>::Ptr cloud_filtered2(new pcl::PointCloud<PointT>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2(new pcl::PointCloud<pcl::Normal>);
	pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients), coefficients_cylinder(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices), inliers_cylinder(new pcl::PointIndices);
	pcl::PointCloud<PointT>::Ptr cloud_cylinder(new pcl::PointCloud<PointT>());

	//3.读取数据
	// Read in the cloud data 读取云数据，获取数据含有多少个数据点
	reader.read("D:/MaTong1.pcd", *cloud);
	std::cerr << "PointCloud has: " << cloud->points.size() << " data points." << std::endl;
	
	//4.进行滤波操作
	// Build a passthrough filter to remove spurious NaNs （建立一个直通过滤器以消除错误的 NaNs）
	// 直通滤波，将Z轴不在（0，1.5）范围的点过滤掉，将剩余的点存储到cloud_filtered对象中
	pass.setInputCloud(cloud);     // 设置输入点云
	pass.setFilterFieldName("z");  // 设置过滤时所需要点云类型的z字段
	pass.setFilterLimits(0, 1.5);  // 设置在过滤字段上的范围
	pass.filter(*cloud_filtered);  // 执行滤波，保存过滤结果在cloud_filtered
	std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size() << " data points." << std::endl;  //打印点云数据个数

	//5.法线估计
	// Estimate point normals（估计点法向）
	// 过滤后的点云进行法线估计，为后续进行基于法线的分割准备数据
	ne.setSearchMethod(tree);
	ne.setInputCloud(cloud_filtered);
	ne.setKSearch(50);
	ne.compute(*cloud_normals);

	//6.1平面分割
	// Create the segmentation object for the planar model and set all the parameters
	// 为平面模型创建分割对象并设置所有参数，***详细介绍在下面7.1***
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
	seg.setNormalDistanceWeight(0.1);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(100);
	seg.setDistanceThreshold(0.03);
	seg.setInputCloud(cloud_filtered);
	seg.setInputNormals(cloud_normals);
	
	//6.2获取内点与系数
	// Obtain the plane inliers and coefficients
	// 获得平面内点和系数
	seg.segment(*inliers_plane, *coefficients_plane);
	std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

	//6.3点集抽取
	// Extract the planar inliers from the input cloud（从点云中抽取分割的处在平面上的点集）
	extract.setInputCloud(cloud_filtered);
	extract.setIndices(inliers_plane);
	extract.setNegative(false);

	//6.4存储分割点集
	// Write the planar inliers to disk（存储分割得到的平面上的点到点云文件）
	pcl::PointCloud<PointT>::Ptr cloud_plane(new pcl::PointCloud<PointT>());
	extract.filter(*cloud_plane);
	std::cerr << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points." << std::endl;
	writer.write("table_scene_mug_stereo_textured_plane.pcd", *cloud_plane, false);

	//6.5 
	// Remove the planar inliers, extract the rest（移除平面内点，提取其余部分）
	extract.setNegative(true);
	extract.filter(*cloud_filtered2);
	extract_normals.setNegative(true);
	extract_normals.setInputCloud(cloud_normals);
	extract_normals.setIndices(inliers_plane);
	extract_normals.filter(*cloud_normals2);

	//7.1圆柱分割
	// Create the segmentation object for cylinder segmentation and set all the parameters
	// 为圆柱体分割创建分割对象并设置所有参数
	seg.setOptimizeCoefficients(true);   //设置对估计模型优化
	seg.setModelType(pcl::SACMODEL_CYLINDER);  //设置分割模型为圆柱形
	seg.setMethodType(pcl::SAC_RANSAC);  //使用RANSAC鲁棒估计器来获得柱面系数
	seg.setNormalDistanceWeight(0.1);  //表面法线影响设置为0.1的权重
	seg.setMaxIterations(10000);       //设置迭代的最大次数10000
	seg.setDistanceThreshold(0.05);    //从每个内点到模型不大于5cm施加距离阈值
	seg.setRadiusLimits(0.05, 0.1);       //圆柱模型的半径限制为小于10cm。 已修改
	seg.setInputCloud(cloud_filtered2);
	seg.setInputNormals(cloud_normals2);

	//7.2获取内点与系数
	// Obtain the cylinder inliers and coefficients(获取圆柱内点和系数)
	seg.segment(*inliers_cylinder, *coefficients_cylinder);
	std::cerr << "Cylinder coefficients(圆柱系数): " << *coefficients_cylinder << std::endl;

	//7.3存储分割点集
	// Write the cylinder inliers to disk（存储分割得到的圆柱上的点到点云文件）
	extract.setInputCloud(cloud_filtered2);
	extract.setIndices(inliers_cylinder);
	extract.setNegative(false);
	
	extract.filter(*cloud_cylinder);
	if (cloud_cylinder->points.empty())
		std::cerr << "Can't find the cylindrical component（找不到圆柱形部件）." << std::endl;
	else
	{
		std::cerr << "PointCloud representing the cylindrical component（表示圆柱形组件的点云）: " << cloud_cylinder->points.size() << " data points." << std::endl;
		//writer.write("table_scene_mug_stereo_textured_cylinder.pcd", *cloud_cylinder, false); //将点云文件写出
	}

	//显示提取的结果
	//读取文件 	
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->addPointCloud<pcl::PointXYZ>(cloud_cylinder, "sample cloud");
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	return 0;
}
