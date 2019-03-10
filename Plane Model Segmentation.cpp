
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

	// Fill in the cloud data ��ʼ��cloud--------------------------------------------
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
	//��ʼ�ָ�-------------------------------------------------------------------------------
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
///������ʾPointXYZ
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

	pcl::SACSegmentation<pcl::PointXYZ> seg;  //�����ָ����
	seg.setOptimizeCoefficients(true);        //���öԹ��Ƶ�ģ�Ͳ��������Ż�����
	seg.setModelType(pcl::SACMODEL_PLANE);    //���÷ָ�ģ�����
	seg.setMethodType(pcl::SAC_RANSAC);       //�������ĸ�����������Ʒ���
	seg.setMaxIterations(1000);                //��������������
	seg.setDistanceThreshold(0.01);            //�����ж��Ƿ�Ϊģ���ڵ�ľ�����ֵ
	seg.setInputCloud(cloud);
	seg.segment(*inliers, *coefficients);

	pcl::ExtractIndices<pcl::PointXYZ> extract; //����������ȡ����
	extract.setInputCloud(cloud);      //�����������
	extract.setIndices(inliers);                 //���÷ָ����ڵ�Ϊ��Ҫ��ȡ�ĵ㼯
	extract.setNegative(negative);                  //������ȡ�ڵ�������  ��false�ָ�ģ�����true ��ȡ�ָ�ģ����� ����ĵ�
	extract.filter(*cloud_projected);                    //��ȡ����洢��cloud_p
}

int main_fenge(int argc, char** argv)
//int main(int argc, char** argv)
{
	pcl::PCDReader reader;   //��ȡPCD
	pcl::PCDWriter writer;   //д��PCD
	pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;  //�����ָ����ʹ�����淨�߽���ƽ��ָ���ࣩ
	
	pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);//�����ָ�ʱ����Ҫ��ģ��ϵ������coefficients 
	pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices); //�洢�ڵ�ĵ��������϶���inliers   plane ƽ��
	pcl::ExtractIndices<pcl::PointXYZ> extract;  //����������ȡ����

	//DataSet
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_vg_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_NaNs_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	//��ȡ�ļ� 
	reader.read("D:/MaTong.pcd", *cloud);
	std::cout << "PointCloud before filtering has: " << cloud->points.size() << " data points." << std::endl; //*

	
	// �²���������Ҷ�Ӵ�СΪ0.01
	pcl::VoxelGrid<pcl::PointXYZ> vg;      //����դ���²�������
	vg.setInputCloud(cloud);               // ԭʼ����
	vg.setLeafSize(0.005f, 0.005f, 0.005f);   // ���ò������ش�С
	vg.filter(*cloud_vg_filtered);         //����
	std::cout << "PointCloud after filtering has: " << cloud_vg_filtered->points.size() << " data points." << std::endl;

	// Build a passthrough filter to remove spurious NaNs ����һ��ֱͨ�����������������NaNs
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloud_vg_filtered);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0, 1.5);
	pass.filter(*cloud_NaNs_filtered);//Ԥ������ȫ��

	// Estimate point normals
	ne.setSearchMethod(tree);
	ne.setInputCloud(cloud_NaNs_filtered);
	ne.setKSearch(50);
	ne.compute(*cloud_normals);

	// Create the segmentation object for the planar model and set all the parameters
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);  //����ģ������
	seg.setNormalDistanceWeight(0.1);
	seg.setMethodType(pcl::SAC_RANSAC);  //�����������һ���Է�������
	seg.setMaxIterations(100);        //��������������
	seg.setDistanceThreshold(0.05);  //�趨���뷧ֵ�����뷧ֵ�����˵㱻��Ϊ�Ǿ��ڵ��Ǳ����������������ʾ�㵽����ģ�͵ľ������ֵ
	seg.setInputCloud(cloud_NaNs_filtered);
	seg.setInputNormals(cloud_normals);
	// Obtain the plane inliers and coefficients
	seg.segment(*inliers_plane, *coefficients_plane);  //�����ָ�ʵ�֣��洢�ָ������㼸��inliers���洢ƽ��ģ�͵�ϵ��coefficients
	std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;
	// Extract the planar inliers from the input cloud
	extract.setInputCloud(cloud_NaNs_filtered);
	extract.setIndices(inliers_plane);
	extract.setNegative(false);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
	extract.filter(*cloud_plane);//�õ� ƽ��
	std::cerr << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points." << std::endl;

	// Remove the planar inliers, extract the rest
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_objects(new pcl::PointCloud<pcl::PointXYZ>());
	extract.setNegative(true);
	extract.filter(*cloud_objects);//�õ��˵ȴ��ָ��ǰ��ͼ
	//�ٽ���ŷʽ�ָ���Ŀ����Ƽ���--------------------------------------------------------------------------------
	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZ>);
	
	tree2->setInputCloud(cloud_objects); //���������������������ڴ洢ʵ�ʵĵ�����Ϣ

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(0.8); //���ý��������������뾶Ϊ2cm
	ec.setMinClusterSize(10);//����һ��������Ҫ�����ٵ���ĿΪ100
	ec.setMaxClusterSize(2500);//����һ��������Ҫ��������ĿΪ20000
	ec.setSearchMethod(tree2);//���õ��Ƶ���������
	ec.setInputCloud(cloud_objects);
	ec.extract(cluster_indices);//�ӵ�������ȡ���࣬������������������cluster_indices��

	/*Ϊ�˴ӵ������������зָ��ÿ�����࣬����������ʵ���������ÿ�δ���һ���µĵ������ݼ������ҽ����е�ǰ����ĵ�д�뵽�������ݼ��С�*/
	//�������ʵ�������cluster_indices��ֱ���ָ�����о���
	//int j = 0;
	//for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	//{
	//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
	//	//�����µĵ������ݼ�cloud_cluster�������е�ǰ����д�뵽�������ݼ���
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
	
	//��ʾ��ȡ�Ľ��
	//��ȡ�ļ� 	
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	viewer = simpleVis(cloud_objects);
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	return (0);
}