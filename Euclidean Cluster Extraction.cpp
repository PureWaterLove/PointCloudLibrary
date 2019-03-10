/*
	ŷ����ô���ȡ
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
#include <pcl/visualization/pcl_visualizer.h> // ����������ʾ��ȡ�Ľ��
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

	// Create the filtering object: downsample the dataset using a leaf size of 1cm������ɸѡ����ʹ��1cm��Ҷ��С�����ݼ�������С������
	pcl::VoxelGrid<pcl::PointXYZ> vg;  //����դ���²�������
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	vg.setInputCloud(cloud);   // ԭʼ����
	vg.setLeafSize(0.01f, 0.01f, 0.01f);  // ���ò������ش�С
	vg.filter(*cloud_filtered);    //����
	std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size() << " data points." << std::endl; //*
	
	// Create the segmentation object for the planar model and set all the parameters
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PCDWriter writer;
	seg.setOptimizeCoefficients(true);   //���öԹ���ģ���Ż�
	seg.setModelType(pcl::SACMODEL_PLANE);//���÷ָ�ģ��Ϊƽ��
	seg.setMethodType(pcl::SAC_RANSAC); //ʹ��RANSAC³�������������ƽ��ϵ��
	seg.setMaxIterations(100);   //���õ�����������100
	seg.setDistanceThreshold(0.02); //��ÿ���ڵ㵽ģ�Ͳ�����2cmʩ�Ӿ�����ֵ

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

		// Extract the planar inliers from the input cloud��������������ȡƽ���ڵ㣩
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud(cloud_filtered);
		extract.setIndices(inliers);
		extract.setNegative(false);

		// Get the points associated with the planar surface
		extract.filter(*cloud_plane);
		std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points." << std::endl;

		// Remove the planar inliers, extract the rest�� ��ȥƽ����ڵ㣬��ȡʣ����ƣ�
		extract.setNegative(true);
		extract.filter(*cloud_f);
		*cloud_filtered = *cloud_f;
	}

	// Creating the KdTree object for the search method of the extraction��Ϊ��ȡ�㷨��������������һ��KdTree���󡣣�
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud_filtered);

	std::vector<pcl::PointIndices> cluster_indices; //����vector <int>�е�ʵ��������Ϣ��ÿ����⵽�ļ�Ⱥ�������������ڴ˴�
	//cluster_indices��һ��������
	//���а���ÿ����⵽�ļ�Ⱥ��һ��PointIndicesʵ����
	//���� cluster_indices [0]���������е�һ���ص�����������

	//
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(0.02); // 2cm //���ý��������������뾶Ϊ2cm
	ec.setMinClusterSize(100);   //����һ��������Ҫ�����ٵ���ĿΪ100
	ec.setMaxClusterSize(25000);  //����һ��������Ҫ��������ĿΪ20000
	ec.setSearchMethod(tree);   //���õ��Ƶ���������
	ec.setInputCloud(cloud_filtered);
	ec.extract(cluster_indices);  //�ӵ�������ȡ���࣬������������������cluster_indices��
	/*
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(0.8); //���ý��������������뾶Ϊ2cm
	ec.setMinClusterSize(10);//����һ��������Ҫ�����ٵ���ĿΪ100
	ec.setMaxClusterSize(2500);//����һ��������Ҫ��������ĿΪ20000
	ec.setSearchMethod(tree2);//���õ��Ƶ���������
	ec.setInputCloud(cloud_objects);
	ec.extract(cluster_indices);//�ӵ�������ȡ���࣬������������������cluster_indices��
	*/

	/*Ϊ�˴ӵ������������зָ��ÿ�����࣬����������ʵ���������ÿ�δ���һ���µĵ������ݼ������ҽ����е�ǰ����ĵ�д�뵽�������ݼ��С�*/
	//�������ʵ�������cluster_indices��ֱ���ָ�����о���
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

	//��ʾ��ȡ�Ľ��
	//��ȡ�ļ� 	Original picture
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