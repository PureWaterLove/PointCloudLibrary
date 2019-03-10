/*
	Բ��ģ�ͷָ�
*/
#include "stdafx.h"
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h> // ����������ʾ��ȡ�Ľ��
#include <pcl/sample_consensus/method_types.h> //����������Ʒ���ͷ�ļ�
#include <pcl/sample_consensus/model_types.h>  //ģ�Ͷ���ͷ�ļ�
#include <pcl/segmentation/sac_segmentation.h> //���ڲ���һ���Էָ�����ͷ�ļ�
#include <boost/thread/thread.hpp>

///-------------------------------------------------------

typedef pcl::PointXYZ PointT;

int main_CMS(int argc, char** argv) //Cylinder model segmentation
//int main(int argc, char** argv) 
{
	//1.�������
	// All the objects needed
	pcl::PCDReader reader;      //����PCD�ļ���ȡ����
	pcl::PCDWriter writer;     //����PCD�ļ�д������
	pcl::PassThrough<PointT> pass;  //�����˲�������
	pcl::NormalEstimation<PointT, pcl::Normal> ne;  //���÷��߹��ƶ���
	pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;  //���÷ָ����
	pcl::ExtractIndices<PointT> extract;   //���õ���ȡ����
	pcl::ExtractIndices<pcl::Normal> extract_normals;     //���õ���ȡ����
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());

	//2.�������ݼ�
	// Datasets
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<PointT>::Ptr cloud_filtered2(new pcl::PointCloud<PointT>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2(new pcl::PointCloud<pcl::Normal>);
	pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients), coefficients_cylinder(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices), inliers_cylinder(new pcl::PointIndices);
	pcl::PointCloud<PointT>::Ptr cloud_cylinder(new pcl::PointCloud<PointT>());

	//3.��ȡ����
	// Read in the cloud data ��ȡ�����ݣ���ȡ���ݺ��ж��ٸ����ݵ�
	reader.read("D:/MaTong1.pcd", *cloud);
	std::cerr << "PointCloud has: " << cloud->points.size() << " data points." << std::endl;
	
	//4.�����˲�����
	// Build a passthrough filter to remove spurious NaNs ������һ��ֱͨ����������������� NaNs��
	// ֱͨ�˲�����Z�᲻�ڣ�0��1.5����Χ�ĵ���˵�����ʣ��ĵ�洢��cloud_filtered������
	pass.setInputCloud(cloud);     // �����������
	pass.setFilterFieldName("z");  // ���ù���ʱ����Ҫ�������͵�z�ֶ�
	pass.setFilterLimits(0, 1.5);  // �����ڹ����ֶ��ϵķ�Χ
	pass.filter(*cloud_filtered);  // ִ���˲���������˽����cloud_filtered
	std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size() << " data points." << std::endl;  //��ӡ�������ݸ���

	//5.���߹���
	// Estimate point normals�����Ƶ㷨��
	// ���˺�ĵ��ƽ��з��߹��ƣ�Ϊ�������л��ڷ��ߵķָ�׼������
	ne.setSearchMethod(tree);
	ne.setInputCloud(cloud_filtered);
	ne.setKSearch(50);
	ne.compute(*cloud_normals);

	//6.1ƽ��ָ�
	// Create the segmentation object for the planar model and set all the parameters
	// Ϊƽ��ģ�ʹ����ָ�����������в�����***��ϸ����������7.1***
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
	seg.setNormalDistanceWeight(0.1);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(100);
	seg.setDistanceThreshold(0.03);
	seg.setInputCloud(cloud_filtered);
	seg.setInputNormals(cloud_normals);
	
	//6.2��ȡ�ڵ���ϵ��
	// Obtain the plane inliers and coefficients
	// ���ƽ���ڵ��ϵ��
	seg.segment(*inliers_plane, *coefficients_plane);
	std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

	//6.3�㼯��ȡ
	// Extract the planar inliers from the input cloud���ӵ����г�ȡ�ָ�Ĵ���ƽ���ϵĵ㼯��
	extract.setInputCloud(cloud_filtered);
	extract.setIndices(inliers_plane);
	extract.setNegative(false);

	//6.4�洢�ָ�㼯
	// Write the planar inliers to disk���洢�ָ�õ���ƽ���ϵĵ㵽�����ļ���
	pcl::PointCloud<PointT>::Ptr cloud_plane(new pcl::PointCloud<PointT>());
	extract.filter(*cloud_plane);
	std::cerr << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points." << std::endl;
	writer.write("table_scene_mug_stereo_textured_plane.pcd", *cloud_plane, false);

	//6.5 
	// Remove the planar inliers, extract the rest���Ƴ�ƽ���ڵ㣬��ȡ���ಿ�֣�
	extract.setNegative(true);
	extract.filter(*cloud_filtered2);
	extract_normals.setNegative(true);
	extract_normals.setInputCloud(cloud_normals);
	extract_normals.setIndices(inliers_plane);
	extract_normals.filter(*cloud_normals2);

	//7.1Բ���ָ�
	// Create the segmentation object for cylinder segmentation and set all the parameters
	// ΪԲ����ָ���ָ�����������в���
	seg.setOptimizeCoefficients(true);   //���öԹ���ģ���Ż�
	seg.setModelType(pcl::SACMODEL_CYLINDER);  //���÷ָ�ģ��ΪԲ����
	seg.setMethodType(pcl::SAC_RANSAC);  //ʹ��RANSAC³�����������������ϵ��
	seg.setNormalDistanceWeight(0.1);  //���淨��Ӱ������Ϊ0.1��Ȩ��
	seg.setMaxIterations(10000);       //���õ�����������10000
	seg.setDistanceThreshold(0.05);    //��ÿ���ڵ㵽ģ�Ͳ�����5cmʩ�Ӿ�����ֵ
	seg.setRadiusLimits(0.05, 0.1);       //Բ��ģ�͵İ뾶����ΪС��10cm�� ���޸�
	seg.setInputCloud(cloud_filtered2);
	seg.setInputNormals(cloud_normals2);

	//7.2��ȡ�ڵ���ϵ��
	// Obtain the cylinder inliers and coefficients(��ȡԲ���ڵ��ϵ��)
	seg.segment(*inliers_cylinder, *coefficients_cylinder);
	std::cerr << "Cylinder coefficients(Բ��ϵ��): " << *coefficients_cylinder << std::endl;

	//7.3�洢�ָ�㼯
	// Write the cylinder inliers to disk���洢�ָ�õ���Բ���ϵĵ㵽�����ļ���
	extract.setInputCloud(cloud_filtered2);
	extract.setIndices(inliers_cylinder);
	extract.setNegative(false);
	
	extract.filter(*cloud_cylinder);
	if (cloud_cylinder->points.empty())
		std::cerr << "Can't find the cylindrical component���Ҳ���Բ���β�����." << std::endl;
	else
	{
		std::cerr << "PointCloud representing the cylindrical component����ʾԲ��������ĵ��ƣ�: " << cloud_cylinder->points.size() << " data points." << std::endl;
		//writer.write("table_scene_mug_stereo_textured_cylinder.pcd", *cloud_cylinder, false); //�������ļ�д��
	}

	//��ʾ��ȡ�Ľ��
	//��ȡ�ļ� 	
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->addPointCloud<pcl::PointXYZ>(cloud_cylinder, "sample cloud");
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	return 0;
}
