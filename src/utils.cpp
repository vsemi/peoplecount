#include "utils.h"

#include <stdio.h>
#include <fstream>
#include <iostream>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <thread>

#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/search/kdtree.h>

#include <pcl/filters/uniform_sampling.h>

#include <pcl/features/moment_of_inertia_estimation.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

void split_cloud_4(
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_1,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_2,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_3,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_4
)
{
	int n_points = cloud->points.size();
	for (int i = 0; i < n_points; i ++)
	{
		pcl::PointXYZRGB p = cloud->points[i];
		
		if (p.x >= 0 && p.y >= 0)
		{
			cloud_1->points.push_back(p);
		} 
		else if (p.x >= 0 && p.y < 0)
		{
			cloud_2->points.push_back(p);
		}
		else if (p.x < 0 && p.y >= 0)
		{
			cloud_3->points.push_back(p);
		}
		else if (p.x < 0 && p.y < 0)
		{
			cloud_4->points.push_back(p);
		}		
	}

	int n_1_points = cloud_1->points.size();	
	cloud_1->resize(n_1_points);
	cloud_1->width = n_1_points;
	cloud_1->height = 1;
	cloud_1->is_dense = false;

	int n_2_points = cloud_2->points.size();	
	cloud_2->resize(n_2_points);
	cloud_2->width = n_2_points;
	cloud_2->height = 1;
	cloud_2->is_dense = false;

	int n_3_points = cloud_3->points.size();	
	cloud_3->resize(n_3_points);
	cloud_3->width = n_3_points;
	cloud_3->height = 1;
	cloud_3->is_dense = false;

	int n_4_points = cloud_4->points.size();	
	cloud_4->resize(n_4_points);
	cloud_4->width = n_4_points;
	cloud_4->height = 1;
	cloud_4->is_dense = false;
}

void filter_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	if (cloud->points.size() >= 3)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_result(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
		sor.setMeanK(5);
		sor.setStddevMulThresh(1.0);
		sor.setInputCloud(cloud);
		sor.filter(*cloud_result);

		*cloud = *cloud_result;
	}
}

void filterAndDownSample(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_output, float *data_background)
{
	int n_points = cloud_input->points.size();
	for (int i = 0; i < n_points; i ++)
	{
		pcl::PointXYZRGB p = cloud_input->points[i];
		float d_background = data_background[i];
		bool is_valid = false;
		if (std::isfinite(d_background) && std::isfinite(p.z) && p.z < (d_background - 0.5))
		{
			is_valid = true;
		} else if ((! std::isfinite(d_background)) && std::isfinite(p.z))
		{
			is_valid = true;
		}
		if (is_valid)
		{
			cloud_output->points.push_back(p);
		}
	}
	cloud_output->width = cloud_output->points.size();
	cloud_output->height = 1;
	cloud_output->is_dense = false;

	if (cloud_output->points.size() >= 3)
	{
		//std::cout << "   detect -> clean -> points: " << cloud_output->points.size() << std::endl;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_1(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_2(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_3(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_4(new pcl::PointCloud<pcl::PointXYZRGB>);

		split_cloud_4(cloud_output, cloud_1, cloud_2, cloud_3, cloud_4);

		std::thread th_filter_1(&filter_cloud, cloud_1);
		std::thread th_filter_2(&filter_cloud, cloud_2);
		std::thread th_filter_3(&filter_cloud, cloud_3);
		std::thread th_filter_4(&filter_cloud, cloud_4);

		th_filter_1.join();
		th_filter_2.join();
		th_filter_3.join();
		th_filter_4.join();

		*cloud_output = *cloud_1;
		*cloud_output += *cloud_2;
		*cloud_output += *cloud_3;
		*cloud_output += *cloud_4;
	}	

	if (cloud_output->points.size() > 3) 
	{
		//std::cout << "   detect -> filtered -> points: " << cloud_output->points.size() << std::endl;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::UniformSampling<pcl::PointXYZRGB> uniform_sampling;
		uniform_sampling.setInputCloud(cloud_output);
		uniform_sampling.setRadiusSearch(0.075);
		uniform_sampling.filter (*cloud_downsampled);

		*cloud_output = *cloud_downsampled;
		//std::cout << "   detect -> downsampled -> points: " << cloud_output->points.size() << std::endl;
	}	
}