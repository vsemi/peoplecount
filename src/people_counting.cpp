#include "people_counting.h"

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

#include <opencv2/imgproc.hpp>

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

#include "tracking.h"

std::vector<TrackedObject> tracked_objects;

size_t read(std::string path, float *background)
{
    size_t result;
    FILE* fp = fopen(path.c_str(), "rb");

    result = fread(background, sizeof(float), (size_t)(sizeof(float) * 9600), fp);

    return result;
}
void write(std::string filename, float *background)
{
    std::ofstream zOut(filename, std::ios::out | std::ios::binary);
    zOut.write(reinterpret_cast<char*>(background), 9600 * sizeof(float));
    zOut.close();
}
void train(float *data, float *background)
{
	float x, y, z;
	int index;
	for (int i = 0; i < 9600; i ++)
	{
		index = i * 8;
		x = data[index];
		y = data[index + 1];
		z = data[index + 2];
		//std::cout << "i: " << i << " z: " << z << " background[i]: " << background[i] << std::endl;

		if (z > 0.0) 
		{
			if (background[i] > 0.0)
			{
				if (z < background[i])
				{
					background[i] = z;
					//std::cout << "set -> i: " << i << " z: " << z << " background[i]: " << background[i] << std::endl;
				}
			} else 
			{
				background[i] = z;
				//std::cout << "set -> i: " << i << " z: " << z << " background[i]: " << background[i] << std::endl;
			}
		}
	}
}

int detect(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_objects, int &in, int &out)
{
	//std::cout << "   detect -> cloud_objects->points: " << cloud_objects->points.size() << std::endl;

	refersh_track_objects(tracked_objects);

	if (cloud_objects->points.size() >= 15)
	{
		pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
		tree->setInputCloud(cloud_objects);

		std::vector<pcl::PointIndices> cluster_indices;
		pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
		ec.setClusterTolerance(0.1);
		ec.setMinClusterSize(15);
		ec.setMaxClusterSize(25000);
		ec.setSearchMethod(tree);
		ec.setInputCloud(cloud_objects);
		ec.extract(cluster_indices);
		int cc = 0;

		for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
		{
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
			for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
				cloud_cluster->points.push_back(cloud_objects->points[*pit]);
			}
			
			cc ++;

			if (cloud_cluster->points.size() < 3)  continue;

			//std::cout << "      detect -> cloud_cluster->points.size(): " << cloud_cluster->points.size() << std::endl;
			
			TrackedObject tracked_object;
			tracked_object.cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
			*(tracked_object.cloud) = *cloud_cluster;

			if (evaluate(
				tracked_object.cloud, tracked_object.center, 
				tracked_object.dx, tracked_object.dy, 
				tracked_object.min_x , tracked_object.min_y, tracked_object.min_z, 
				tracked_object.max_x, tracked_object.max_y, tracked_object.max_z)
			)
			{
				//std::cout << "      -> tracked_object.center.y: " << tracked_object.center.y << std::endl;
				find_and_track_object(tracked_objects, tracked_object);
			}
		}
		//if (cc > 1) std::cout << "   detect -> cc: " << cc << std::endl;
	}

	//int prev_in = in;
	//int prev_out = out;

	audit_track_objects(tracked_objects, in, out);
	//if (tracked_objects.size() > 0) std::cout << "   -> tracked_objects.size(): " << tracked_objects.size() << std::endl;

	//if (prev_in != in || prev_out != out) std::cout << "   detect -> in: " << in << " out: " << out << std::endl;

	return tracked_objects.size();
}
