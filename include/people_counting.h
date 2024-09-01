#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <utils.h>

size_t read(std::string path, float *background);
void write(std::string path, float *background);
void train(float *data, float *background);
int detect(
	std::vector<Cloud> cloud_clusters, 
	int &in, int &out, 
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_clustered, 
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_detected, 
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_centers,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_debug
);
