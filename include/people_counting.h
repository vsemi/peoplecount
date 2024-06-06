#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

size_t read(std::string path, float *background);
void write(std::string path, float *background);
void train(float *data, float *background);
int detect(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr, int &in, int &out);
