#pragma once

#include <stdint.h>
#include <vector>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

void filterAndDownSample(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_output, float *data_background);
