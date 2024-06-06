
#include "dgp.h"

#include <iostream>
#include <time.h>
#include <chrono>

#include <opencv2/imgproc.hpp>

#include <pcl/filters/filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/search/kdtree.h>

#include <pcl/features/moment_of_inertia_estimation.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include "color.hpp"

ImageColorizer *imageColorizer = new ImageColorizer();

const uint16_t MASK_OUT_CONFIDENCE = 0x3FFF;

#pragma pack(push,1)
typedef struct
{
    uint8_t grayscale;
    uint16_t distance;
} distanceGrayscale_t;
#pragma pack(pop)

static int  range             = 4000;

int h = 60;
int w = 160;

double angle_x      = 50.0f;  
double angle_y      = 18.75f; 

uint8_t  grayscale;
float    depth_val;
uint16_t x, y;

int32_t rgb;

float THETA_H = M_PI * angle_x / 180.0f;
float ALPHA_H = (M_PI - THETA_H) / 2;
float gamma_i_h;
float X;

float THETA_V = M_PI * angle_y / 180.0f;
float ALPHA_V = 2 * M_PI - (THETA_V / 2);
float gamma_i_v;
float Y;
float Z;

// read recorded data
void read_frame(std::string filename, cv::Mat* depth_map, cv::Mat* grayscale){
 
	int rows = 60;
	int cols = 160;
	*depth_map = cv::Mat::zeros(rows, cols, CV_32F);
	*grayscale = cv::Mat::zeros(rows, cols, CV_8UC1);
	
	size_t result;
	FILE* fp = fopen(filename.c_str(), "rb");

	int *data_header = new int[5];
	result = fread(data_header, 4, (size_t)(5), fp);
	int n_points = data_header[0];
	int n_point_bytes = data_header[1];
	int n_poses = data_header[2];
	int n_pose_fields = data_header[3];
	int n_pose_field_bytes = data_header[4];
	//cout << "------ HEADERS ------ " << endl;
	//cout << "n_points: " << n_points << endl;
	//cout << "n_point_bytes: " << n_point_bytes << endl;
	//cout << "n_poses: " << n_poses << endl;
	//cout << "n_pose_fields: " << n_pose_fields << endl;
	//cout << "n_pose_field_bytes: " << n_pose_field_bytes << endl;
	
	uint8_t *data = new uint8_t[n_point_bytes * n_points];
	result = fread(data, sizeof(uint8_t), (size_t)(n_point_bytes * n_points), fp);
	//cout << "result: " << result << endl;

	for (int i = 0; i < n_points; i ++) {
		distanceGrayscale_t *distanceGrayscale =  (distanceGrayscale_t *)data;

		unsigned int d = distanceGrayscale[i].distance & MASK_OUT_CONFIDENCE;
		unsigned int g = distanceGrayscale[i].grayscale;
		//cout << "point " << i << " grayscale: " << g << " distance: " << d << endl;
		int x = i % 160;
		int y = (int) (i / 160);
		float fd = 0.0;
		if (d >= 16000) {
			fd = std::numeric_limits<float>::quiet_NaN();
		} else {
			fd = 0.001 * d;
		}
		depth_map->at<float>(y, x) = fd;
		grayscale->at<uint8_t>(y, x) = g;
	}

	fclose(fp);
	//cout << "read_frame done. " << endl;
}

void to_point_cloud(cv::Mat depth_mat, cv::Mat grayscale_mat, pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud, cv::Mat &depth_bgr, cv::Mat_<cv::Point3f> &point3f_mat_, int pointCloudColor, float *data_background) {

	imageColorizer->setRange(0, range);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud_temp(new pcl::PointCloud<pcl::PointXYZRGB>);
	Color c;
	uint8_t r;
	uint8_t g;
	uint8_t b;
	int index;
	//cout << "to_point_cloud 0 " << endl;
	for(y = 0; y < 60; y++){
		for(x = 0; x < 160; x++){
			depth_val = depth_mat.at<float>(y, x);
			grayscale = grayscale_mat.at<uint8_t>(y, x);
			//std::cout << "	depth_val: " << depth_val << std::endl;
			//std::cout << "	grayscale: " << grayscale << std::endl;
			
			//if(depth_val != std::numeric_limits<float>::quiet_NaN()) {
			if(depth_val > 0.0) {
				
				c = imageColorizer->getColor((int) (depth_val * 1000));

				r = c.r;
				g = c.g;
				b = c.b;

				if (pointCloudColor == 1) {
					rgb = ((uint32_t)grayscale << 16 | (uint32_t)grayscale << 8 | (uint32_t)grayscale);
				} else {
					rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
				}

				gamma_i_h = ALPHA_H + (float)x * (THETA_H / w);
				gamma_i_v = ALPHA_V + (float)y * (THETA_V / h);

				Z = abs(depth_val * sin(gamma_i_h));
				Z = abs(Z * cos(gamma_i_v));

				X = Z / tan(gamma_i_h);
				Y = -1 * Z * tan(gamma_i_v);

				index = 160 * y + x;
				if (Z < data_background[index]) 
				{
					pcl::PointXYZRGB point;
					point.x = X;
					point.y = Y;
					point.z = Z;

					point.rgb = *reinterpret_cast<float*>(&rgb);

					_cloud_temp->points.push_back(point);
				}

				cv::Point3f &point3f = point3f_mat_.at<cv::Point3f>(y, x);
				point3f.x = X;
				point3f.y = Y;
				point3f.z = Z;
				cv::Vec3b &color = depth_bgr.at<cv::Vec3b>(cv::Point(x,y));
				color[0] = b;
				color[1] = g;
				color[2] = r;

			} else{
				cv::Vec3b &color = depth_bgr.at<cv::Vec3b>(cv::Point(x,y));
				color[0] = 0;
				color[1] = 0;
				color[2] = 0;
			}
		} //ensd for x
	} //end for y
	_cloud->points.clear();
	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*_cloud_temp, *_cloud, indices);
/*
	_cloud->points.clear();
	if (_cloud_temp->points.size() > 0) {

		pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
		sor.setMeanK(30);
		sor.setStddevMulThresh(1.0);
		sor.setInputCloud(_cloud_temp);
		sor.filter(*_cloud_temp);
		
		std::vector<int> indices;
		pcl::removeNaNFromPointCloud(*_cloud_temp, *_cloud, indices);
	}
*/
	*_cloud = *_cloud_temp;
	_cloud->resize(_cloud->points.size());
	_cloud->width = _cloud->points.size();
	_cloud->height = 1;
	_cloud->is_dense = false;		
}
// end of reading recorded data
