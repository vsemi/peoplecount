#pragma once

#include <stdint.h>
#include <vector>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/common/io.h>

typedef pcl::PointXYZRGB Point;
typedef pcl::PointCloud<Point>::Ptr PointCloud;

static int next_object_id = 0;

struct TrackedObject
{
	int id;
	PointCloud cloud;

	Point center, origin, origin_min, origin_max;
	float dx, dy, min_x, min_y, min_z, max_x, max_y, max_z;

	bool tracked;

	bool moved;
	double movement;

	float height;
};

bool evaluate(PointCloud cloud, Point &center, float &dx, float &dy, float &min_x , float &min_y, float &min_z, float &max_x, float &max_y, float &max_z);

void refersh_track_objects(std::vector<TrackedObject> &tracked_objects);

void find_and_track_object(std::vector<TrackedObject> &tracked_objects, TrackedObject o);

void audit_track_objects(std::vector<TrackedObject> &tracked_objects, int &in, int &out);
