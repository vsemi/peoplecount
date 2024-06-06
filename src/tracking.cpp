#include "tracking.h"

#include <iostream>
#include <time.h>
#include <string>
#include <thread>
#include <chrono>

bool evaluate(PointCloud cloud, Point &center, float &dx, float &dy, float &min_x , float &min_y, float &min_z, float &max_x, float &max_y, float &max_z)
{
	int points = cloud->points.size();
	min_x = 7.5;
	min_y = 7.5;
	min_z = 7.5;
	max_x = -7.5;
	max_y = -7.5;
	max_z = -7.5;
	for (int i = 0; i < points; i ++)
	{
		Point p = cloud->points[i];
		if (p.z < min_z)
		{
			min_z = p.z;
			center = p;
		}
		if (p.z > max_z)
		{
			max_z = p.z;
		}
		if (p.x < min_x)
		{
			min_x = p.x;
		}
		if (p.x > max_x)
		{
			max_x = p.x;
		}
		if (p.y < min_y)
		{
			min_y = p.y;
		}
		if (p.y > max_y)
		{
			max_y = p.y;
		}
	}
	dx = max_x - min_x;
	dy = max_y - min_y;
	//std::cout << "   -> evaluate cluser - dx: " << dx << " dy: " << dy << " min_x: "  << min_x << " max_x: "  << max_x << " min_y: "  << min_y << " max_y: "  << max_y << " min_z: "  << min_z << " max_z: "  << max_z  << " points: " << points << std::endl;

	center.x = 0.5 * (max_x + min_x);
	center.y = 0.5 * (max_y + min_y);
	if (dx > 0.2 || dy > 0.2)
	{
		//std::cout << "   -> valid cluser - dx: " << dx << " dy: " << dy << " min_x: "  << min_x << " max_x: "  << max_x << " min_y: "  << min_y << " max_y: "  << max_y << " min_z: "  << min_z << " max_z: "  << max_z  << " points: " << points << std::endl;
		return true;
	} else {
		//std::cout << "   -> invalid cluser - dx: " << dx << " dy: " << dy << " min_x: "  << min_x << " max_x: "  << max_x << " min_y: "  << min_y << " max_y: "  << max_y << " min_z: "  << min_z << " max_z: "  << max_z  << " points: " << points << std::endl;
	}
	
	return false;
}

void refersh_track_objects(std::vector<TrackedObject> &tracked_objects)
{
	for (size_t i = 0; i < tracked_objects.size(); i ++)
	{
		TrackedObject &t_o = tracked_objects[i];
		t_o.tracked = false;
	}
}

void find_and_track_object(std::vector<TrackedObject> &tracked_objects, TrackedObject o)
{
	bool found = false;

	float nearest_distance = 100.0;
	int nearest_index = -1;
	
	float nearest_distance_moved = 100.0;
	int nearest_index_moved = -1;
	std::vector<int> movedObjectIDs;

	for (size_t i = 0; i < tracked_objects.size(); i ++)
	{
		TrackedObject t = tracked_objects[i];
		if (t.tracked) continue; 

		 float ovx = std::min(0.1 * t.dx, 0.1 * o.dx);
		 float ovy = std::min(0.1 * t.dy, 0.1 * o.dy);
		 
		//std::cout << "      -> ovx: " << ovx << " ovy:" << ovy << std::endl;
		//std::cout << "   -> t.id: " << t.id << " t.dx: " << t.dx << " t.dy: " << t.dy << " t.min_x: "  << t.min_x << " t.max_x: "  << t.max_x << " t.min_y: "  << t.min_y << " t.max_y: "  << t.max_y << " t.max_z: "  << t.max_z  << " t.points: " << t.cloud->points.size() << std::endl;
		if (
			t.min_x + ovx > o.max_x ||
			t.max_x - ovx < o.min_x ||
			t.max_y - ovy < o.min_y ||
			t.min_y + ovy > o.max_y
		)
		{
			//std::cout << "      -> not overlapped id: " << t.id << " ......................................... " << std::endl;
		} else 
		{
			float dx = t.center.x - o.center.x;
			float dy = t.center.y - o.center.y;
			float d = sqrt(dx * dx + dy * dy);
			
			//std::cout << "      -> overlapped id: " << t.id << " d: " << d << " t.moved: " << t.moved << " --d: " << d << " -nearest_distance: " << nearest_distance << std::endl;
			
			if (d < nearest_distance)
			{
				nearest_distance = d;
				nearest_index = i;
			}
			if (t.moved)
			{
				movedObjectIDs.push_back(i);
			}
			if (t.moved && d < nearest_distance_moved)
			{
				nearest_distance_moved = d;
				nearest_index_moved = i;
			}
			found = true;
		}
	}
	if (found)
	{
		if (movedObjectIDs.size() == 1)
		{
			nearest_index = nearest_index_moved;
		} else
		{
			float max_movement = 0;
			for (size_t j = 0; j < movedObjectIDs.size(); j ++)
			{
				TrackedObject t = tracked_objects[movedObjectIDs[j]];
				if (t.movement > max_movement)
				{
					max_movement = t.movement;
					nearest_index = movedObjectIDs[j];
					//std::cout << "      -> overlapped moved id: " << t.id << " t.movement: " << t.movement << " t.moved: " << t.moved << std::endl;
				}
			}
		}
	}
	//std::cout << "      -> nearest overlapped points index: " << nearest_index << " nearest_distance: " << nearest_distance << std::endl;
	
	if (found)
	{
		//std::cout << "      -> feeding points: " << o.cloud->points.size() << std::endl;
		TrackedObject &t_o = tracked_objects[nearest_index];
		
		//std::cout << "      -> tracked object id: " << t_o.id << std::endl;
		//std::cout << "      -> tracked t_o.id: " << t_o.id << " t_o.dx: " << t_o.dx << " t_o.dy: " << t_o.dy << " t_o.min_x: "  << t_o.min_x << " t_o.max_x: "  << t_o.max_x << " t_o.min_y: "  << t_o.min_y << " t_o.max_y: "  << t_o.max_y << " t_o.max_z: "  << t_o.max_z  << " t_o.points: " << t_o.cloud->points.size() << std::endl;
		//std::cout << "      -> tracking found points: " << t_o.cloud->points.size() << " nearest_distance: " << nearest_distance << std::endl;

		float dx = t_o.origin.x - o.center.x;
		float dy = t_o.origin.y - o.center.y;
		
		float d = sqrt(dx * dx + dy * dy);
		
		if (d > t_o.movement) t_o.movement = d;
		//std::cout << "         -> movement: " << t_o.movement << " t_o.dx: " << t_o.dx << std::endl;
		
		if (t_o.movement >= 0.025)
		{
			//std::cout << "         -> moved: " << d << std::endl;
			t_o.moved = true;
		}

		*(t_o.cloud) = *(o.cloud);
		t_o.tracked = true;
		t_o.center = o.center;
		t_o.dx = o.dx;
		t_o.dy = o.dy;
		t_o.min_x = o.min_x;
		t_o.min_y = o.min_y;
		t_o.max_x = o.max_x;
		t_o.max_y = o.max_y;
		t_o.max_z = o.max_z;

		if ((2.5 - t_o.min_z) > t_o.height) t_o.height = 2.5 - t_o.min_z;
	}
	if (! found)
	{
		o.tracked = true;
		o.id = next_object_id;

		//std::cout << "            -> not found, adding new ...: " << std::endl;
		o.origin = o.center;
		o.moved = false;
		
		o.height = 2.5 - o.center.z; 
		o.movement = 0.0;

		Point origin_min, origin_max;
		o.origin_min.x = o.min_x;
		o.origin_min.y = o.min_y;
		o.origin_min.z = o.min_z;

		o.origin_max.x = o.max_x;
		o.origin_max.y = o.max_y;
		o.origin_max.z = o.max_z;

		//std::cout << "      -> new object id: " << o.id << " o.origin.x: " << o.origin.x << " o.origin.y: " << o.origin.y << std::endl;

		tracked_objects.push_back(o);
		next_object_id ++;
	}
}

void audit_track_objects(std::vector<TrackedObject> &tracked_objects, int &in, int &out)
{
	int total_tracked = 0;
	for (size_t i = 0; i < tracked_objects.size(); i ++)
	{
		TrackedObject &t_o = tracked_objects[i];
		if (! t_o.tracked)
		{
			//std::cout << "         -> lost object id: " << t_o.id << " origin.x: " << t_o.origin.x << " origin.y: " << t_o.origin.y << " center.x: " << t_o.center.x << " center.y: " << t_o.center.y << " movement: " << t_o.movement << " height: " << t_o.height << std::endl;
			//std::cout << "            min_x: " << t_o.min_x << " min_y: " << t_o.min_y << " min_z: " << t_o.min_z << " max_x: " << t_o.max_x << " max_y: " << t_o.max_y << " max_z: " << t_o.max_z << std::endl;
			//std::cout << "     origin min_x: " << t_o.origin_min.x << " min_y: " << t_o.origin_min.y << " min_z: " << t_o.origin_min.z << " max_x: " << t_o.origin_max.x << " max_y: " << t_o.origin_max.y << " max_z: " << t_o.origin_max.z << std::endl;

			//std::cout << "      -> t_o.origin.y: ================> " << t_o.origin.y << " t_o.center.y: " << t_o.center.y << " t_o.movement: " << t_o.movement << std::endl;
			//if (t_o.origin.y > t_o.center.y && t_o.movement > 0.15)
			if (t_o.origin.y > 0.025 && t_o.center.y < 0.025 && t_o.movement > 0.15)
			{
				//std::cout << "      -> one person in, ID: ================> " << t_o.id << std::endl;
				in ++;
			}
			//else if (t_o.origin.y < t_o.center.y && t_o.movement > 0.15)
			else if (t_o.origin.y < 0.025 && t_o.center.y > 0.025 && t_o.movement > 0.15)
			{
				//std::cout << "      -> one person in ================> " << t_o.id << std::endl;
				out ++;
			}
			tracked_objects.erase (tracked_objects.begin() + i);
			i --;
		} else
		{
			total_tracked ++;
		}
	}
	//std::cout << "   audit_track_objects -> in: " << in << " out: " << out << std::endl;
	//std::cout << "   audit_track_objects -> total_tracked: " << total_tracked << std::endl;
}
