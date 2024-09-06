#include <stdio.h>
#include <fstream>
#include <iostream>
#include <string.h>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <thread>
#include <mutex>
#include <errno.h>
#include <fcntl.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <pwd.h>
#include <algorithm>
#include <signal.h>

#include <arpa/inet.h>
#include <cerrno>
#include <ifaddrs.h>
#include <net/if.h>
#include <sysexits.h>
#include <sys/socket.h>

#include <sys/reboot.h>

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/filesystem.hpp>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

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

#include <Camera.hpp>
#include "people_counting.h"
#include "dgp.h"
#include "utils.h"

#include <sqlite3.h>

#include "MQTTAsync.h"
#include "lora.hpp"
#include "led.h"

#include "MJPGStreamer.h"

u_int32_t sensor_uid;

const char *homedir;
const char *sd_card = "";
const char *comm_protocal = "";

char* action;

Camera* camera;
float *data_background;
sqlite3 *db;

volatile bool exit_requested = false;

unsigned int integrationTime0 = 300;
unsigned int integrationTime1 = 50;

unsigned int amplitude0 = 100;
unsigned int amplitude1 = 100;

int hdr = 0;

bool is_stopped = false;
cv::Mat depth_bgr(60, 160, CV_8UC3, cv::Scalar(0, 0, 0));
cv::Mat_<cv::Point3f> point3f_mat_(60, 160);

std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> queque_raw;
std::vector<Cloud> queque_filtered;
std::vector<std::vector<Cloud>> queque_clustered;
std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> queque_record;

std::mutex mutex_queque_raw;
std::mutex mutex_queque_filtered;
std::mutex mutex_queque_clustered;
std::mutex mutex_queque_record;

std::string title;

MQTTAsync client;
std::string mqtt_topic;
LoRaUart lora;
bool mqtt_subscribed = false;
bool lora_available = false;

bool mqtt_connected = false;

int ping_enabled = 0;

const char* serverURI = "ssl://a1y8b7gmw22f9i-ats.iot.us-east-2.amazonaws.com:8883";

const char* ca_file     = "/home/vsemi/aws/root-CA.crt";
const char* client_cert = "/home/vsemi/aws/mqtt5.certificate.pem";
const char* client_key  = "/home/vsemi/aws/mqtt5.private.key";
std::string clientId = "";
const char* clientPass = "";

char const *LED_RED = "83";
char const *LED_GREEN = "84";

bool led_available = false;

std::string otg_ip_address = "10.42.0.1";//"10.10.31.191";

MQTTAsync_connectOptions conn_opts = MQTTAsync_connectOptions_initializer5;

int subscribe_mqtt_topic();
int conn_sub_mqtt();

MJPGStreamer* streamer;

std::vector<std::string> queue_payload;
int send_message();

inline bool file_exists (const std::string& name) {
	if (name != "") {
		struct stat buffer;
		return (stat (name.c_str(), &buffer) == 0);
	}

	return false;
}

bool has_ip_address(std::string ip)
{
    struct ifaddrs* ptr_ifaddrs = nullptr;

    auto result = getifaddrs(&ptr_ifaddrs);
    if( result != 0 ){
        std::cout << "`getifaddrs()` failed: " << strerror(errno) << std::endl;

        return EX_OSERR;
    }

    for(
        struct ifaddrs* ptr_entry = ptr_ifaddrs;
        ptr_entry != nullptr;
        ptr_entry = ptr_entry->ifa_next
    ){
        std::string ipaddress_human_readable_form;
        std::string netmask_human_readable_form;

        std::string interface_name = std::string(ptr_entry->ifa_name);
        sa_family_t address_family = ptr_entry->ifa_addr->sa_family;
        if( address_family == AF_INET ){

            if( ptr_entry->ifa_addr != nullptr ){
                char buffer[INET_ADDRSTRLEN] = {0, };
                inet_ntop(
                    address_family,
                    &((struct sockaddr_in*)(ptr_entry->ifa_addr))->sin_addr,
                    buffer,
                    INET_ADDRSTRLEN
                );

                ipaddress_human_readable_form = std::string(buffer);

				std::cout << interface_name << ": IP address = " << ipaddress_human_readable_form << std::endl;

				if (ipaddress_human_readable_form == ip) return true;
            }
        }
    }

    freeifaddrs(ptr_ifaddrs);

    return false;
}

int empty_frames_interval = 0;
void process(Camera* camera)
{
	ErrorNumber_e status;

	ToFImage tofImage(camera->getWidth(), camera->getHeight());
	camera->setIntegrationTimeGrayscale(0);

	std::chrono::steady_clock::time_point st_time0;
	std::chrono::steady_clock::time_point en_time0;

	std::chrono::steady_clock::time_point st_time;
	std::chrono::steady_clock::time_point en_time;

	double interval, frame_rate;
	st_time0 = std::chrono::steady_clock::now();
	st_time = std::chrono::steady_clock::now();
	int data_frame_id = 0;

	while (! exit_requested)
	{
		try
		{

			//en_time = std::chrono::steady_clock::now();
			//interval = ((double) std::chrono::duration_cast<std::chrono::microseconds>(en_time - st_time).count()) / 1000.0;
			//if (interval < 100)
			//{
			//	usleep(1000);
			//	continue;
			//}
			//st_time = std::chrono::steady_clock::now();

			//std::cout << "To request next frame ... " << std::endl;
			status = camera->getDistance(tofImage);
			if (status != ERROR_NUMMBER_NO_ERROR)
			{
				std::cerr << "Error: " << status << std::endl;
				usleep(500000);
				continue;
			}

			if (strcmp(action, "train") == 0 || strcmp(action, "train-detect") == 0 || strcmp(action, "train-record") == 0)
			{
				train(tofImage.data_3d_xyz_rgb, data_background);
				if (data_frame_id >= 10 && data_frame_id < 100 && data_frame_id % 10 == 0)
				{
					std::cout << "Training in progress: " << data_frame_id << "%, please wait ..." << std::endl;

					if (led_available)
					{
						gpio_high(LED_RED);
					}
				} else
				{
					if (led_available)
					{
						gpio_low(LED_RED);
					}
				}
				if (data_frame_id > 100)
				{
					std::string fn = std::string(sd_card) + "/model.bin";
					write(fn, data_background);

					std::cout << "\nTraining completed, starting detect ..." << std::endl;

					if (strcmp(action, "train") == 0)
					{
						break;
					} else if (strcmp(action, "train-detect") == 0)
					{
						action = (char*)"detect";
						title = " Recording";
					} else if (strcmp(action, "train-record") == 0)
					{
						action = (char*)"record";
						title = " Recording";
					}
					if (led_available)
					{
						gpio_high(LED_RED);
					}

					ping_enabled = 5;
				}
			}
			else if (strcmp(action, "view") == 0)
			{
				depth_bgr = cv::Mat(tofImage.height, tofImage.width, CV_8UC3, tofImage.data_2d_bgr);

				cv::Mat depth_bgr_enlarge;
				if (hdr == 1)
				{
					cv::Rect roi(0, 0, 160, 30);
					cv::Mat depth_bgr_spatial = depth_bgr(roi);
					cv::resize(depth_bgr_spatial, depth_bgr_enlarge, cv::Size(1920, 720), cv::INTER_LINEAR);
				} else{
					cv::resize(depth_bgr, depth_bgr_enlarge, cv::Size(1920, 720), cv::INTER_LINEAR);
				}

				cv::Mat _blank(360, 1920, CV_8UC3, cv::Scalar(0, 0, 0));
				cv::Mat depth_bgr_display;
				cv::vconcat(depth_bgr_enlarge, _blank, depth_bgr_display);

				cv::putText(
					depth_bgr_display, "Device ID: " + std::to_string(sensor_uid), cv::Point(20, 780),
					cv::FONT_HERSHEY_DUPLEX, 1.4, cv::Scalar(255,255,255), 1, cv::LINE_AA
				);

				cv::putText(
					depth_bgr_display, "Make sure part of the gate stub or door frame can be seen by ToF camera.", cv::Point(20, 840),
					cv::FONT_HERSHEY_DUPLEX, 1.2, cv::Scalar(255,255,255), 1, cv::LINE_AA
				);

				cv::putText(
					depth_bgr_display, "For more info, visit http://www.vsemi.io", cv::Point(1020, 960),
					cv::FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(255,255,255), 1, cv::LINE_AA
				);
				cv::putText(
					depth_bgr_display, "2023 Visionary Semiconductor Inc. All rights reserved", cv::Point(1020, 1000),
					cv::FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(255,255,255), 1, cv::LINE_AA
				);

				streamer->write(depth_bgr_display);
			} else
			{
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

				if (strcmp(action, "detect") == 0 || strcmp(action, "record") == 0 || strcmp(action, "train-detect") == 0 || strcmp(action, "train-record") == 0)
				{
					pcl::PointXYZRGB* data_ptr = reinterpret_cast<pcl::PointXYZRGB*>(tofImage.data_3d_xyz_rgb);
					std::vector<pcl::PointXYZRGB> pts(data_ptr, data_ptr + tofImage.n_points);
					point_cloud_ptr->points.clear();
					point_cloud_ptr->points.insert(point_cloud_ptr->points.end(), pts.begin(), pts.end());
					point_cloud_ptr->resize(tofImage.n_points);
					point_cloud_ptr->width = tofImage.n_points;
					point_cloud_ptr->height = 1;
					point_cloud_ptr->is_dense = false;
				}

				std::unique_lock<std::mutex> lock_queque_raw(mutex_queque_raw);
				queque_raw.push_back(point_cloud_ptr);
				//std::cout << "   =====================> queque_raw:  " << queque_raw.size() << std::endl;
				lock_queque_raw.unlock();

				if (strcmp(action, "record") == 0)
				{
					std::unique_lock<std::mutex> lock_queque_record(mutex_queque_record);
					if (point_cloud_ptr->points.size() >= 15)
					{
						queque_record.push_back(point_cloud_ptr);
						empty_frames_interval = 0;
					} else
					{
						if (empty_frames_interval < 5)
						{
							queque_record.push_back(point_cloud_ptr);
							empty_frames_interval ++;
						}
					}
					lock_queque_record.unlock();
				}
			}

			data_frame_id ++;

		}
		catch( ... )
		{
			std::cerr << "Unknown error in process ... " << std::endl;
		}
	}

	exit_requested = true;

	en_time0 = std::chrono::steady_clock::now();
	interval = ((double) std::chrono::duration_cast<std::chrono::microseconds>(en_time0 - st_time0).count()) / 1000000.0;
	frame_rate = ((double) data_frame_id) / interval;
	std::cout << "Frames: " << data_frame_id << " time spent: " << interval << " frame rate: " << frame_rate << std::endl;
}

int save_to_db(int i, int o, time_t t)
{
	char *zErrMsg = 0;
	std::string sql = "INSERT INTO PEOPLECOUNT(DEVICEID, ENTER, EXIT, DATETIME) VALUES(" + std::to_string(sensor_uid) + ", " + std::to_string(i) + ", " + std::to_string(o) + ", " + std::to_string(t) + "); ";
	//std::cout << "sql: " << sql << std::endl;

	int rc = sqlite3_exec(db, sql.c_str(), [](void *NotUsed, int argc, char **argv, char **azColName){
		int i;
		for(i = 0; i<argc; i++) {
			printf("%s = %s\n", azColName[i], argv[i] ? argv[i] : "NULL");
		}
		printf("\n");
		return 0;
	}, 0, &zErrMsg);

	if( rc != SQLITE_OK ){
		fprintf(stderr, "SQL error: %s\n", zErrMsg);
		sqlite3_free(zErrMsg);
	} else {
		//fprintf(stdout, "Data inserted successfully\n");
	}

	return rc;
}

int retrieve_from_db(time_t ts, time_t te)
{
	char *zErrMsg = 0;
	std::string sql = "SELECT DEVICEID, ENTER, EXIT, DATETIME FROM PEOPLECOUNT WHERE DATETIME >= " + std::to_string(ts) + " AND DATETIME <= " + std::to_string(te) + "; ";
	//std::cout << "sql: " << sql << std::endl;

	int rc = sqlite3_exec(db, sql.c_str(), [](void *data, int argc, char **argv, char **azColName){

		int device = sensor_uid;
		int enter = std::stoi( argv[1] );
		int exit = std::stoi( argv[2] );
		time_t dt = (time_t) std::stoi( argv[3] );

		//std::cout << "   device: " << device << " enter: " << enter << " exit: " << exit << " dt: " << std::to_string(dt) << std::endl;

		std::string payload = "{\"device_id\":\"" + std::to_string(device) + "\", \"msg_type\":\"uplink\", \"dt\":\"" + std::to_string(dt) + "\"";
		payload += ", \"uplink_data\":[{\"in\": \"" + std::to_string(enter) + "\", \"out\": \"" + std::to_string(exit) + "\"}]";
		payload += "}";
		queue_payload.push_back(payload);

		return 0;
	}, 0, &zErrMsg);

	if( rc != SQLITE_OK ){
		fprintf(stderr, "SQL error: %s\n", zErrMsg);
		sqlite3_free(zErrMsg);
	} else {
		fprintf(stdout, "Data retrived successfully\n");

		send_message();
	}

	return rc;
}

int send_message_payload(std::string payload)
{
	int len = payload.size();

	std::vector<char> bytes(payload.begin(), payload.end());
	char *data = &bytes[0];

	int rc;
	if (strcmp(comm_protocal, "mqtt") == 0 && mqtt_connected)
	{
		MQTTAsync_responseOptions opts = MQTTAsync_responseOptions_initializer;
		opts.context = client;

		MQTTAsync_message pubmsg = MQTTAsync_message_initializer;
		pubmsg.payload = data;
		pubmsg.payloadlen = len;
		pubmsg.qos = 1;
		pubmsg.retained = 0;

		if ((rc = MQTTAsync_sendMessage(client, mqtt_topic.c_str(), &pubmsg, &opts)) != MQTTASYNC_SUCCESS)
		{
			std::cout << "Failed to send MQTT message, rc: " << rc << std::endl;
		}
		//std::cout << "MQTT message sent: \n" << payload << std::endl;
	}
	if (strcmp(comm_protocal, "lora") == 0 && lora_available)
	{
		lora.sendData(data, len);

		std::cerr << std::endl;
		for (int i = 0; i < len; i ++)
		{
			std::stringstream ss;
			ss << std::hex << std::setw(2) << static_cast<char>(data[i]);
			std::cerr << ss.str();
		}
		std::cerr << std::endl;
	}

	return rc;
}

int send_message()
{
	if (queue_payload.size() > 0)
	{
		if (strcmp(comm_protocal, "mqtt") == 0 && mqtt_connected)
		{
			for (int i = 0; i < queue_payload.size(); i ++)
			{
				std::string payload = queue_payload[i];

				char p[payload.size() + 1];
				strcpy(p, payload.c_str());

				int rc = send_message_payload(payload);
				if (rc == 0)
				{
					queue_payload.erase(queue_payload.begin() + i);
					i --;
				}
			}
		} else if (strcmp(comm_protocal, "lora") == 0 && lora_available)
		{
			// for low band lora, send one msg at a time
			std::string payload = queue_payload[0];

			char p[payload.size() + 1];
			strcpy(p, payload.c_str());

			int rc = send_message_payload(payload);
			if (rc == 0)
			{
				queue_payload.erase(queue_payload.begin());
			}
		}
	} else
	{
		// send heart beat
		if (ping_enabled > 0)
		{
			time_t t = std::time(nullptr);

			std::string payload = "{\"device_id\":\"" + std::to_string(sensor_uid) + "\", \"msg_type\":\"ping\", \"dt\":\"" + std::to_string(t) + "\"";
			payload += ", \"version\": \"2.1.9\"";
			payload += "}";

			send_message_payload(payload);

			ping_enabled --;
			if (ping_enabled < 0) ping_enabled = 0;
		}
	}

	return 0;
}

void pre_filter_cloud()
{
	while (! exit_requested)
	{
		try {

			if (queque_raw.size() > 0)
			{
				std::unique_lock<std::mutex> lock_queque_raw(mutex_queque_raw);
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr = queque_raw[0];
				queque_raw.erase (queque_raw.begin());
				lock_queque_raw.unlock();

				Cloud cloud;
				filterAndDownSample(point_cloud_ptr, cloud, data_background);

				std::unique_lock<std::mutex> lock_queque_filtered(mutex_queque_filtered);
				queque_filtered.push_back(cloud);
				//std::cout << "   =====================> queque_filtered:  " << queque_filtered.size() << std::endl;
				lock_queque_filtered.unlock();
			}
		} catch (...)
		{
			//
		}
	}
}

void pre_cluster_cloud()
{
	while (! exit_requested)
	{
		try {

			if (queque_filtered.size() > 0)
			{
				std::unique_lock<std::mutex> lock_queque_filtered(mutex_queque_filtered);
				Cloud cloud = queque_filtered[0];
				queque_filtered.erase (queque_filtered.begin());
				lock_queque_filtered.unlock();

				std::vector<Cloud> cloud_clusters;
				clustering_cloud(cloud, cloud_clusters, 0.03, 90, 9600);

				std::unique_lock<std::mutex> lock_queque_clustered(mutex_queque_clustered);
				queque_clustered.push_back(cloud_clusters);
				//std::cout << "   =====================> queque_clustered:  " << queque_clustered.size() << std::endl;
				lock_queque_clustered.unlock();
			}
		} catch (...)
		{
			//
		}
	}
}

int ping_count = 0;
bool led_green_on = false;
void detect_and_track()
{
	int in = 0, out = 0;
	time_t t_prev = std::time(nullptr);
	while (! exit_requested)
	{
		try {
			//std::cout << "   detect -> queque_clustered.size(): " << queque_clustered.size() << std::endl;
			if (queque_clustered.size() > 0)
			{
				std::unique_lock<std::mutex> lock_queque_clustered(mutex_queque_clustered);
				std::vector<Cloud> clusters = queque_clustered[0];
				queque_clustered.erase (queque_clustered.begin());
				lock_queque_clustered.unlock();

				pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_clustered(new pcl::PointCloud<pcl::PointXYZRGB>);
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_detected(new pcl::PointCloud<pcl::PointXYZRGB>);
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_centers(new pcl::PointCloud<pcl::PointXYZRGB>);
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_debug(new pcl::PointCloud<pcl::PointXYZRGB>);

				int tracked = detect(clusters, out, in, cloud_clustered, cloud_detected, cloud_centers, cloud_debug);
				//std::cout << "   tracked: " << tracked << " in: " << in << " out: " << out << std::endl;

				bool msg_sent = false;
				if (in != 0 || out != 0)
				{
					time_t t = std::time(nullptr);
					if ((t - t_prev) >= 1)
					{
						//std::cout << "   detect -> in: " << in << " out: " << out << std::endl;

						std::string payload = "{\"device_id\":\"" + std::to_string(sensor_uid) + "\", \"msg_type\":\"uplink\", \"dt\":\"" + std::to_string(t) + "\"";

						payload += ", \"uplink_data\":[{\"in\": \"" + std::to_string(in) + "\", \"out\": \"" + std::to_string(out) + "\"}]";

						payload += "}";

						queue_payload.push_back(payload);

						int r = send_message();

						save_to_db(in, out, t);

						in = 0;
						out = 0;
						t_prev = t;
						msg_sent = true;
					}
				}

				if (tracked > 0 && ping_count%2 == 0)
				{
					ping_count = 0;
					//std::cout << "   Track ... " << std::endl;

					if (! led_green_on)
					{
						if (led_available) gpio_high(LED_GREEN);
						led_green_on = true;
					} else
					{
						if (led_available) gpio_low(LED_GREEN);
						led_green_on = false;
					}
				} else if ((! msg_sent) && tracked == 0 && ping_count%50 == 0)
				{
					ping_count = 0;
					//std::cout << "   Ping ... " << std::endl;
					if (led_available) gpio_high(LED_GREEN);
					led_green_on = true;

					send_message();
				} else
				{
					if (led_available) gpio_low(LED_GREEN);
					led_green_on = false;
				}
				ping_count ++;
			}
		} catch(...)
		{
			std::cerr << "Unknown error in detect and track ... " << std::endl;
		}
	}
}

int i_frame_id = 0;
int i_frame_total = 0;
bool refresh_folder_path(std::string &f_path)
{
	char date_str[9];
	std::time_t t = std::time(NULL);
	std::strftime(date_str, sizeof(date_str), "%Y%m%d", std::localtime(&t));
	f_path = std::string(sd_card) + "/" + std::string(date_str);

	if (!file_exists(f_path))
	{
		int status = mkdir(f_path.c_str(),0777);
		if (status == 0)
		{
			std::cerr << "Folder " << f_path << " created, recording started ..." << std::endl;
			i_frame_id = 0;
		} else
		{
			std::cerr << "Failed to create folder in " << sd_card << ", recording skipped!" << std::endl;
			return false;
		}
	}

	return true;
}

void record_data()
{
	std::string f_path = "";
	bool file_system_ok = refresh_folder_path(f_path);
	if (! file_system_ok)
	{
		exit_requested = true;
	}

	while (! exit_requested)
	{
		try
		{

			if (queque_record.size() > 0)
			{
				file_system_ok = refresh_folder_path(f_path);
				if (! file_system_ok)
				{
					exit_requested = true;
				}

				pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_raw = queque_record[0];
				queque_record.erase (queque_record.begin());

				std::string fn = f_path + "/" + std::to_string(i_frame_id) + ".pcd";

				//std::cerr << "Saving data into fn: " << fn << std::endl;
				pcl::io::savePCDFileBinary(fn.c_str(), *cloud_raw);

				if (i_frame_total > 50000) {
					exit_requested = true;
					std::cerr << "Maximum number of frames reached: " << i_frame_total << std::endl;
					std::cerr << "Please clean data storage and restart recording again! " << std::endl;
				}

				i_frame_id ++;
				i_frame_total ++;
			} else{
				usleep(1);
			}
		} catch (...)
		{
			std::cerr << "Unknown error in record_data ... " << std::endl;
		}
	}
}

void onSubscribe(void* context, MQTTAsync_successData5* response)
{
	mqtt_subscribed = true;
	std::cout << "\n" << std::endl;
	std::cout << "Device " << sensor_uid << " registered to IoT network." << std::endl;
	std::cout << "Open people counting demo page (iot_demo.html) in browser, \n";
	std::cout << "Or copy and paste https://main.dtxphb7frfari.amplifyapp.com into browser address bar, \n";
	std::cout << "enter the device ID " << sensor_uid << " into the Device ID field \nand click Subscribe button.\n" << std::endl;
}

void onSubscribeFailure(void* context, MQTTAsync_failureData5* response)
{
	std::cout << "   MQTT Subscribe failed, rc: " << response->code << std::endl;
}

void onConnectFailure(void* context, MQTTAsync_failureData5* response)
{
	std::cout << "   MQTT connect failed, rc: " << response->code << std::endl;
	std::cout << "   message: " << std::string(response->message) << std::endl;
}

void onConnect(void* context, MQTTAsync_successData5* response)
{
	mqtt_connected = true;
	std::cout << "MQTT connected."<< std::endl;
	//subscribe_mqtt_topic();
}

int messageArrived(void* context, char* topicName, int topicLen, MQTTAsync_message* message)
{
	char* msg = (char*)message->payload;
	std::string msg_str(msg);

	//std::cout << "msg_str: "<< msg_str << std::endl;
	boost::property_tree::ptree pt;
	std::istringstream is(msg_str);
	boost::property_tree::read_json(is, pt);
	//std::cout << "   msg_str passed."<< std::endl;

	std::string type = pt.get<std::string>("msg_type");
	//std::cout << "type: "<< type << std::endl;
	if (type == "downlink")
	{
		std::string id = pt.get<std::string>("device_id");
		std::cout << "id: "<< id << std::endl;
		int i_id = std::stoi(id);
		int i_sensor_id = *(int*)&sensor_uid;
		if (i_id == i_sensor_id)
		{
			std::cout << "downlink message for id: " << id << std::endl;
			std::cout << "      msg_str: " << msg_str << std::endl;

			std::string command = pt.get<std::string>("command");
			if (command == "ping")
			{
				ping_enabled = 5;
			} else if (command == "retrieve")
			{
				std::string dt_str = pt.get<std::string>("dt");
				std::string ts_str = pt.get<std::string>("ts");

				std::cout << "retrieve request for ts: " << ts_str << " dt: " << dt_str << std::endl;

				int ts = std::stoi( ts_str );
				int te = std::stoi( dt_str );

				retrieve_from_db(ts, te);
			} else if (command == "upgrade")
			{
				std::cout << "upgrade request for id: " << id << std::endl;

				std::cout << "   -> updating firmware ... " << std::endl;
				system("/home/cat/update_firmware.sh >> /home/cat/upgrade.log &");
				//std::cout << "   -> firmware upgrade completed." << std::endl;
				//std::cout << "   -> restart ... " << std::endl;
				//system("/home/cat/restart.sh &");
				exit_requested = true;
			}
		}
	}

	MQTTAsync_freeMessage(&message);
	MQTTAsync_free(topicName);
	return 1;
}

void connlost(void *context, char *cause)
{
	mqtt_connected = false;
	fprintf(stderr, "Lost connection to broker, reconnecting...\n");
}

int subscribe_mqtt_topic()
{
	int rc;

	MQTTAsync_responseOptions opts = MQTTAsync_responseOptions_initializer;

    opts.context = client;
	opts.onSuccess5 = onSubscribe;
	opts.onFailure5 = onSubscribeFailure;
	mqtt_topic = "vsemi/iot/peoplecount/" + std::to_string(sensor_uid);

	if ((rc = MQTTAsync_subscribe(client, mqtt_topic.c_str(), 1, &opts)) != MQTTASYNC_SUCCESS)
	{
		std::cout << "   Failed to start subscribe, rc: " << rc << std::endl;
	} else
	{
		//std::cout << "Subscribed to topic: " << mqtt_topic << std::endl;
	}

	return rc;
}

static void mqtt_trace_callback(enum MQTTASYNC_TRACE_LEVELS level, char *message)
{
    if (level >= MQTTASYNC_TRACE_ERROR)
        fprintf(stderr, "%s\n", message);
}

int ssl_error_callback (const char *str, size_t len, void *u)
{
	fprintf(stderr, "%s\n", str);

	return 0;
}
void on_mqtt_connected(void *context, char *cause)
{
	std::cout << "Subscribe to the IoT topic ... " << std::endl;
	//if (! mqtt_connected)
	//{
		subscribe_mqtt_topic();
	//}
	mqtt_connected = true;
}
int connectToMQTT()
{
    MQTTAsync_setTraceCallback(mqtt_trace_callback);

	MQTTAsync_createOptions create_opts = MQTTAsync_createOptions_initializer5;

	conn_opts.keepAliveInterval = 30;
	conn_opts.MQTTVersion = MQTTVERSION_5;
	conn_opts.automaticReconnect = 1;
	conn_opts.cleanstart = 1;

	conn_opts.onSuccess5 = onConnect;
	conn_opts.onFailure5 = onConnectFailure;

	MQTTAsync_SSLOptions ssl_opts = MQTTAsync_SSLOptions_initializer;

	ssl_opts.verify = 1;
	ssl_opts.trustStore = ca_file;
	ssl_opts.keyStore = client_cert;
	ssl_opts.privateKey = client_key;
	ssl_opts.ssl_error_cb = &ssl_error_callback;
	ssl_opts.ssl_error_context = client;

	ssl_opts.sslVersion = MQTT_SSL_VERSION_TLS_1_2;
	ssl_opts.enableServerCertAuth = false;

	conn_opts.ssl = &ssl_opts;

	int rc;

	//std::cout << "To create MQTTAsync client ..." << std::endl;
	if ((rc = MQTTAsync_createWithOptions(&client, serverURI, clientId.c_str(), MQTTCLIENT_PERSISTENCE_NONE, NULL, &create_opts)) != MQTTASYNC_SUCCESS)
	{
		std::cout << "   Failed to create MQTT client, rc: " << rc << std::endl;
		rc = EXIT_FAILURE;
	}
	//std::cout << "MQTTAsync create client done: " << rc << std::endl;
	if (rc == 0)
	{
		if ((rc = MQTTAsync_setConnected(client, NULL, on_mqtt_connected)) != MQTTASYNC_SUCCESS)
		{
			printf("Failed to set callback, return code %d\n", rc);
			rc = EXIT_FAILURE;
		}
	}
	if (rc == 0)
	{
		if ((rc = MQTTAsync_setCallbacks(client, NULL, connlost, messageArrived, NULL)) != MQTTASYNC_SUCCESS)
		{
			printf("Failed to set callback, return code %d\n", rc);
			rc = EXIT_FAILURE;
		}
	}
	//std::cout << "MQTTAsync setCallbacks done: " << rc << std::endl;
	if (rc == 0)
	{
		conn_opts.context = client;
		if ((rc = MQTTAsync_connect(client, &conn_opts)) != MQTTASYNC_SUCCESS)
		{
			std::cout << "   Failed to connect to MQTT, rc: " << rc << std::endl;
			rc = EXIT_FAILURE;
		}
	}
	//std::cout << "MQTTAsync connect done: " << rc << std::endl;

	return rc;
}

void loRa_command(std::string command)
{
	std::string payload = command + "\r\n";

	int len = payload.size();
	std::cout << "len: " << len << std::endl;
	std::vector<char> bytes(payload.begin(), payload.end());
	char *data = &bytes[0];

	lora.sendData(data, len);
	int n = lora.readData(32);

    std::cerr << "LoRa command sent:" << payload << std::endl;
    //for (int i = 0; i < len; i ++)
    //{
    //    std::stringstream ss;
    //    ss << std::hex << std::setw(2) << static_cast<char>(data[i]);
    //    std::cerr << ss.str();// << " ";
    //}
    //std::cerr << std::endl;

    std::cerr << "LoRa received status after command:" << std::endl;
	std::cerr << "   ===> n: " << n << std::endl;
    for (int i = 0; i < n; i ++)
    {
        std::stringstream ss;
        ss << std::hex << std::setw(2) << static_cast<char>(lora.rxArray[i]);
        std::cerr << ss.str();// << " ";
    }
	std::cerr << std::endl;
}

int lora_NETWORKID = 0;
int lora_ADDRESS = 0;
int lora_BAND = 923000000;

void start()
{
	ErrorNumber_e status;

	status = camera->setOperationMode(MODE_BEAM_A);
	if (status != ERROR_NUMMBER_NO_ERROR)
	{
		std::cerr << "Set OperationMode failed." << std::endl;
	}

	status = camera->setModulationFrequency(MODULATION_FREQUENCY_20MHZ);
	if (status != ERROR_NUMMBER_NO_ERROR)
	{
		std::cerr << "Set ModulationFrequency failed." << std::endl;
	}

	status = camera->setModulationChannel(0, 0);
	if (status != ERROR_NUMMBER_NO_ERROR)
	{
		std::cerr << "Set tModulationChannel failed." << std::endl;
	}

	camera->setAcquisitionMode(AUTO_REPEAT);
	if (status != ERROR_NUMMBER_NO_ERROR)
	{
		std::cerr << "Set AcquisitionMode failed." << std::endl;
	}

	status = camera->setOffset(0);
	if (status != ERROR_NUMMBER_NO_ERROR)
	{
		std::cerr << "Set Offset failed." << std::endl;
	}

	status = camera->setIntegrationTime3d(0, integrationTime0);
	if (status != ERROR_NUMMBER_NO_ERROR)
	{
		std::cerr << "Set IntegrationTime3d 0 failed." << std::endl;
	}
	status = camera->setIntegrationTime3d(1, integrationTime1);
	if (status != ERROR_NUMMBER_NO_ERROR)
	{
		std::cerr << "Set IntegrationTime3d 1 failed." << std::endl;
	}
	status = camera->setIntegrationTime3d(2, 0);
	if (status != ERROR_NUMMBER_NO_ERROR)
	{
		std::cerr << "Set IntegrationTime3d 2 failed." << std::endl;
	}
	status = camera->setIntegrationTime3d(3, 0);
	if (status != ERROR_NUMMBER_NO_ERROR)
	{
		std::cerr << "Set IntegrationTime3d 3 failed." << std::endl;
	}
	status = camera->setIntegrationTime3d(4, 0);
	if (status != ERROR_NUMMBER_NO_ERROR)
	{
		std::cerr << "Set IntegrationTime3d 4 failed." << std::endl;
	}
	status = camera->setIntegrationTime3d(5, 0);
	if (status != ERROR_NUMMBER_NO_ERROR)
	{
		std::cerr << "Set IntegrationTime3d 5 failed." << std::endl;
	}

	status = camera->setMinimalAmplitude(0, amplitude0);
	if (status != ERROR_NUMMBER_NO_ERROR)
	{
		std::cerr << "Set MinimalAmplitude 0 failed." << std::endl;
	}
	status = camera->setMinimalAmplitude(1, amplitude1);
	if (status != ERROR_NUMMBER_NO_ERROR)
	{
		std::cerr << "Set MinimalAmplitude 1 failed." << std::endl;
	}
	status = camera->setMinimalAmplitude(2, 0);
	if (status != ERROR_NUMMBER_NO_ERROR)
	{
		std::cerr << "Set MinimalAmplitude 2 failed." << std::endl;
	}
	status = camera->setMinimalAmplitude(3, 0);
	if (status != ERROR_NUMMBER_NO_ERROR)
	{
		std::cerr << "Set MinimalAmplitude 3 failed." << std::endl;
	}
	status = camera->setMinimalAmplitude(4, 0);
	if (status != ERROR_NUMMBER_NO_ERROR)
	{
		std::cerr << "Set MinimalAmplitude 4 failed." << std::endl;
	}
	status = camera->setMinimalAmplitude(5, 0);
	if (status != ERROR_NUMMBER_NO_ERROR)
	{
		std::cerr << "Set MinimalAmplitude 5 failed." << std::endl;
	}

	camera->setRange(50, 5500);

	HDR_e hdr_mode = HDR_OFF;
	if (hdr == 2)
	{
		hdr_mode = HDR_TEMPORAL;
	} else if (hdr == 1)
	{
		hdr_mode = HDR_SPATIAL;
	}

	status = camera->setHdr(hdr_mode);
	if (status != ERROR_NUMMBER_NO_ERROR)
	{
		std::cerr << "Set HDR failed." << std::endl;
	}

	std::cout << "integrationTime0: " << integrationTime0 << std::endl;
	std::cout << "integrationTime1: " << integrationTime1 << std::endl;
	std::cout << "amplitude:        " << amplitude0 << std::endl;
	std::cout << "HDR:              " << hdr_mode << std::endl;
	std::cout << "\n" << std::endl;

	if (strcmp(action, "record") == 0 || strcmp(action, "train-record") == 0)
	{
		std::thread th_record_data(&record_data);
		th_record_data.detach();
	}

	std::thread th_detect_and_track(&detect_and_track);
	th_detect_and_track.detach();

	std::thread th_pre_cluster_cloud(&pre_cluster_cloud);
	th_pre_cluster_cloud.detach();

	std::thread th_pre_filter_cloud(&pre_filter_cloud);
	th_pre_filter_cloud.detach();

	process(camera);

	delete camera;
}

int init_db()
{
	std::string db_f_path = std::string(sd_card) + "/peoplecount.db";
	bool db_exists = false;

	if (file_exists(db_f_path))
	{
		db_exists = true;
	}

	int rc = sqlite3_open(db_f_path.c_str(), &db);

	if( rc ) {
		fprintf(stderr, "Can't open database: %s\n", sqlite3_errmsg(db));
		return 1;
	} else {
		fprintf(stderr, "Opened database successfully\n");
	}

	if (! db_exists)
	{
		char *zErrMsg = 0;
		const char *sql = "CREATE TABLE PEOPLECOUNT("  \
		"DEVICEID       INT     NOT NULL," \
		"ENTER          INT     NOT NULL," \
		"EXIT           INT     NOT NULL," \
		"DATETIME       INT PRIMARY KEY);";

		rc = sqlite3_exec(db, sql, [](void *NotUsed, int argc, char **argv, char **azColName){
			int i;
			for(i = 0; i<argc; i++) {
				printf("%s = %s\n", azColName[i], argv[i] ? argv[i] : "NULL");
			}
			printf("\n");
			return 0;
		}, 0, &zErrMsg);

		if( rc != SQLITE_OK ){
			fprintf(stderr, "SQL error: %s\n", zErrMsg);
			sqlite3_free(zErrMsg);
		} else {
			fprintf(stdout, "Table created successfully\n");
		}
	}

	return 0;
}

int conn_lora()
{
	std::string loraCommand;

	loraCommand = "AT+NETWORKID=" + std::to_string(lora_NETWORKID);
	loRa_command(loraCommand);
	usleep(1000000);

	loraCommand = "AT+ADDRESS=" + std::to_string(lora_ADDRESS);
	loRa_command(loraCommand);
	usleep(1000000);

	loraCommand = "AT+BAND=" + std::to_string(lora_BAND);
	loRa_command(loraCommand);
	usleep(1000000);

	loraCommand = "AT+NETWORKID=?";
	loRa_command(loraCommand);
	usleep(1000000);

	loraCommand = "AT+ADDRESS?";
	loRa_command(loraCommand);
	usleep(1000000);

	loraCommand = "AT+BAND?";
	loRa_command(loraCommand);
	usleep(1000000);

	loraCommand = "AT+PARAMETER?";
	loRa_command(loraCommand);
	usleep(1000000);

	return 0;
}
int conn_sub_mqtt()
{
	int mqtt_ok = -1;

	//std::cout << "Connecting to MQTT broker: " << serverURI << ", clientId: " << clientId << std::endl;
	std::cout << "Connecting to MQTT: " << std::endl;
	while ((! exit_requested) && (mqtt_ok != 0))
	{
		usleep(1000000);
		mqtt_ok = connectToMQTT();
		if (led_available)
		{
			gpio_high(LED_RED);
		}

		if (mqtt_ok == 0)
		{
			std::cout << "      conn succeeded, wait for callback ... " << std::endl;
			int count = 0;
			while (! mqtt_connected)
			{
				usleep(1000000);
				if (led_available)
				{
					gpio_low(LED_RED);
				}

				if (count > 5)
				{
					// no callback in 5 s, give up
					std::cout << "      conn no callback in 5 s, give up and try again ... " << std::endl;
					mqtt_ok = -1;
					break;
				}
				count ++;
			}
		} else
		{
			if (led_available)
			{
				gpio_low(LED_RED);
			}

			std::cout << "Waiting for network connection ..." << std::endl;
			usleep(5000000);
		}
	}
	if (led_available)
	{
		gpio_high(LED_RED);
	}
	usleep(1000000);

	// mqtt_subscribed
	int count = 0;
	while (! mqtt_subscribed)
	{
		gpio_high(LED_RED);
		usleep(1000000);
		if (led_available)
		{
			gpio_low(LED_RED);
			usleep(1000000);
		}

		if (count > 30)
		{
			// no callback in 5 s, give up
			std::cout << "      sub no callback in 5 s, give up ... " << std::endl;
			mqtt_ok = -1;
			break;
		}
		count ++;
	}

	return mqtt_ok;
}

void exit_handler(int s){
	std::cout << "\nExiting ... " << std::endl;
	exit_requested = true;
	fflush(stdout);
	usleep(2000000);
	signal(SIGINT, exit_handler);
	//exit(0);
}

int main(int argc, char** argv) {

	struct sigaction sigIntHandler;
	sigIntHandler.sa_handler = exit_handler;
	sigemptyset(&sigIntHandler.sa_mask);
	sigIntHandler.sa_flags = 0;
	sigaction(SIGINT, &sigIntHandler, NULL);

	bool _dev = true;

	if (file_exists("/home/cat/certs"))
	{
		ca_file     = "/home/cat/certs/root-CA.crt";
		client_cert = "/home/cat/certs/mqtt5.certificate.pem";
		client_key  = "/home/cat/certs/mqtt5.private.key";

		gpio_init(LED_RED);
		gpio_init(LED_GREEN);

		gpio_high(LED_RED);
		gpio_high(LED_GREEN);

		led_available = true;

		otg_ip_address = "10.42.0.1";

		_dev = false;
	}

	if ((homedir = getenv("HOME")) == NULL) {
		homedir = getpwuid(getuid())->pw_dir;
	}

	if (argc >= 8)
	{
		action = argv[1];
		char* integrationTime0_str = argv[2];
		char* integrationTime1_str = argv[3];
		char* amplitude_str = argv[4];
		char* hdr_str = argv[5];

		integrationTime0 = std::stoi( integrationTime0_str );
		integrationTime1 = std::stoi( integrationTime1_str );
		amplitude0 = std::stoi( amplitude_str );
		amplitude1 = amplitude0;

		hdr = std::stoi( hdr_str );

		std::cout << "Mode:            " << action << std::endl;

		if (argc >= 7)
		{
			sd_card = argv[6];
			std::cout << "Data storage:    " << sd_card << std::endl;
		}

		if (argc >= 8)
		{
			comm_protocal = argv[7];
			std::cout << "Comm protocal:   " << comm_protocal << std::endl;
		}

		if (strcmp(comm_protocal, "mqtt") == 0)
		{
			// for mqtt
			if (argc >= 9)
			{
				serverURI = argv[8];
			}
			if (argc >= 10)
			{
				ca_file = argv[9];
			}
			if (argc >= 12)
			{
				clientId = argv[10];
				clientPass = argv[11];
			}
			std::cout << "   MQTT URI:     " << serverURI << std::endl;
			std::cout << "   MQTT ca:      " << ca_file << std::endl;
			std::cout << "   MQTT client:  " << clientId << std::endl;

		} else if (strcmp(comm_protocal, "lora") == 0) {
			// for lora
			if (argc >= 9)
			{
				lora_NETWORKID = std::stoi(argv[8]);
			}
			if (argc >= 10)
			{
				lora_ADDRESS = std::stoi(argv[9]);
			}
			if (argc >= 11)
			{
				lora_BAND = std::stoi(argv[10]);
			}
			std::cout << "   NETWORKID:    " << lora_NETWORKID << std::endl;
			std::cout << "   ADDRESS:      " << lora_ADDRESS << std::endl;
			std::cout << "   BAND:         " << lora_BAND << std::endl;
		}

	} else if (argc == 6 && strcmp(argv[1], "view") == 0)
	{
		action = argv[1];
		char* integrationTime0_str = argv[2];
		char* integrationTime1_str = argv[3];
		char* amplitude_str = argv[4];
		char* hdr_str = argv[5];

		integrationTime0 = std::stoi( integrationTime0_str );
		integrationTime1 = std::stoi( integrationTime1_str );
		amplitude0 = std::stoi( amplitude_str );
		amplitude1 = amplitude0;

		hdr = std::stoi( hdr_str );

		std::cout << "Mode:            " << action << std::endl;
	} else {
		std::cout << "Usage: peoplecount [train | detect | record || train-detect | train-record | view] <integrationTime0> <integrationTime1> <amplitude> <HDR> <Data Storage> [lora | mqtt] [MQTT Broker] [MQTT Client ID]" << std::endl;
		std::cout << "Option for lora | mqtt:" << std::endl;
		std::cout << "   ... lora <NETWORKID> <ADDRESS> [BAND]" << std::endl;
		std::cout << "   ... mqtt [MQTT Broker] [MQTT Client ID]" << std::endl;
		std::cout << "Example for train:        ./peoplecount train 1000 100 20 2 /home/cat/data mqtt" << std::endl;
		std::cout << "Example for detect:       ./peoplecount detect 1000 100 20 2 /home/cat/data mqtt" << std::endl;
		std::cout << "Example for record:       ./peoplecount record 1000 100 20 2 /home/cat/data mqtt" << std::endl;
		std::cout << "Example for train-detect: ./peoplecount train-detect 1000 100 20 2 /home/cat/data mqtt" << std::endl;
		std::cout << "Example for train-record: ./peoplecount train-record 1000 100 20 2 /home/cat/data mqtt" << std::endl;
		std::cout << "Example for train-detect: ./peoplecount train-detect 1000 100 20 2 /home/cat/data lora 2 2" << std::endl;
		std::cout << "Example for train-detect: ./peoplecount train-record 1000 100 20 2 /home/cat/data lora 2 2" << std::endl;
		std::cout << "Example for view:         ./peoplecount view 1000 100 20 2" << std::endl;
		return 1;
	}
	if (_dev)
	{
		sd_card = (char*) "/home/vsemi/data/peoplecount/test";
	}
	bool result_save_locally = false;
	if (strcmp(action, "train") == 0 || strcmp(action, "detect") == 0 || strcmp(action, "record") == 0 || strcmp(action, "train-detect") == 0 || strcmp(action, "train-record") == 0)
	{
		if (file_exists(sd_card))
		{
			//std::cout << "Data Storage: " << sd_card << std::endl;
			result_save_locally = true;
		} else
		{
			std::cout << "No Data Storage found, please insert a Data Storage. " << std::endl;
			return 2;
		}
	}

	data_background = new float[9600];
	if (strcmp(action, "detect") == 0 || strcmp(action, "record") == 0)
	{
		std::string background_file_path = std::string(sd_card) + "/model.bin";
		if (file_exists(background_file_path))
		{
			read(background_file_path, data_background);
		} else
		{
			std::cout << "No background model found, please train background first: " << std::endl;
			std::cout << "Usage: peoplecount train <integrationTime0> <integrationTime1> <amplitude> <HDR> <Data Storage>" << std::endl;
			std::cout << "Example for train:  ./peoplecount train 1000 100 20 2 /home/cat/data" << std::endl;
			return 3;
		}
	}
	if (result_save_locally)
	{
		if (init_db() != 0)
		{
			return -1;
		}
	}

	usleep(2000000);
	if (led_available)
	{
		gpio_low(LED_RED);
		gpio_low(LED_GREEN);
	}

	bool tof_ok = false;
	std::cout << "Connect to ToF sensor ... " << std::endl;
	while ((! exit_requested) && (! tof_ok))
	{
		if (led_available)
		{
			usleep(1000000);
			gpio_high(LED_RED);
		}
		camera = Camera::usb_tof_camera_160("/dev/ttyACM0");
		tof_ok = camera->open();

		if (! tof_ok)
		{
			usleep(1000000);
			if (led_available)
			{
				gpio_low(LED_RED);
			}
			std::cout << "Opening ToF sensor ..." << std::endl;
		} else
		{
			sensor_uid = camera->getID();

			clientId = "VSEMI_" + std::to_string(sensor_uid);
		}
	}
	usleep(2000000);
	if (led_available)
	{
		gpio_low(LED_RED);
	}

	std::cout << "\n" << std::endl;
	std::cout << "--------------------------------------" << std::endl;
	std::cout << "Device ID:         " << sensor_uid << std::endl;
	std::cout << "--------------------------------------" << std::endl;
	std::cout << "\n" << std::endl;

	bool otg_connected = has_ip_address(otg_ip_address);
	std::cout << "otg connected ..." << otg_connected << std::endl;

	if (otg_connected)
	{
		action = (char*)"view";
	}

	if (strcmp(action, "view") == 0)
	{
		streamer = new MJPGStreamer();
		streamer->start(8800);

		if (led_available)
		{
			gpio_high(LED_RED);
			gpio_high(LED_GREEN);
		}

		integrationTime0 = 400;

		std::cout << "\nStarting view mode, point browser to http://10.42.0.1:8800 to view ToF distance iamge.\n" << std::endl;
	} else
	{
		if (strcmp(comm_protocal, "lora") == 0)
		{
			lora_available = lora.openConnection("/dev/ttyS8");
			std::cout << "Opening LoRa ..." << lora_available << std::endl;

			if (! lora_available) {
				return 4;
			} else
			{
				conn_lora();
			}
		}

		// conn and sub to mqtt
		if (strcmp(comm_protocal, "mqtt") == 0)
		{
			conn_sub_mqtt();
		}
	}

	start();

	if (strcmp(action, "view") == 0)
	{
		std::cout << "To stop streamer ..." << std::endl;
		streamer->stop();
		std::cout << "streamer stopped!" << std::endl;

		delete streamer;
		std::cout << "streamer deleted!" << std::endl;
	}

	sqlite3_close(db);
	std::cout << "DB closed!" << std::endl;

    if (led_available)
	{
		gpio_deinit(LED_RED);
        gpio_deinit(LED_GREEN);
	}

	return 0;
}
