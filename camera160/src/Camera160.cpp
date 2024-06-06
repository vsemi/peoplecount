#include <Camera160.hpp>

#include<iostream>
#include <limits>
#include <algorithm>
#include <sys/stat.h>
#include <dirent.h>

#include "SerialConnection.hpp"
#include "constants.h"
#include "util.h"

#pragma pack(push,1)
typedef struct
{
    uint8_t grayscale;
    uint16_t distance;
}distanceGrayscale_t;
#pragma pack(pop)

inline bool file_exists (const std::string& name) {
	struct stat buffer;
	return (stat (name.c_str(), &buffer) == 0);
}

Camera160::Camera160(std::string port)
{
	_port = port;
	imageColorizer = new ImageColorizer();

	setWidth(160, 60);

	n_points = getWidth() * getHeight();

	n_data_points = n_points;

	acquisitionMode = AUTO_REPEAT;
	opMode          = MODE_BEAM_A;
	hdr             = HDR_OFF;

	d3X = new double[n_points];
	d3Y = new double[n_points];
	d3Z = new double[n_points];

	this->data_background = new float[n_points];
	this->removeBackground = false;

	valid_mask = new bool[n_points];

	setFoV(50.0, 18.75);
}

Camera160::~Camera160() throw ()
{
	delete communication;
	delete[] d3X;
	delete[] d3Y;
	delete[] d3Z;
	delete imageColorizer;
	delete data_background;
}

std::string Camera160::getPort()
{
	return _port;
}

void Camera160::setFoV(double angle_x, double angle_y)
{
	_angle_x = angle_x;
	_angle_y = angle_y;

	initD3F(d3X, d3Y, d3Z, getWidth(), getHeight(), _angle_x, _angle_y);
}

void Camera160::setBackground(float *background, bool removeBackground)
{
	std::memcpy(this->data_background, background, n_points * sizeof(float));
	this->removeBackground = removeBackground;
	//for (int i = 0; i < n_points; i ++)
	//{
	//	std::cout << "after set ->data_background: " << this->data_background[i] << std::endl;
	//}
}

void Camera160::startStream()
{
	communication->startStream();
}

/**
 * @bstop streaming
 *
 * @return Error code from ErrorNumber_e
 */
ErrorNumber_e Camera160::stopStream()
{
	return communication->sendCommandWithoutData(Constants::CommandList::COMMAND_STOP_STREAM, false);
}

bool Camera160::open()
{
	if (! file_exists(_port))
	{
		//std::cerr << "Error: ToF camera is not available at port: " << _port << std::endl;
		return false;
	}

	communication = new Communication(new SerialConnection(_port));
	//std::cout << "Open camera at port: " << _port << std::endl;

	bool status = false;
	int attempts = 30;
	ErrorNumber_e errorNumber;
	while (attempts > 0)
	{
		attempts --;

		status = communication->open();

		if (status)
		{
			uint16_t chipId, waferId;
			errorNumber = getChipInformation(chipId, waferId);

			if (errorNumber == ERROR_NUMMBER_NO_ERROR)
			{
				setID(chipId, waferId);
				break;
			} else
			{
				communication->close();

			}

		} else
		{
			communication->close();
			break;
		}
#ifdef _WIN32
		Sleep(3000);
#else
		//usleep(5000000);
		usleep(1000000);
#endif
	}
	if (status)
	{
		if (errorNumber != ERROR_NUMMBER_NO_ERROR)
		{
			std::cerr << "Identification could not be read." << std::endl;
		}
	}

	return status;
}

/**
 * @brief Request the chip information
 *
 * @param chipId Reference to the variable to write the chip id
 * @param waferId Reference to the variable to write the wafer id
 * @return Error code from ErrorNumber_e
 */
ErrorNumber_e Camera160::getChipInformation(uint16_t &chipId, uint16_t &waferId)
{
	ErrorNumber_e status = communication->sendCommandWithoutData(Constants::CommandList::COMMAND_GET_CHIP_INFORMATION, GET_CHIP_INFORMATION_SIZE);

	if (status == ERROR_NUMMBER_NO_ERROR)
	{
		std::vector<uint8_t> data_array = communication->data();

		waferId = static_cast<uint16_t>(getUint16LittleEndian(data_array, Constants::ChipInformation::INDEX_WAFER_ID - Constants::Data::SIZE_HEADER));
		chipId  = static_cast<uint16_t>(getUint16LittleEndian(data_array, Constants::ChipInformation::INDEX_CHIP_ID - Constants::Data::SIZE_HEADER));
	}

	return status;
}

/**
 * @brief Request the identification
 *
 * This identification helps to detect, which device is connected to the PC and if the bootloader
 * is active or the application. When the bootloader is active, the flag "isBootloader" is set.
 * In addition the version of the device is read. This is the coast version and has nothing to do with
 * the firmware version.
 *
 * @param device Reference where the device is written to
 * @param isBootloader Reference to the bootloader flag
 * @param version Reference where the version is written to
 * @return Error code from ErrorNumber_e
 */
ErrorNumber_e Camera160::getIdentification(unsigned int &device, unsigned int &version)
{
	ErrorNumber_e status = communication->sendCommandWithoutData(Constants::CommandList::COMMAND_IDENTIFY, GET_IDENTIFY_SIZE);

	if (status == ERROR_NUMMBER_NO_ERROR)
	{
		std::vector<uint8_t> data_array = communication->data();

		uint32_t identificationValue = getUint32LittleEndian(data_array, 0);

		device = (identificationValue & Constants::Identification::MASK_CHIP_TYPE_DEVICE) >> Constants::Identification::SHIFT_CHIP_TYPE_DEVICE;
		version = (identificationValue & Constants::Identification::MASK_VERSION) >> Constants::Identification::SHIFT_VERSION;
	}

	return status;
}

/**
 * @brief Request the firmware rlease
 *
 * @param major Reference to write the major number
 * @param minor Reference to write the minor number
 * @return Error code from ErrorNumber_e
 */
ErrorNumber_e Camera160::getFirmwareRelease(unsigned int &major, unsigned int &minor)
{
	ErrorNumber_e status = communication->sendCommandWithoutData(Constants::CommandList::COMMAND_GET_FIRMWARE_RELEASE, GET_FIRMWARE_VERSION_SIZE);

	if (status == ERROR_NUMMBER_NO_ERROR)
	{
		uint32_t firmwareRelease = getUint32LittleEndian(communication->data(), 0);

		major = getValueMsb(firmwareRelease);
		minor = getValueLsb(firmwareRelease);
	}

	return status;
}

/**
 * @brief Request the temperature
 *
 * This function will be answered by the signal "receivedTemperature"
 */
ErrorNumber_e Camera160::getTemperature(int16_t &temperature)
{
	ErrorNumber_e status = communication->sendCommandWithoutData(Constants::CommandList::COMMAND_GET_TEMPERATURE, GET_TEMPERATURE_SIZE);

	if (status == ERROR_NUMMBER_NO_ERROR)
	{
		temperature = static_cast<int16_t>(getInt16LittleEndian(communication->data(), 0));
	}

	return status;
}

/**
 * @brief Request distance
 *
 * This function will be answered by the signal "receivedDistance"
 *
 * @param acquisitionMode Mode for acquisition: 0 = single, 1 = auto repeat, 3 = stream
 */
ErrorNumber_e Camera160::getDistance(ToFImage &tofImage)
{
	int dataSize;
	bool streaming = false;
	if(acquisitionMode == STREAM) streaming = true;

	if(hdr == HDR_SPATIAL){
		dataSize = static_cast<int>(n_data_points + 88); //16 bit distance
	}else{
		dataSize = static_cast<int>(2 * n_data_points + 88); //16 bit distance
	}

	ErrorNumber_e status;
	bool hdrTemporal = (hdr == HDR_TEMPORAL);
	
	std::map<unsigned int, std::vector<uint8_t> > map_data_array;

	unsigned int total_filled = 0;
	unsigned int total_accquired = 0;
	integrationTime3ds.clear();
	do
	{
		total_filled = 0;

		status = communication->sendCommandSingleByte(Constants::CommandList::COMMAND_GET_DISTANCE, static_cast<uint8_t>(acquisitionMode), dataSize, streaming);
		total_accquired ++;

		if (status != ERROR_NUMMBER_NO_ERROR)
		{
			return ERROR_NUMBER_INVALID_DATA;
		}
		std::vector<uint8_t> data_array = communication->data();
		if (data_array.size() == 0)
		{
			break;
		}
		
		unsigned int currentIntegrationTime3dWf = getUint16LittleEndian(data_array, C160::ImageHeader::INDEX_CURRENT_INTEGRATION_TIME_3D_WF);
		if (hdrTemporal)
		{

			for (unsigned int i = 0; i < C160::ImageHeader::NUM_INTEGRATION_TIME_3D; i++)
			{
				unsigned int integrationTime3d = getUint16LittleEndian(data_array, C160::ImageHeader::INDEX_INTEGRATION_TIME_0 + (i * sizeof(uint16_t)));

				if (integrationTime3d > 0)
				{
					std::map<unsigned int, bool>::iterator it = integrationTime3ds.find(integrationTime3d);
					if (it != integrationTime3ds.end())
					{
						if (currentIntegrationTime3dWf == integrationTime3d) it->second = true;
					} else
					{
						integrationTime3ds[integrationTime3d] = (currentIntegrationTime3dWf == integrationTime3d);
					}
				}
			}

			for(std::map<unsigned int, bool>::iterator it = integrationTime3ds.begin(); it != integrationTime3ds.end(); ++it)
			{
				unsigned int filled = static_cast<unsigned int>(it->second);
				total_filled += filled;
			}
		}
		
		map_data_array[currentIntegrationTime3dWf] = data_array;
		
		if (hdrTemporal)
		{
			if (total_filled >= integrationTime3ds.size())
			{
				break;
			}
		}
	} while(hdrTemporal && total_accquired < C160::ImageHeader::NUM_INTEGRATION_TIME_3D && total_filled < integrationTime3ds.size());

	int frame_index = 0;

	uint16_t image_width = 0;
	uint16_t image_height= 0;

	uint16_t leftX = 0;
	uint16_t topY = 0;
	uint16_t rightX  = 0;
	uint16_t bottomY = 0;
	for(std::map<unsigned int, std::vector<uint8_t> >::iterator it = map_data_array.begin(); it != map_data_array.end(); ++it)
	{
		std::vector<uint8_t> data_array = it->second;

		if (frame_index == 0)
		{
			image_width = static_cast<uint16_t>(getUint16LittleEndian(data_array, C160::ImageHeader::INDEX_WIDTH));
			image_height = static_cast<uint16_t>(getUint16LittleEndian(data_array, C160::ImageHeader::INDEX_HEIGHT));

			leftX = static_cast<uint16_t>(getUint16LittleEndian(data_array, C160::ImageHeader::INDEX_ORIGIN_X));
			topY = static_cast<uint16_t>(getUint16LittleEndian(data_array, C160::ImageHeader::INDEX_ORIGIN_Y));
			rightX  = image_width  + leftX;
			bottomY = image_height + topY;
		}

		uint16_t *distance =  (uint16_t *)(&data_array.data()[C160::ImageHeader::SIZE]);

		unsigned int val;
		int i_data_point = 0, i_2d_color = 0, i_point_3d = 0, i_out_index = 0, p_index = 0;
		Color c;
		int rgb;
		uint8_t r;
		uint8_t g;
		uint8_t b;
		bool valid_point;

		for(int y = 0; y < 60; y++)
		{
			for(int x = 0; x < 160; x++)
			{
				if (x < leftX || x >= rightX || y < topY || y >= bottomY)
				{
					val = C160::Pixel::LIMIT_VALID_PIXEL;
				} else {
					val = (distance[i_data_point] & C160::Pixel::MASK_OUT_CONFIDENCE);
					i_data_point ++;
				}

				if (frame_index == 0) valid_mask[i_out_index] = false;

				if ((! hdrTemporal) || frame_index == 0)
				{
					tofImage.data_grayscale[i_out_index] = 0;
					tofImage.data_amplitude[i_out_index] = 0;
				}

				if ((! hdrTemporal) || frame_index == 0 || (! valid_mask[i_out_index]))
				{
					c = imageColorizer->getColor(val);

					r = c.r;
					g = c.g;
					b = c.b;

					valid_point = false;
					if(val < C160::Pixel::LIMIT_VALID_PIXEL)
					{
						rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);

						p_index = i_out_index;
						if (hdr == HDR_SPATIAL)
						{
							p_index = 320 * y  + x;
						}

						cal3DXYZ(p_index, getWidth(), getHeight(), val, d3X, d3Y, d3Z, X, Y, Z);

						tofImage.saturated_mask[i_out_index]     = 0;

						tofImage.data_depth[i_out_index]         = Z;

						tofImage.data_2d_bgr[i_2d_color]         = b;
						tofImage.data_2d_bgr[i_2d_color + 1]     = g;
						tofImage.data_2d_bgr[i_2d_color + 2]     = r;

						//std::cout << i_out_index << " z: " << Z << " data_background[i]: " << data_background[i_out_index] << std::endl;
						//if ((! removeBackground) || (removeBackground && Z < (data_background[i_out_index] - 0.05)))
						if ((! removeBackground) || (removeBackground && Z < (this->data_background[i_out_index] - 0.5)))
						{
							//std::cout << "valid -----------" << i_out_index << " z: " << Z << " data_background[i]: " << this->data_background[i_out_index] << std::endl;
							tofImage.data_3d_xyz_rgb[i_point_3d]     = X;
							tofImage.data_3d_xyz_rgb[i_point_3d + 1] = Y;
							tofImage.data_3d_xyz_rgb[i_point_3d + 2] = Z;
							tofImage.data_3d_xyz_rgb[i_point_3d + 3] = 0.0f;
							tofImage.data_3d_xyz_rgb[i_point_3d + 4] = *reinterpret_cast<float*>(&rgb);
							tofImage.data_3d_xyz_rgb[i_point_3d + 5] = *reinterpret_cast<float*>(&r); //r;
							tofImage.data_3d_xyz_rgb[i_point_3d + 6] = *reinterpret_cast<float*>(&g); //g;
							tofImage.data_3d_xyz_rgb[i_point_3d + 7] = *reinterpret_cast<float*>(&b); //b;

							valid_point = true;
						} else{
							valid_point = false;
						}

						
						valid_mask[i_out_index] = true;

					} else if (val == C160::Pixel::VALUE_ADC_OVERFLOW || val == C160::Pixel::VALUE_SATURATION)
					{
						tofImage.saturated_mask[i_out_index]     = 255;

						tofImage.data_2d_bgr[i_2d_color]         = b;
						tofImage.data_2d_bgr[i_2d_color + 1]     = g;
						tofImage.data_2d_bgr[i_2d_color + 2]     = r;
					} else
					{
						tofImage.saturated_mask[i_out_index]     = 0;

						tofImage.data_2d_bgr[i_2d_color]         = std::numeric_limits<uint8_t>::quiet_NaN();
						tofImage.data_2d_bgr[i_2d_color + 1]     = std::numeric_limits<uint8_t>::quiet_NaN();
						tofImage.data_2d_bgr[i_2d_color + 2]     = std::numeric_limits<uint8_t>::quiet_NaN();
					}
					if (! valid_point)
					{
						tofImage.data_depth[i_out_index]         = std::numeric_limits<float>::quiet_NaN();

						tofImage.data_3d_xyz_rgb[i_point_3d]     = std::numeric_limits<float>::quiet_NaN();
						tofImage.data_3d_xyz_rgb[i_point_3d + 1] = std::numeric_limits<float>::quiet_NaN();
						tofImage.data_3d_xyz_rgb[i_point_3d + 2] = std::numeric_limits<float>::quiet_NaN();
						tofImage.data_3d_xyz_rgb[i_point_3d + 3] = std::numeric_limits<float>::quiet_NaN();
						tofImage.data_3d_xyz_rgb[i_point_3d + 4] = std::numeric_limits<float>::quiet_NaN();
						tofImage.data_3d_xyz_rgb[i_point_3d + 5] = std::numeric_limits<float>::quiet_NaN();
						tofImage.data_3d_xyz_rgb[i_point_3d + 6] = std::numeric_limits<float>::quiet_NaN();
						tofImage.data_3d_xyz_rgb[i_point_3d + 7] = std::numeric_limits<float>::quiet_NaN();
					}
				}

				i_point_3d += 8;
				i_2d_color += 3;
				i_out_index ++;
			}
		}
		frame_index ++;
	}

	return status;
}

ErrorNumber_e Camera160::getDistanceGrayscale(ToFImage &tofImage)
{
	int dataSize;
	bool streaming = false;
	if(acquisitionMode == STREAM) streaming = true;

	if(hdr == HDR_SPATIAL)  dataSize = static_cast<int>(3 * n_data_points / 2 + 88); //16 bit distance/2 + 8 bit grayscale/2
	else dataSize = static_cast<int>(3 * n_data_points + 88); //16 bit distance + 8 bit grayscale;

	ErrorNumber_e status;
	bool hdrTemporal = (hdr == HDR_TEMPORAL);
	
	std::map<unsigned int, std::vector<uint8_t> > map_data_array;

	unsigned int total_filled = 0;
	unsigned int total_accquired = 0;
	integrationTime3ds.clear();
	do
	{
		total_filled = 0;

		status = communication->sendCommandSingleByte(Constants::CommandList::COMMAND_GET_DISTANCE_GRAYSCALE, static_cast<uint8_t>(acquisitionMode), dataSize, streaming);
		total_accquired ++;

		if (status != ERROR_NUMMBER_NO_ERROR)
		{
			return ERROR_NUMBER_INVALID_DATA;
		}
		std::vector<uint8_t> data_array = communication->data();
		if (data_array.size() == 0)
		{
			break;
		}
		
		unsigned int currentIntegrationTime3dWf = getUint16LittleEndian(data_array, C160::ImageHeader::INDEX_CURRENT_INTEGRATION_TIME_3D_WF);
		if (hdrTemporal)
		{

			for (unsigned int i = 0; i < C160::ImageHeader::NUM_INTEGRATION_TIME_3D; i++)
			{
				unsigned int integrationTime3d = getUint16LittleEndian(data_array, C160::ImageHeader::INDEX_INTEGRATION_TIME_0 + (i * sizeof(uint16_t)));

				if (integrationTime3d > 0)
				{
					std::map<unsigned int, bool>::iterator it = integrationTime3ds.find(integrationTime3d);
					if (it != integrationTime3ds.end())
					{
						if (currentIntegrationTime3dWf == integrationTime3d) it->second = true;
					} else
					{
						//integrationTime3ds.insert(std::pair<unsigned int, bool>(integrationTime3d,currentIntegrationTime3dWf == integrationTime3d));
						integrationTime3ds[integrationTime3d] = (currentIntegrationTime3dWf == integrationTime3d);
					}
				}
			}

			for(std::map<unsigned int, bool>::iterator it = integrationTime3ds.begin(); it != integrationTime3ds.end(); ++it)
			{
				unsigned int filled = static_cast<unsigned int>(it->second);
				total_filled += filled;
			}
		}
		
		map_data_array[currentIntegrationTime3dWf] = data_array;
		
		if (hdrTemporal)
		{
			if (total_filled >= integrationTime3ds.size())
			{
				break;
			}
		}
	} while(hdrTemporal && total_accquired < C160::ImageHeader::NUM_INTEGRATION_TIME_3D && total_filled < integrationTime3ds.size());

	int frame_index = 0;

	uint16_t image_width = 0;
	uint16_t image_height= 0;

	uint16_t leftX = 0;
	uint16_t topY = 0;
	uint16_t rightX  = 0;
	uint16_t bottomY = 0;
	for(std::map<unsigned int, std::vector<uint8_t> >::iterator it = map_data_array.begin(); it != map_data_array.end(); ++it)
	{
		std::vector<uint8_t> data_array = it->second;

		if (frame_index == 0)
		{
			image_width = static_cast<uint16_t>(getUint16LittleEndian(data_array, C160::ImageHeader::INDEX_WIDTH));
			image_height = static_cast<uint16_t>(getUint16LittleEndian(data_array, C160::ImageHeader::INDEX_HEIGHT));

			leftX = static_cast<uint16_t>(getUint16LittleEndian(data_array, C160::ImageHeader::INDEX_ORIGIN_X));
			topY = static_cast<uint16_t>(getUint16LittleEndian(data_array, C160::ImageHeader::INDEX_ORIGIN_Y));
			rightX  = image_width  + leftX;
			bottomY = image_height + topY;
		}

		distanceGrayscale_t *distanceGrayscale =  (distanceGrayscale_t *)(&data_array.data()[C160::ImageHeader::SIZE]);

		unsigned int val;
		int i_data_point = 0, i_2d_color = 0, i_point_3d = 0, i_out_index = 0, p_index = 0;
		Color c;
		int rgb;
		uint8_t r;
		uint8_t g;
		uint8_t b;
		bool valid_point;

		for(int y = 0; y < 60; y++)
		{
			for(int x = 0; x < 160; x++)
			{
				if (x < leftX || x >= rightX || y < topY || y >= bottomY)
				{
					val = C160::Pixel::LIMIT_VALID_PIXEL;
				} else {
					distanceGrayscale_t p = distanceGrayscale[i_data_point];
					val = p.distance & C160::Pixel::MASK_OUT_CONFIDENCE;
					if ((! hdrTemporal) || frame_index == 0)
					{
						tofImage.data_grayscale[i_out_index] = p.grayscale;
					}
					i_data_point ++;
				}

				if (frame_index == 0) valid_mask[i_out_index] = false;

				if ((! hdrTemporal) || frame_index == 0)
				{
					tofImage.data_amplitude[i_out_index] = 0;
				}

				if ((! hdrTemporal) || frame_index == 0 || (! valid_mask[i_out_index]))
				{
					c = imageColorizer->getColor(val);

					r = c.r;
					g = c.g;
					b = c.b;

					valid_point = false;
					if(val < C160::Pixel::LIMIT_VALID_PIXEL)
					{
						rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);

						p_index = i_out_index;
						if (hdr == HDR_SPATIAL)
						{
							p_index = 320 * y  + x;
						}

						cal3DXYZ(p_index, getWidth(), getHeight(), val, d3X, d3Y, d3Z, X, Y, Z);

						tofImage.saturated_mask[i_out_index]     = 0;

						tofImage.data_depth[i_out_index]         = Z;

						tofImage.data_2d_bgr[i_2d_color]         = b;
						tofImage.data_2d_bgr[i_2d_color + 1]     = g;
						tofImage.data_2d_bgr[i_2d_color + 2]     = r;

						tofImage.data_3d_xyz_rgb[i_point_3d]     = X;
						tofImage.data_3d_xyz_rgb[i_point_3d + 1] = Y;
						tofImage.data_3d_xyz_rgb[i_point_3d + 2] = Z;
						tofImage.data_3d_xyz_rgb[i_point_3d + 3] = 0.0f;
						tofImage.data_3d_xyz_rgb[i_point_3d + 4] = *reinterpret_cast<float*>(&rgb);
						tofImage.data_3d_xyz_rgb[i_point_3d + 5] = *reinterpret_cast<float*>(&r); //r;
						tofImage.data_3d_xyz_rgb[i_point_3d + 6] = *reinterpret_cast<float*>(&g); //g;
						tofImage.data_3d_xyz_rgb[i_point_3d + 7] = *reinterpret_cast<float*>(&b); //b;

						valid_point = true;
						valid_mask[i_out_index] = true;

					} else if (val == C160::Pixel::VALUE_ADC_OVERFLOW || val == C160::Pixel::VALUE_SATURATION)
					{
						tofImage.saturated_mask[i_out_index]     = 255;

						tofImage.data_2d_bgr[i_2d_color]         = b;
						tofImage.data_2d_bgr[i_2d_color + 1]     = g;
						tofImage.data_2d_bgr[i_2d_color + 2]     = r;
					} else
					{
						tofImage.saturated_mask[i_out_index]     = 0;

						tofImage.data_2d_bgr[i_2d_color]         = std::numeric_limits<uint8_t>::quiet_NaN();
						tofImage.data_2d_bgr[i_2d_color + 1]     = std::numeric_limits<uint8_t>::quiet_NaN();
						tofImage.data_2d_bgr[i_2d_color + 2]     = std::numeric_limits<uint8_t>::quiet_NaN();
					}
					if (! valid_point)
					{
						tofImage.data_depth[i_out_index]         = std::numeric_limits<float>::quiet_NaN();

						tofImage.data_3d_xyz_rgb[i_point_3d]     = std::numeric_limits<float>::quiet_NaN();
						tofImage.data_3d_xyz_rgb[i_point_3d + 1] = std::numeric_limits<float>::quiet_NaN();
						tofImage.data_3d_xyz_rgb[i_point_3d + 2] = std::numeric_limits<float>::quiet_NaN();
						tofImage.data_3d_xyz_rgb[i_point_3d + 3] = std::numeric_limits<float>::quiet_NaN();
						tofImage.data_3d_xyz_rgb[i_point_3d + 4] = std::numeric_limits<float>::quiet_NaN();
						tofImage.data_3d_xyz_rgb[i_point_3d + 5] = std::numeric_limits<float>::quiet_NaN();
						tofImage.data_3d_xyz_rgb[i_point_3d + 6] = std::numeric_limits<float>::quiet_NaN();
						tofImage.data_3d_xyz_rgb[i_point_3d + 7] = std::numeric_limits<float>::quiet_NaN();
					}
				}

				i_point_3d += 8;
				i_2d_color += 3;
				i_out_index ++;
			}
		}
		frame_index ++;
	}

	return status;
}

/**
 * @brief Request distance and amplitude
 *
 * This function will be answered by the signal "receivedDistanceAmplitude"
 */
ErrorNumber_e Camera160::getDistanceAmplitude(ToFImage &tofImage)
{
	int dataSize;
	bool streaming = false;
	if(acquisitionMode == STREAM) streaming = true;

	if(hdr == HDR_SPATIAL) dataSize = static_cast<int>(2 * n_data_points + 88); //16 bit distance/2 + 16 bit amplitude/2
	else dataSize = static_cast<int>(4 * n_data_points + 88); //16 bit distance + 16 bit amplitude

	ErrorNumber_e status;
	bool hdrTemporal = (hdr == HDR_TEMPORAL);
	
	std::map<unsigned int, std::vector<uint8_t> > map_data_array;

	unsigned int total_filled = 0;
	unsigned int total_accquired = 0;
	integrationTime3ds.clear();
	do
	{
		total_filled = 0;

		status = communication->sendCommandSingleByte(Constants::CommandList::COMMAND_GET_DISTANCE_AMPLITUDE, static_cast<uint8_t>(acquisitionMode), dataSize, streaming);
		total_accquired ++;

		if (status != ERROR_NUMMBER_NO_ERROR)
		{
			return ERROR_NUMBER_INVALID_DATA;
		}
		std::vector<uint8_t> data_array = communication->data();
		if (data_array.size() == 0)
		{
			break;
		}
		
		unsigned int currentIntegrationTime3dWf = getUint16LittleEndian(data_array, C160::ImageHeader::INDEX_CURRENT_INTEGRATION_TIME_3D_WF);
		if (hdrTemporal)
		{

			for (unsigned int i = 0; i < C160::ImageHeader::NUM_INTEGRATION_TIME_3D; i++)
			{
				unsigned int integrationTime3d = getUint16LittleEndian(data_array, C160::ImageHeader::INDEX_INTEGRATION_TIME_0 + (i * sizeof(uint16_t)));

				if (integrationTime3d > 0)
				{
					std::map<unsigned int, bool>::iterator it = integrationTime3ds.find(integrationTime3d);
					if (it != integrationTime3ds.end())
					{
						if (currentIntegrationTime3dWf == integrationTime3d) it->second = true;
					} else
					{
						//integrationTime3ds.insert(std::pair<unsigned int, bool>(integrationTime3d,currentIntegrationTime3dWf == integrationTime3d));
						integrationTime3ds[integrationTime3d] = (currentIntegrationTime3dWf == integrationTime3d);
					}
				}
			}

			for(std::map<unsigned int, bool>::iterator it = integrationTime3ds.begin(); it != integrationTime3ds.end(); ++it)
			{
				unsigned int filled = static_cast<unsigned int>(it->second);
				total_filled += filled;
			}
		}
		
		map_data_array[currentIntegrationTime3dWf] = data_array;

		if (hdrTemporal)
		{
			if (total_filled >= integrationTime3ds.size())
			{
				break;
			}
		}
	} while(hdrTemporal && total_accquired < C160::ImageHeader::NUM_INTEGRATION_TIME_3D && total_filled < integrationTime3ds.size());

	int frame_index = 0;

	uint16_t image_width = 0;
	uint16_t image_height= 0;

	uint16_t leftX = 0;
	uint16_t topY = 0;
	uint16_t rightX  = 0;
	uint16_t bottomY = 0;
	for(std::map<unsigned int, std::vector<uint8_t> >::iterator it = map_data_array.begin(); it != map_data_array.end(); ++it)
	{
		std::vector<uint8_t> data_array = it->second;

		if (frame_index == 0)
		{
			image_width = static_cast<uint16_t>(getUint16LittleEndian(data_array, C160::ImageHeader::INDEX_WIDTH));
			image_height = static_cast<uint16_t>(getUint16LittleEndian(data_array, C160::ImageHeader::INDEX_HEIGHT));

			leftX = static_cast<uint16_t>(getUint16LittleEndian(data_array, C160::ImageHeader::INDEX_ORIGIN_X));
			topY = static_cast<uint16_t>(getUint16LittleEndian(data_array, C160::ImageHeader::INDEX_ORIGIN_Y));
			rightX  = image_width  + leftX;
			bottomY = image_height + topY;
		}

		uint32_t *distanceAmplitude =  (uint32_t *)(&data_array.data()[C160::ImageHeader::SIZE]);

		unsigned int val;
		int i_data_point = 0, i_2d_color = 0, i_point_3d = 0, i_out_index = 0, p_index = 0;
		Color c;
		int rgb;
		uint8_t r;
		uint8_t g;
		uint8_t b;
		bool valid_point;

		for(int y = 0; y < 60; y++)
		{
			for(int x = 0; x < 160; x++)
			{
				if (x < leftX || x >= rightX || y < topY || y >= bottomY)
				{
					val = C160::Pixel::LIMIT_VALID_PIXEL;
				} else {
					uint32_t p = distanceAmplitude[i_data_point];
					val = p & 0xFFFF;
					if ((! hdrTemporal) || frame_index == 0)
					{
						tofImage.data_amplitude[i_out_index] = p >> 16;
					}
					i_data_point ++;
				}

				if (frame_index == 0) valid_mask[i_out_index] = false;

				if ((! hdrTemporal) || frame_index == 0)
				{
					tofImage.data_grayscale[i_out_index] = 0;
				}

				if ((! hdrTemporal) || frame_index == 0 || (! valid_mask[i_out_index]))
				{
					c = imageColorizer->getColor(val);

					r = c.r;
					g = c.g;
					b = c.b;

					valid_point = false;
					if(val < C160::Pixel::LIMIT_VALID_PIXEL)
					{
						rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);

						p_index = i_out_index;
						if (hdr == HDR_SPATIAL)
						{
							p_index = 320 * y  + x;
						}

						cal3DXYZ(p_index, getWidth(), getHeight(), val, d3X, d3Y, d3Z, X, Y, Z);

						tofImage.saturated_mask[i_out_index]     = 0;

						tofImage.data_depth[i_out_index]         = Z;

						tofImage.data_2d_bgr[i_2d_color]         = b;
						tofImage.data_2d_bgr[i_2d_color + 1]     = g;
						tofImage.data_2d_bgr[i_2d_color + 2]     = r;

						tofImage.data_3d_xyz_rgb[i_point_3d]     = X;
						tofImage.data_3d_xyz_rgb[i_point_3d + 1] = Y;
						tofImage.data_3d_xyz_rgb[i_point_3d + 2] = Z;
						tofImage.data_3d_xyz_rgb[i_point_3d + 3] = 0.0f;
						tofImage.data_3d_xyz_rgb[i_point_3d + 4] = *reinterpret_cast<float*>(&rgb);
						tofImage.data_3d_xyz_rgb[i_point_3d + 5] = *reinterpret_cast<float*>(&r); //r;
						tofImage.data_3d_xyz_rgb[i_point_3d + 6] = *reinterpret_cast<float*>(&g); //g;
						tofImage.data_3d_xyz_rgb[i_point_3d + 7] = *reinterpret_cast<float*>(&b); //b;

						valid_point = true;
						valid_mask[i_out_index] = true;

					} else if (val == C160::Pixel::VALUE_ADC_OVERFLOW || val == C160::Pixel::VALUE_SATURATION)
					{
						tofImage.saturated_mask[i_out_index]     = 255;

						tofImage.data_2d_bgr[i_2d_color]         = b;
						tofImage.data_2d_bgr[i_2d_color + 1]     = g;
						tofImage.data_2d_bgr[i_2d_color + 2]     = r;
					} else
					{
						tofImage.saturated_mask[i_out_index]     = 0;

						tofImage.data_2d_bgr[i_2d_color]         = std::numeric_limits<uint8_t>::quiet_NaN();
						tofImage.data_2d_bgr[i_2d_color + 1]     = std::numeric_limits<uint8_t>::quiet_NaN();
						tofImage.data_2d_bgr[i_2d_color + 2]     = std::numeric_limits<uint8_t>::quiet_NaN();
					}
					if (! valid_point)
					{
						tofImage.data_depth[i_out_index]         = std::numeric_limits<float>::quiet_NaN();

						tofImage.data_3d_xyz_rgb[i_point_3d]     = std::numeric_limits<float>::quiet_NaN();
						tofImage.data_3d_xyz_rgb[i_point_3d + 1] = std::numeric_limits<float>::quiet_NaN();
						tofImage.data_3d_xyz_rgb[i_point_3d + 2] = std::numeric_limits<float>::quiet_NaN();
						tofImage.data_3d_xyz_rgb[i_point_3d + 3] = std::numeric_limits<float>::quiet_NaN();
						tofImage.data_3d_xyz_rgb[i_point_3d + 4] = std::numeric_limits<float>::quiet_NaN();
						tofImage.data_3d_xyz_rgb[i_point_3d + 5] = std::numeric_limits<float>::quiet_NaN();
						tofImage.data_3d_xyz_rgb[i_point_3d + 6] = std::numeric_limits<float>::quiet_NaN();
						tofImage.data_3d_xyz_rgb[i_point_3d + 7] = std::numeric_limits<float>::quiet_NaN();
					}
				}

				i_point_3d += 8;
				i_2d_color += 3;
				i_out_index ++;
			}
		}
		frame_index ++;
	}

	return status;
}

ErrorNumber_e Camera160::getLensCalibrationData(CameraInfo &cameraInfo)
{
	int size = 136 + Constants::Data::SIZE_OVERHEAD;
	ErrorNumber_e status = communication->sendCommandWithoutData(Constants::CommandList::COMMAND_GET_LENS_CALIBRATION_DATA, size); //(8+9)*8

	if (status == ERROR_NUMMBER_NO_ERROR)
	{
		std::vector<uint8_t> data_array = communication->data();
		int numberOfElements = data_array.size() / sizeof(double);

		if (numberOfElements >= 9)
		{
			int numD = numberOfElements - 9; //9 ->3x3 cameraMatrix

			uint8_t *p = data_array.data();

			for(int i = 0; i < numD; i++) {
				cameraInfo.D[i] = *((double*)(&p[i * sizeof(double)]));
			}

			for(int i = numD; i < numberOfElements; i++) {
				cameraInfo.K[i-numD] = *((double*)(&p[i * sizeof(double)]));
			}
		}
	}

	return status;
}

/**
 * @brief Set mode
 *
 * Call this function to set the operation mode
 *
 * @param mode Mod eto set
 */
ErrorNumber_e Camera160::setOperationMode(OperationMode_e mode)
{
	this->opMode = mode;

	switch(mode)
	{
		case MODE_BEAM_A:
			communication->sendCommandSingleByte(Constants::CommandList::COMMAND_SET_MODE, (uint8_t)MODE_BEAM_A);
			break;
		case MODE_BEAM_B:
			communication->sendCommandSingleByte(Constants::CommandList::COMMAND_SET_MODE, (uint8_t)MODE_BEAM_B);
			break;
		default:
			return ERROR_NUMBER_INVALID_PARAMETER;
	}
	return ERROR_NUMMBER_NO_ERROR;
}

void Camera160::setAcquisitionMode(AcquisitionMode_e mode)
{
	acquisitionMode = mode;
}

ErrorNumber_e Camera160::setOffset(const int offset){
	return communication->sendCommandInt16(Constants::CommandList::COMMAND_SET_OFFSET, static_cast<int16_t>(offset));
}

/**
 * @brief Set integration time 3D
 *
 * @param index Index of the integration time
 * @param integrationTime Integration time in us
 * @return Error code from ErrorNumber_e
 */
ErrorNumber_e Camera160::setIntegrationTime3d(const unsigned int index, const unsigned int integrationTime)
{
	uint8_t output[Constants::Command::SIZE_TOTAL];

	memset(output, 0, sizeof(output));

	//Add the command
	output[Constants::Command::INDEX_COMMAND] = Constants::CommandList::COMMAND_SET_INTEGRATION_TIME_3D;

	//Add the index
	output[Constants::IntegrationTime::INDEX_INDEX_3D] = static_cast<uint8_t>(index);

	//Add the time
	setUint16LittleEndian(output, Constants::IntegrationTime::INDEX_INTEGRATION_TIME_3D, integrationTime);

	//Send blocking
	return communication->sendCommand(output, SET_INTEGRATION_TIME_DIS_SIZE);
}

ErrorNumber_e Camera160::setAutoIntegrationTime3d()
{
	return setIntegrationTime3d(0xff, 0);
}

/**
 * @brief Set integration time grayscale
 *
 * @param integrationTime Integration time in us
 * @return Error code from ErrorNumber_e
 */
ErrorNumber_e Camera160::setIntegrationTimeGrayscale(const unsigned int integrationTime)
{
	return communication->sendCommandUint16(Constants::CommandList::COMMAND_SET_INTEGRATION_TIME_GRAYSCALE, static_cast<uint16_t>(integrationTime));
}

/**
 * @brief Set modulation frequency
 *
 * @param modulationFrequency Selected modulation frequency
 * @return Error code from ErrorNumber_e
 */
ErrorNumber_e Camera160::setModulationFrequency(const ModulationFrequency_e modulationFrequency)
{
	ErrorNumber_e status = ERROR_NUMBER_INVALID_PARAMETER;

	if(modulationFrequency == MODULATION_FREQUENCY_10MHZ){
		status = communication->sendCommandSingleByte(Constants::CommandList::COMMAND_SET_MODULATION_FREQUENCY, Constants::ModulationFrequency::VALUE_10MHZ);
	}else if(MODULATION_FREQUENCY_20MHZ){
		status = communication->sendCommandSingleByte(Constants::CommandList::COMMAND_SET_MODULATION_FREQUENCY, Constants::ModulationFrequency::VALUE_20MHZ);
	}

  return status;
}

/**
 * @brief Set modulation channel
 *
 * @param autoChannelEnabled
 * @param channel
 * @return Error code from ErrorNumber_e
 */
ErrorNumber_e Camera160::setModulationChannel(const bool autoChannelEnabled, const int channel)
{
	uint8_t valueEnabled = 0;

	if (autoChannelEnabled)
	{
		valueEnabled = 1;
	}

	return communication->sendCommand2xByte(Constants::CommandList::COMMAND_SET_MODULATION_CHANNEL, valueEnabled, channel, Constants::Type::DATA_ACK, false);
}

/**
 * @brief Set the filter settings
 *
 * Factor example:
 * 300 gives 300 x actualValue + 700 x lastValue
 *
 * @param threshold Threshold where the filter is cleared
 * @param factor Factor for the actual value
 * @return Error code from ErrorNumber_e
 */
ErrorNumber_e Camera160::setFilter(const unsigned int threshold, const unsigned int factor)
{
	return communication->sendCommand2xUint16(Constants::CommandList::COMMAND_SET_FILTER, static_cast<uint16_t>(threshold), static_cast<uint16_t>(factor));
}


/**
 * @brief Set the filter single spot settings
 *
 * Factor example:
 * 300 gives 300 x actualValue + 700 x lastValue
 *
 * @param threshold Threshold where the filter is cleared
 * @param factor Factor for the actual value
 */
ErrorNumber_e Camera160::setFilterSpot(const unsigned int threshold, const unsigned int factor)
{
	return communication->sendCommand2xUint16(Constants::CommandList::COMMAND_SET_FILTER_SINGLE_SPOT, static_cast<uint16_t>(threshold), static_cast<uint16_t>(factor));
}


ErrorNumber_e Camera160::setDcsFilter(const bool enabled)
{
	uint8_t value = 0;

	//bool to 0/1
	if (enabled)
	{
		value = 1;
	}

	return communication->sendCommandSingleByte(Constants::CommandList::COMMAND_SET_DCS_FILTER, value);
}

/**
 * @brief Enable/disable the gaussian filter
 *
 * @param enabled gaussian filter enabled or not
 */
ErrorNumber_e Camera160::setGaussianFilter(const bool enabled)
{
	uint8_t value = 0;

	//bool to 0/1
	if (enabled)
	{
		value = 1;
	}

	return communication->sendCommandSingleByte(Constants::CommandList::COMMAND_SET_GAUSSIAN_FILTER, value, Constants::Type::DATA_ACK);
}


/**
 * @brief Set the calibration mode
 *
 * In calibration mode the device disables the compensation and sends raw distance information. This is used during the calibration
 * procedure.
 *
 * calibration mode enabled --> compensation disabled
 * calibration mode disabled --> compensation enabled
 *
 * @param enabled Enable or disable calibration mode
 * @return Error code from ErrorNumber_e
 */
ErrorNumber_e Camera160::setCalibrationMode(const bool enabled)
{
	uint8_t value = 0;

	if (enabled)
	{
		value = 1;
	}

	return communication->sendCommandSingleByte(Constants::CommandList::COMMAND_CALIBRATE_DRNU, value);
}

ErrorNumber_e Camera160::setMinimalAmplitude(const unsigned index, const unsigned int amplitude){
	uint8_t output[Constants::Command::SIZE_TOTAL];

	memset(output, 0, sizeof(output));

	//Add the command
	output[Constants::Command::INDEX_COMMAND] = Constants::CommandList::COMMAND_SET_MINIMAL_AMPLITUDE;

	//Add the index
	output[Constants::Amplitude::INDEX_INDEX] = static_cast<uint8_t>(index);

	//Add the amplitude
	setUint16LittleEndian(output, Constants::Amplitude::INDEX_AMPLITUDE, amplitude);

	return communication->sendCommand(output, Constants::Command::SIZE_PAYLOAD);
}

ErrorNumber_e Camera160::setBinning(const int binning){
	return communication->sendCommandSingleByte(Constants::CommandList::COMMAND_SET_BINNING, static_cast<uint8_t>(binning));
}

ErrorNumber_e Camera160::setFrameRate(const unsigned int FrameTime){
	return communication->sendCommandUint16(Constants::CommandList::COMMAND_SET_FRAME_RATE, static_cast<uint16_t>(FrameTime));
}

ErrorNumber_e Camera160::setHdr(HDR_e hdr){

	bool hdrSpatial = false;
	if(hdr == HDR_SPATIAL || (this->hdr == HDR_SPATIAL && hdr != HDR_SPATIAL))
	{
		hdrSpatial = true;
	}

	this->hdr = hdr;

	unsigned int i_hdr = 0;
	if (this->hdr == HDR_SPATIAL)
	{
		i_hdr = 1;
	} else if (this->hdr == HDR_TEMPORAL)
	{
		i_hdr = 2;
	}

	if (hdrSpatial) {
#ifdef _WIN32
		Sleep(3000);
#else
		//usleep(5000000);
		usleep(10000);
#endif
	}

	ErrorNumber_e status = communication->sendCommandSingleByte(Constants::CommandList::COMMAND_SET_HDR, static_cast<uint8_t>(i_hdr));

	if (hdrSpatial) {
#ifdef _WIN32
		Sleep(3000);
#else
		//usleep(5000000);
		usleep(10000);
#endif
	}

	return status;
}


/**
 * @brief Set the ROI
 *
 * Set the ROI (region of interest) of the image
 *
 * @param xMin X coordinate top left
 * @param yMin Y coordinate top left
 * @param xMax X coordinate bottom right
 * @param yMax Y coordinate bottom right
 */
ErrorNumber_e Camera160::setRoi(const unsigned int xMin, const unsigned int yMin, const unsigned int xMax, const unsigned int yMax)
{

  xMin_ = xMin;
  yMin_ = yMin;
  xMax_ = xMax;
  yMax_ = yMax;

  n_data_points = (xMax_ - xMin_ + 1) * (yMax_ - yMin_ + 1);

  uint8_t output[Constants::Command::SIZE_TOTAL + 4 * sizeof(uint16_t)];

  memset(output, 0, sizeof(output));

  //Add the command
  output[Constants::Command::INDEX_COMMAND] = Constants::CommandList::COMMAND_SET_ROI;

  //xMin
  setUint16LittleEndian(output, Constants::ROI::INDEX_ROI_X_MIN, xMin);

  //yMin
  setUint16LittleEndian(output, Constants::ROI::INDEX_ROI_Y_MIN, yMin);

  //xMax
  setUint16LittleEndian(output, Constants::ROI::INDEX_ROI_X_MAX, xMax);

  //yMax
  setUint16LittleEndian(output, Constants::ROI::INDEX_ROI_Y_MAX, yMax);

  ErrorNumber_e status = communication->sendCommand(output, SET_ROI_SIZE);

  //usleep(500000);

  return status;
}

void Camera160::setRange(int start, int stop)
{
	imageColorizer->setRange(start, stop);
}
