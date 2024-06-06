/**
 * Copyright (C) 2021 Visionary Semiconductor Inc.
 *
 * @defgroup Camera160
 * @brief camera interface
 * @ingroup driver
 *
 * @{
 */
#ifndef TOF_CAMERA_160_H_
#define TOF_CAMERA_160_H_

#include<string>
#include<math.h>
#include <vector>
#include <map>

#include "color.hpp"
#include "Camera.hpp"


#include "Communication.hpp"
#include "camera_info.h"

#define GET_CHIP_INFORMATION_SIZE     12
#define GET_IDENTIFY_SIZE             12
#define GET_FIRMWARE_VERSION_SIZE     12
#define GET_TEMPERATURE_SIZE          10

#define SET_INTEGRATION_TIME_DIS_SIZE  8

#define SET_ROI_SIZE  8

namespace C160
{
	namespace ImageHeader
	{
		const uint32_t NUM_INTEGRATION_TIME_3D = 6; // originally was 8, but seems not in use, only get 6 from sensor;
		const uint32_t SIZE = 80;                                                ///<Number of bytes for the header, 2 bytes spare
		namespace Beam
		{
			const uint8_t BEAM_NONE = 0;
			const uint8_t BEAM_A = 1;
			const uint8_t BEAM_B = 2;
		}
		const uint32_t INDEX_VERSION = 0;
		const uint32_t INDEX_FRAME_COUNTER = 1;
		const uint32_t INDEX_TIMESTAMP = 3;
		const uint32_t INDEX_FIRMWARE_VERSION = 5;
		const uint32_t INDEX_HARDWARE_VERSION = 9;
		const uint32_t INDEX_CHIP_ID = 10;
		const uint32_t INDEX_WIDTH = 12;
		const uint32_t INDEX_HEIGHT = 14;
		const uint32_t INDEX_ORIGIN_X = 16;
		const uint32_t INDEX_ORIGIN_Y = 18;
		const uint32_t INDEX_CURRENT_INTEGRATION_TIME_3D_WF = 20;
		const uint32_t INDEX_CURRENT_INTEGRATION_TIME_3D_NF = 22;
		const uint32_t INDEX_CURRENT_INTEGRATION_TIME_GRAYSCALE = 24;
		const uint32_t INDEX_INTEGRATION_TIME_GRAYSCALE = 26;
		const uint32_t INDEX_INTEGRATION_TIME_0 = 28;
		const uint32_t INDEX_INTEGRATION_TIME_1 = 30;
		const uint32_t INDEX_INTEGRATION_TIME_2 = 32;
		const uint32_t INDEX_INTEGRATION_TIME_3 = 34;
		const uint32_t INDEX_INTEGRATION_TIME_4 = 36;
		const uint32_t INDEX_INTEGRATION_TIME_5 = 38;
		const uint32_t INDEX_INTEGRATION_TIME_6 = 40;
		const uint32_t INDEX_INTEGRATION_TIME_7 = 42;
		const uint32_t INDEX_AMPLITUDE_LIMIT_0 = 44;
		const uint32_t INDEX_AMPLITUDE_LIMIT_1 = 46;
		const uint32_t INDEX_AMPLITUDE_LIMIT_2 = 48;
		const uint32_t INDEX_AMPLITUDE_LIMIT_3 = 50;
		const uint32_t INDEX_AMPLITUDE_LIMIT_4 = 52;
		const uint32_t INDEX_OFFSET = 54;
		const uint32_t INDEX_BINNING = 56;
		const uint32_t INDEX_DISTANCE_TEMPORAL_FILTER_FACTOR = 57;
		const uint32_t INDEX_DISTANCE_TEMPORAL_FILTER_THRESHOLD = 59;
		const uint32_t INDEX_SINGLE_VALUE_TEMPORAL_FILTER_FACTOR = 61;
		const uint32_t INDEX_SINGLE_VALUE_TEMPORAL_FILTER_THRESHOLD = 63;
		const uint32_t INDEX_MODULATION_FREQUENCY = 65;
		const uint32_t INDEX_MODULATION_CHANNEL = 66;
		const uint32_t INDEX_FLAGS = 67;
		const uint32_t INDEX_TEMPERATURE = 69;
		const uint32_t INDEX_ILLUMINATION_BEAM = 71;
		const uint32_t INDEX_BEAM_B_DISTANCE = 72;
		const uint32_t INDEX_BEAM_B_AMPLITUDE = 74;
	}
	namespace Pixel
	{
		const uint32_t LIMIT_VALID_PIXEL = 16000;
		const uint32_t VALUE_LOW_AMPLITUDE = 16001;
		const uint32_t VALUE_ADC_OVERFLOW = 16002;
		const uint32_t VALUE_SATURATION = 16003;
		const uint16_t MASK_OUT_CONFIDENCE = 0x3FFF;
	}
}

class Camera160 : public Camera
{
	public:
		Camera160(std::string port);
		~Camera160() throw ();

		std::string getPort();
		bool open();

		void startStream();
		ErrorNumber_e stopStream();

		ErrorNumber_e getChipInformation(uint16_t &chipId, uint16_t &waferId);
		ErrorNumber_e getIdentification(unsigned int &device, unsigned int &version);
		ErrorNumber_e getFirmwareRelease(unsigned int &major, unsigned int &minor);

		ErrorNumber_e getTemperature(int16_t &temperature);

		ErrorNumber_e getDistance(ToFImage &tofImage);
		ErrorNumber_e getDistanceGrayscale(ToFImage &tofImage);
		ErrorNumber_e getDistanceAmplitude(ToFImage &tofImage);

		ErrorNumber_e getLensCalibrationData(CameraInfo &cameraInfo);

		ErrorNumber_e setOperationMode(OperationMode_e mode);
		void setAcquisitionMode(AcquisitionMode_e mode);
		ErrorNumber_e setOffset(const int offset);
		ErrorNumber_e setIntegrationTime3d(const unsigned int index, const unsigned int integrationTime);
		ErrorNumber_e setAutoIntegrationTime3d();

		ErrorNumber_e setIntegrationTimeGrayscale(const unsigned int integrationTime);
		ErrorNumber_e setModulationFrequency(const ModulationFrequency_e modulationFrequency);
		ErrorNumber_e setFilter(const unsigned int threshold, const unsigned int factor);
		ErrorNumber_e setFilterSpot(const unsigned int threshold, const unsigned int factor);
		ErrorNumber_e setDcsFilter(const bool enabled);
		ErrorNumber_e setGaussianFilter(const bool enabled);
		ErrorNumber_e setCalibrationMode(const bool enabled);

		ErrorNumber_e setMinimalAmplitude(const unsigned index, const unsigned int amplitude);
		ErrorNumber_e setBinning(const int binning);
		ErrorNumber_e setFrameRate(const unsigned int FrameTime);
		ErrorNumber_e setHdr(HDR_e hdr);
		ErrorNumber_e setRoi(const unsigned int xMin, const unsigned int yMin, const unsigned int xMax, const unsigned int yMax);

		ErrorNumber_e setModulationChannel(const bool autoChannelEnabled, const int channel);

		void setRange(int start, int stop);

		void setFoV(double angle_x, double angle_y);

		void setBackground(float *background, bool removeBackground);

	private:
		std::string _port;

		Communication* communication;

		AcquisitionMode_e acquisitionMode;
		OperationMode_e opMode         ;
		HDR_e hdr            ;

	    unsigned int xMin_;
	    unsigned int yMin_;
	    unsigned int xMax_;
	    unsigned int yMax_;

		int n_points;
		int n_data_points;

		double* d3X;
		double* d3Y;
		double* d3Z;

		float X, Y, Z;

		std::map<unsigned int, bool> integrationTime3ds;
		bool* valid_mask;

		ImageColorizer *imageColorizer;

		float *data_background;
		bool removeBackground;
};

#endif /* TOF_CAMERA_160_H_ */

/** @} */
