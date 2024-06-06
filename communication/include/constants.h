/**
 * Copyright (C) 2001 Visionary Semiconductor Inc.
 *
 * @defgroup Constants
 * @ingroup driver
 *
 * @brief Constants communication between firmware and driver.
 *
 * @{
 */
#ifndef CONSTANTS_H_
#define CONSTANTS_H_

#include <stdint.h>

//! Constants
/*!
 * Constants needed for communication on the firmware and on the PC.
 */

//#define SINGLE 0
//#define AUTO_REPEAT 1
//#define STREAM 3

//#define HDR_OFF 0
//#define HDR_SPATIAL 1
//#define HDR_TEMPORAL 2

namespace Constants
{
	namespace Data
	{
		const uint8_t  START_MARK = 0xFA;                                           ///<Start marker for data
		const uint32_t INDEX_TYPE = 1;                                             ///<Index where the type is found in the buffer
		const uint32_t INDEX_LENGTH = 2;                                           ///<Index where the length is found in the buffer
		const uint32_t INDEX_DATA = 4;                                             ///<Index where the data is found in the buffer
		const uint32_t SIZE_HEADER = 4;                                            ///<Number of bytes for the data header
		const uint32_t SIZE_CHECKSUM = sizeof(uint32_t);                           ///<Size of the checksum
		const uint32_t SIZE_OVERHEAD = SIZE_HEADER + SIZE_CHECKSUM;                ///<Nuber of overhead bytes = additional bytes to payload
	}
	namespace Command
	{
		const uint8_t  START_MARK = 0xF5;                                           ///<Start marker for commands
		const uint32_t INDEX_COMMAND = 1;                                          ///<Index where the command is found in the buffer
		const uint32_t INDEX_DATA = 2;                                             ///<Index where the data if ound in a command
		const uint32_t INDEX_CHECKSUM = 10;                                        ///<Index where the checksum is found in a command
		const uint32_t SIZE_TOTAL = 14;                                            ///<Number of bytes for one command
		const uint32_t SIZE_PAYLOAD = 8;                                           ///<Number of bytes of the payload of a command
	}
	namespace Type
	{
		const uint8_t DATA_ACK = 0x00;                                             ///<Acknowledge from sensor to host
		const uint8_t DATA_NACK = 0x01;                                            ///<Not acknowledge from sensor to host
		const uint8_t DATA_IDENTIFICATION = 0x02;                                  ///<Identification to identify the device
		const uint8_t DATA_DISTANCE = 0x03;                                        ///<Distance information
		const uint8_t DATA_AMPLITUDE = 0x04;                                       ///<Amplitude information
		const uint8_t DATA_DISTANCE_AMPLITUDE = 0x05;                              ///<Distance and amplitude information
		const uint8_t DATA_GRAYSCALE = 0x06;                                       ///<Grayscale information
		const uint8_t DATA_DCS = 0x07;                                             ///<DCS data
		const uint8_t DATA_DCS_DISTANCE_AMPLITUDE = 0x08;                          ///<DCS, distance and amplitude all together
		const uint8_t DATA_INTEGRATION_TIME = 0x09;                                ///<Integration time, answer to COMMAND_GET_INTEGRATION_TIME_3D
		const uint8_t DATA_DISTANCE_GRAYSCALE = 0x0A;                              ///<Distance and grayscale data
		const uint8_t DATA_LENS_CALIBRATION_DATA = 0xF7;                           ///<Lens calibration data
		const uint8_t DATA_TRACE = 0xF8;                                           ///<Trace data
		const uint8_t DATA_PRODUCTION_INFO = 0xF9;                                 ///<Production info
		const uint8_t DATA_CALIBRATION_DATA = 0xFA;                                ///<Calibration data
		const uint8_t DATA_REGISTER = 0xFB;                                        ///<Register data
		const uint8_t DATA_TEMPERATURE = 0xFC;                                     ///<Temperature data
		const uint8_t DATA_CHIP_INFORMATION = 0xFD;                                ///<Chip information data
		const uint8_t DATA_FIRMWARE_RELEASE = 0xFE;                                ///<Firmware release
		const uint8_t DATA_ERROR = 0xFF;                                           ///<Error number
	}
	namespace CommandList
	{
		//setup commands
		const uint8_t COMMAND_SET_INTEGRATION_TIME_3D = 0x00;                      ///<Command to set the integration time for 3D operation
		const uint8_t COMMAND_SET_INTEGRATION_TIME_GRAYSCALE = 0x01;               ///<Command to set the integration time for grayscale
		const uint8_t COMMAND_SET_ROI = 0x02;                                      ///<Command to set the region of interest
		const uint8_t COMMAND_SET_BINNING = 0x03;                                  ///<Command to set the binning
		const uint8_t COMMAND_SET_MODE = 0x04;                                     ///<Command to set the mode
		const uint8_t COMMAND_SET_MODULATION_FREQUENCY = 0x05;                     ///<Command to set the modulation frequency
		const uint8_t COMMAND_SET_DLL_STEP = 0x06;                                 ///<Command to set the DLL step
		const uint8_t COMMAND_SET_FILTER = 0x07;                                   ///<Command to set the filter parameters
		const uint8_t COMMAND_SET_OFFSET = 0x08;                                   ///<Command to set the offset
		const uint8_t COMMAND_SET_MINIMAL_AMPLITUDE = 0x09;                        ///<Command to set th minimal amplitude
		const uint8_t COMMAND_SET_DCS_FILTER = 0x0A;                               ///<Command to set the DCS filter
		const uint8_t COMMAND_SET_GAUSSIAN_FILTER = 0x0B;                          ///<Command to set the Gaussian filter
		const uint8_t COMMAND_SET_FRAME_RATE = 0x0C;                               ///<Command to set/limit the frame rate
		const uint8_t COMMAND_SET_HDR = 0x0D;
		const uint8_t COMMAND_SET_MODULATION_CHANNEL = 0x0E;                       ///<Command to set the modulation channel
		const uint8_t COMMAND_SET_FILTER_SINGLE_SPOT = 0x0F;                       ///<Command to set the temporal filter for the single spot

		//acquisition commands
		const uint8_t COMMAND_GET_DISTANCE = 0x20;                                 ///<Command to request distance data
		const uint8_t COMMAND_GET_AMPLITUDE = 0x21;                                ///<Command to request amplitude data
		const uint8_t COMMAND_GET_DISTANCE_AMPLITUDE = 0x22;                       ///<Command to request distance and amplitude data
		const uint8_t COMMAND_GET_DCS_DISTANCE_AMPLITUDE = 0x23;                   ///<Command to request distance, amplitude and DCS data at once
		const uint8_t COMMAND_GET_GRAYSCALE = 0x24;                                ///<Command to request grayscale data
		const uint8_t COMMAND_GET_DCS = 0x25;                                      ///<Command to request DCS data
		const uint8_t COMMAND_SET_AUTO_ACQUISITION = 0x26;                         ///<Command to enable/disable the auto acquisition
		const uint8_t COMMAND_GET_INTEGRATION_TIME_3D = 0x27;                      ///<Command to read the integration time. Important when using automatic mode
		const uint8_t COMMAND_STOP_STREAM = 0x28;                                  ///<Command to stop the stream
		const uint8_t COMMAND_GET_DISTANCE_GRAYSCALE = 0x29;                       ///<Command to request distance and grayscale

		//general commands
		const uint8_t COMMAND_SET_POWER = 0x40;                                    ///<Command to enable/disable the power
		const uint8_t COMMAND_CALIBRATE_DRNU = 0x41;                               ///<Command to start DRNU calibration
		const uint8_t COMMAND_CALILBRATE_OFFSET = 0x42;                            ///<Command to calibrate the system offset
		const uint8_t COMMAND_GET_CALIBRATION = 0x43;                              ///<Command to read back the calibration for backup/restore
		const uint8_t COMMAND_JUMP_TO_BOOTLOADER = 0x44;                           ///<Command for application to jump to bootloader
		const uint8_t COMMAND_UPDATE_FIRMWARE = 0x45;                              ///<Command to update the firmware
		const uint8_t COMMAND_IDENTIFY = 0x47;                                     ///<Command to identify
		const uint8_t COMMAND_GET_CHIP_INFORMATION = 0x48;                         ///<Command to read the chip information
		const uint8_t COMMAND_GET_FIRMWARE_RELEASE = 0x49;                         ///<Command to read the firmware release
		const uint8_t COMMAND_GET_TEMPERATURE = 0x4A;                              ///<Command to read the temperature
		const uint8_t COMMAND_WRITE_CALIBRATION = 0x4B;                            ///<Command to write the calibration
		const uint8_t COMMAND_SET_PRODUCTION_INFO = 0x4F;                          ///<Command to set the production info
		const uint8_t COMMAND_GET_PRODUCTION_INFO = 0x50;                          ///<Command to get the production info
		const uint8_t COMMAND_GET_LENS_CALIBRATION_DATA = 0x54;                    ///<Command to get the lens calibration data
	}
	namespace IntegrationTime
	{
		const uint32_t INDEX_INDEX_3D = 2;                                         ///<Index of the integration time 3d index
		const uint32_t INDEX_INTEGRATION_TIME_3D = 3;                              ///<Index of the integration time 3d
	}
	namespace Identification
	{
		const uint32_t MASK_CHIP_TYPE_DEVICE = 0x00FFFF00;                             ///<Mask out chip type and device
		const uint32_t SHIFT_CHIP_TYPE_DEVICE = 8;                                     ///<Shift for chip type and device

		const uint32_t MASK_VERSION = 0x000000FF;                                      ///<Mask to get the version
		const uint32_t SHIFT_VERSION = 0;                                              ///<Shift for the version
	}
	namespace ModulationFrequency
	{
		const uint8_t VALUE_10MHZ = 0;                                             ///<Value for 10MHz for command "COMMAND_SET_MODULATION_FREQUENCY"
		const uint8_t VALUE_20MHZ = 1;                                             ///<Value for 20MHz for command "COMMAND_SET_MODULATION_FREQUENCY"
	}
	namespace ChipInformation
	{
		const uint32_t INDEX_WAFER_ID = 6;                                         ///<Index of wafer id in data type "DATA_CHIP_INFORMATION"
		const uint32_t INDEX_CHIP_ID = 4;                                          ///<Index of chip id in data type "DATA_CHIP_INFORMATION"
	}
	namespace ROI
	{
		const uint32_t INDEX_ROI_X_MIN = 2;
		const uint32_t INDEX_ROI_Y_MIN = 4;
		const uint32_t INDEX_ROI_X_MAX = 6;
		const uint32_t INDEX_ROI_Y_MAX = 8;
	}
	namespace Amplitude
	{
		const uint32_t INDEX_INDEX = 2;                                          ///<Index of the index
		const uint32_t INDEX_AMPLITUDE = 3;                                      ///<Index of the minimal amplitude
	}
}

#endif /* CONSTANTS_H_ */

/** @} */

