#define _USE_MATH_DEFINES
#include <cmath>
#include <util.h>
#include <iostream>
//#include <math.h>

using namespace std;

uint32_t getUint32LittleEndian(const vector<uint8_t> &array, const unsigned int index)
{
	unsigned int byte0 = array.at(index);
	unsigned int byte1 = array.at(index+1);
	unsigned int byte2 = array.at(index+2);
	unsigned int byte3 = array.at(index+3);
	uint32_t value = (byte3 << 24) | (byte2 << 16) | (byte1 << 8) | byte0;
	return value;
}

int32_t getInt16LittleEndian(const vector<uint8_t> &array, const unsigned int index)
{
	unsigned int byte0 = array[index];
	unsigned int byte1 = array[index+1];

	int32_t value = static_cast<int32_t>((byte1 << 8) | byte0);
	return value;
}

uint32_t getUint16LittleEndian(const vector<uint8_t> &array, const unsigned int index)
{
	//std::cout << "getUint16LittleEndian index: " << (uint32_t)index << std::endl;
	unsigned int byte0 = array[index];
	unsigned int byte1 = array[index + 1];
	//std::cout << "getUint16LittleEndian byte0: " << (uint32_t)byte0 << std::endl;
	//std::cout << "getUint16LittleEndian byte1: " << (uint32_t)byte1 << std::endl;

	uint32_t value = static_cast<uint32_t>((byte1 << 8) | byte0);
	return value;
}

void setUint16LittleEndian(uint8_t *buffer, const unsigned int index, const unsigned int value)
{
	buffer[index] = value & 0xFF;
	buffer[index+1] = (value >> 8) & 0xFF;
}

void setInt16LittleEndian(uint8_t *buffer, const unsigned int index, const int value)
{
	buffer[index] = value & 0xFF;
	buffer[index+1] = (value >> 8) & 0xFF;
}

void setUint24LittleEndian(uint8_t *buffer, const unsigned int index, const unsigned int value)
{
	buffer[index] = value & 0xFF;
	buffer[index+1] = (value >> 8) & 0xFF;
	buffer[index+2] = (value >> 16) & 0xFF;
}

void setUint32LittleEndian(uint8_t *buffer, const unsigned int index, const unsigned int value)
{
	buffer[index] = value & 0xFF;
	buffer[index+1] = (value >> 8) & 0xFF;
	buffer[index+2] = (value >> 16) & 0xFF;
	buffer[index+3] = (value >> 24) & 0xFF;
}

uint16_t getValueLsb(uint32_t value)
{
	return (value & 0xFFFF);
}

uint16_t getValueMsb(uint32_t value)
{
	return (value >> 16);
}

///CRC32: generator polynom
const uint32_t polynom = 0x04C11DB7;
const uint32_t initValue = 0xFFFFFFFF;
const uint32_t xorValue = 0x00000000;

/**
 * @brief Calculate CRC32 of a 8 bit buffer
 *
 * This function calculates the CRC32 checksum for the given buffer.
 *
 * @param data Pointer to the data buffer
 * @param size Size of the data buffer in Bytes
 * @return CRC32 checksum
 */
uint32_t calcCrc32(const uint8_t *data, const uint32_t size)
{
	uint32_t crc = initValue;

	for(uint32_t i = 0; i < size; i++)
	{
		crc = calcCrc32Uint8(crc, data[i]);
	}
	return crc ^ xorValue;
}

/**
 * @brief Calculate CRC32 of a 32 bit buffer
 *
 * This function calculates the CRC32 checksum for the given buffer.
 *
 * @param data Pointer to the data buffer
 * @param size Size of the data buffer as count of 32bit values
 * @return CRC32 checksum
 */
uint32_t calcCrc32(const uint32_t *data, const uint32_t size)
{
	uint32_t crc = initValue;

	for(uint32_t i = 0; i < size; i++)
	{
		crc = calcCrc32Uint32(crc, data[i]);
	}
	return crc ^ xorValue;
}

/**
 * @brief Calculate CRC32 of a 8 bit buffer
 *
 * This function calculates the CRC32 checksum for the given buffer.
 *
 * @param data Pointer to the data buffer
 * @param size Size of the data buffer in Bytes
 * @return CRC32 checksum
 */
uint32_t calcCrc32_32(const uint8_t *data, const uint32_t size)
{
	uint32_t crc = initValue;

	for(uint32_t i = 0; i < size; i++)
	{
		crc = calcCrc32Uint32(crc, data[i]);
	}
	return crc ^ xorValue;
}

/**
 * @brief Calculate CRC32 of one Byte
 *
 * This function calculates the CRC32 checksum of one Byte. It is only
 * used internally as private function
 *
 * @param crc Actual CRC value
 * @param byte Byte to use
 * @return CRC32 checksum
 */
uint32_t calcCrc32Uint8(uint32_t crc, uint8_t data)
{
	int32_t i;

	//This shift is done to make it compatible to the STM32 hardware CRC
	crc = crc ^ (static_cast<uint32_t>(data) << 24);

	for (i = 0; i < 8; i++)
	{
		if (crc & 0x80000000)
		{
			crc = (crc << 1) ^ polynom;
		}
		else
		{
			crc = (crc << 1);
		}
	}
	return (crc);
}

/**
 * @brief Calculate CRC32 of one Byte
 *
 * This function calculates the CRC32 checksum of one 32bit value. It is only
 * used internally as private function
 *
 * @param crc Actual CRC value
 * @param byte Byte to use
 * @return CRC32 checksum
 */
uint32_t calcCrc32Uint32(uint32_t crc, uint32_t data)
{
	int32_t i;

	crc = crc ^ data;

	for(i=0; i<32; i++)
	{
		if (crc & 0x80000000)
		{
			crc = (crc << 1) ^ polynom;
		}
		else
		{
			crc = (crc << 1);
		}
	}
	return(crc);
}

void initD3F(double* d3X, double* d3Y, double* d3Z, int width, int height, double angle_x, double angle_y)
{
	int l = width * height;

	double THETA_H = M_PI * angle_x / 180.0f;
	double ALPHA_H = (M_PI - THETA_H) / 2;

	double THETA_V = M_PI * angle_y / 180.0f;
	double ALPHA_V = 2 * M_PI - (THETA_V / 2);

	for (int i = 0; i < l; i ++)
	{
		int x = i % width;
		int y = i / width;

		d3Z[i] = fabs(fabs(0.001 * sin(ALPHA_H + (double)x * (THETA_H / width))) * cos(ALPHA_V + (double)y * (THETA_V / height)));

		d3X[i] = (fabs(fabs(0.001 * sin(ALPHA_H + (double)x * (THETA_H / width))) * cos(ALPHA_V + (double)y * (THETA_V / height))) / tan(ALPHA_H + (double)x * (THETA_H / width)));

		d3Y[i] = -1.0 * fabs(fabs(0.001 * sin(ALPHA_H + (double)x * (THETA_H / width))) * cos(ALPHA_V + (double)y * (THETA_V / height))) * tan(ALPHA_V + (double)y * (THETA_V / height));
	}
}

void cal3DXYZ(int i, int width, int height, unsigned int val, double* d3X, double* d3Y, double* d3Z, float &X, float &Y, float &Z)
{
	Z = static_cast<double>(val) * d3Z[i];

	X = static_cast<double>(val) * d3X[i];

	Y = static_cast<double>(val) * d3Y[i];
}

