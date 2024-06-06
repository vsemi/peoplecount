/**
 * Copyright (C) 2019 Visionary Semiconductor Inc.
 *
 * @defgroup camera_info CameraInfo
 * @brief Util
 * @ingroup driver
 *
 * @{
 */
#ifndef UTIL_H
#define UTIL_H

#include <stdint.h>
#include <vector>

//LittleEndian
uint32_t getUint16LittleEndian(const std::vector<uint8_t> &array, const unsigned int index);
uint32_t getUint32LittleEndian(const std::vector<uint8_t> &array, const unsigned int index);
int32_t getInt16LittleEndian(const std::vector<uint8_t> &array, const unsigned int index);
void setUint16LittleEndian(uint8_t *buffer, const unsigned int index, const unsigned int value);
void setUint32LittleEndian(uint8_t *buffer, const unsigned int index, const unsigned int value);
void setInt16LittleEndian(uint8_t *buffer, const unsigned int index, const int value);
void setUint24LittleEndian(uint8_t *buffer, const unsigned int index, const unsigned int value);

//U32
uint16_t getValueLsb(uint32_t value);
uint16_t getValueMsb(uint32_t value);

//Crc
uint32_t calcCrc32(const uint8_t *data, const uint32_t size);
uint32_t calcCrc32(const uint32_t *data, const uint32_t size);
uint32_t calcCrc32_32(const uint8_t *data, const uint32_t size);

uint32_t calcCrc32Uint8(uint32_t crc, uint8_t data);
uint32_t calcCrc32Uint32(uint32_t crc, uint32_t data);

void initD3F(double* d3X, double* d3Y, double* d3Z, int width, int height, double angle_x, double angle_y);

void cal3DXYZ(int i, int width, int height, unsigned int val, double* d3X, double* d3Y, double* d3Z, float &X, float &Y, float &Z);

//void cal3DXYZ(int i, int width, int height, unsigned int val, double THETA_H, double ALPHA_H, double THETA_V, double ALPHA_V, float &X, float &Y, float &Z);

#endif // UTIL_H

/** @} */
