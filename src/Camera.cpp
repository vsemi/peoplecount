/*
 * Driver.cpp
 *
 *  Created on: Jun 8, 2021
 *      Author: vsemi
 */
#include "Camera.hpp"

#include "Camera160.hpp"

Camera* Camera::usb_tof_camera_160(std::string port)
{
	return new Camera160(port);
}

Camera::~Camera() throw ()
{
}

uint16_t Camera::getChipId()
{
	return _chipId;
}

uint16_t Camera::getWaferId()
{
	return _waferId;
}

uint32_t Camera::getID()
{
	return _sensorId;
}

void Camera::setID(uint16_t chipId, uint16_t waferId)
{
	_chipId = chipId;
	_waferId = waferId;
	_sensorId = (waferId << 16) + chipId;
}

void Camera::setWidth(int width, int height)
{
	_width = width;
	_height = height;
}

int Camera::getWidth()
{
	return _width;
}
int Camera::getHeight()
{
	return _height;
}
