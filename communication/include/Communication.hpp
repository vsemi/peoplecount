/**
 * Copyright (C) 2018 Visionary Semiconductor Inc.
 *
 * @defgroup Communication
 * @brief Communication interface
 * @ingroup driver
 *
 * @{
 */
#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include "Connection.hpp"

#include <exception>
#include <list>
#include <string>
#include <iostream>
#include <vector>

#include "constants.h"
#include "common.h"

#define GET_INTEGRATION_TIME_DIS_SIZE 10

//! Communication
/*!
 * Communication class.
 */
class Communication
{
public:
	Communication(Connection *connection);
	~Communication();

	bool open();
	void close();

	ErrorNumber_e setPower(const bool enabled);

	void startStream();

	ErrorNumber_e sendCommand(uint8_t *data, int size, bool streamMode = false);
	ErrorNumber_e sendCommandWithoutData(const uint8_t command, int size, bool streamMode = false);
	ErrorNumber_e sendCommandSingleByte(const uint8_t command, const uint8_t payload, int size = Constants::Command::SIZE_PAYLOAD, bool streaming = false);
	ErrorNumber_e sendCommand2xByte(const uint8_t command, const uint8_t payload0, const uint8_t payload1, uint8_t type, bool blocking);
	ErrorNumber_e sendCommandUint16(const uint8_t command, const uint16_t payload);
	ErrorNumber_e sendCommandInt16(const uint8_t command, const int16_t payload);
	ErrorNumber_e sendCommand2xUint16(const uint8_t command, const uint16_t payload0, const uint16_t payload1);

	std::vector<uint8_t> data();

protected:
	Connection *connection;

private:

	bool startStreamMode;
};

#endif // COMMUNICATION_H

/** @} */
