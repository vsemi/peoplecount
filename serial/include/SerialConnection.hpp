/**
 * Copyright (C) 2018 Visionary Semiconductor Inc.
 *
 * @defgroup serial_connection Serial Connection
 * @brief Specialized serial port
 * @ingroup communication
 *
 * @{
 */
#ifndef SERIAL_CONNECTION_H
#define SERIAL_CONNECTION_H

#include <memory.h>
#include <vector>
#include <string>
#include <list>
#include <iostream>
#include <cstring>

#include <errno.h>
#include <fcntl.h>
#ifdef _WIN32
#include <io.h>
#else
#include <unistd.h>
#endif
#include <queue>

#ifdef _WIN32
#include <windows.h>
#endif

#include "constants.h"
#include "Connection.hpp"

//! Specialized implementation of serial port
/*!
* This class implements some specific functionality for the communication into the
* base of serial port
*/
class SerialConnection: public Connection
{    
public:
	SerialConnection(std::string port);
	~SerialConnection();

	bool openConnection();
	void closeConnection();

	size_t sendData(uint8_t *data);
	int readData(int readSize, int totalSize);

private:
	std::string port;

#ifdef _WIN32
	HANDLE serialHandle;
#else
	int fileDescription;
#endif


	int setInterfaceAttribs (int speed);
};

#endif // SERIAL_CONNECTION_H

/** @} */
