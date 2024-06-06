/**
 * Copyright (C) 2018 Visionary Semiconductor Inc.
 *
 * @defgroup serial_connection Serial Connection
 * @brief Specialized serial port
 * @ingroup communication
 *
 * @{
 */
#ifndef CONNECTION_H
#define CONNECTION_H

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

#include "constants.h"

/*!
* Abstract class for communication
*/
class Connection
{    
public:
	Connection();
	virtual ~Connection();

	virtual bool openConnection() = 0;
	virtual void closeConnection() = 0;

	virtual size_t sendData(uint8_t *data) = 0;
	virtual int readData(int readSize, int totalSize) = 0;

	std::vector<uint8_t> data() {
		return data_array;
	}
	void clearRxArray()
	{
		data_array.clear();
		rxArray.clear();
	}

protected:
	bool receiveDataSegment(std::vector<uint8_t> array, int maxSize);

	uint32_t calculateChecksum(const uint8_t *data, const uint32_t size);
	bool checksumIsCorrect(const std::vector<uint8_t> &array, const unsigned int expectedSize);

	int getExpextedSize(const std::vector<uint8_t> &array);

	uint8_t getType(const std::vector<uint8_t> &array);

	std::vector<uint8_t> rxArray;
	int expectedSize;

	uint8_t data_type;
	std::vector<uint8_t> data_array;
};

#endif // SERIAL_CONNECTION_H

/** @} */
