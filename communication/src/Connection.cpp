#include "Connection.hpp"
#include <iostream>

#include <util.h>

using namespace std;

Connection::Connection()
{
	expectedSize = 0;
}

Connection::~Connection()
{
	rxArray.clear();
}

/**
 * @brief Check, if the checksum is correct
 *
 * This function checks the CRC. It extracts the CRC from the received data and calculates the CRC of the received
 * data and then compares them.
 *
 * @param array Pointer to the received data
 * @param expectedSize Expected size of the received data
 * @return Checksum correct or not
 */
bool Connection::checksumIsCorrect(const vector<uint8_t> &array, const unsigned int expectedSize)
{
	//std::cout << "checksumIsCorrect -> expectedSize: " << expectedSize << std::endl;
	//std::cout << "checksumIsCorrect -> array size:   " << array.size() << std::endl;
	//The received CRC is the one in the data
	uint32_t receivedCrc = getUint32LittleEndian(array, (Constants::Data::SIZE_HEADER + expectedSize));
	//std::cout << "checksumIsCorrect -> receivedCrc: " << receivedCrc << std::endl;
	//The wanted CRC is the one calculated out of the payload
	uint32_t wantedCrc = calculateChecksum((uint8_t *)array.data(), (Constants::Data::SIZE_HEADER + expectedSize));
	//std::cout << "checksumIsCorrect -> wantedCrc:   " << wantedCrc << std::endl;
	if (receivedCrc == wantedCrc){
		return true;
	}

	std::cout << "Checksum ERROR!!!" << std::endl;
	return false;
}

/**
 * @brief Extract the expected size from the received data
 *
 * @param array Pointer to the received data
 * @return Expected size
 */
int Connection::getExpextedSize(const std::vector<uint8_t> &array)
{
	int expectedSize = getUint16LittleEndian(array, Constants::Data::INDEX_LENGTH);
	return expectedSize;
}

/**
 * @brief Extract the type from the received data
 *
 * @param array Pointer to the received data
 * @return Received type
 */
uint8_t Connection::getType(const vector<uint8_t> &array)
{
	return array.at(Constants::Data::INDEX_TYPE);
}

/**
 * @brief Process the received data
 *
 * @param array Pointer to the received byte array
 */
bool Connection::receiveDataSegment(std::vector<uint8_t> array, int maxSize)
{
	if (array.size() == 0){
		//std::cerr << "ERROR Connection::receiving data: array.size() = 0" << std::endl;
		return false;
	}

	if(array.at(0) != Constants::Data::START_MARK)
	{
		//std::cerr << "ERROR Connection::receiving data: starting byte is not START_MARK" << std::endl;
		while (array.size() > 0)
		{
			//std::cerr << "   ... erase the first byte" << std::endl;
			array.erase(array.begin(), array.begin() + 1 );
			if(array.size() > 0 && array.at(0) == Constants::Data::START_MARK)
			{
				break;
			}
		}
		if (array.size() == 0)
		{
			//std::cerr << "ERROR Connection::receiving data: starting byte is not START_MARK" << std::endl;
			return false;
		}
	}

	//Cancel if not enough bytes received to know the expectedSize
	if (array.size() < (static_cast<int>(Constants::Data::SIZE_HEADER)))
		return true;

	//Get the expected size
	expectedSize = getExpextedSize(array);

	//std::cerr << "receiveDataSegment -> expectedSize: " << expectedSize << std::endl;
	if (expectedSize == 0 && array.size() >= 8)
	{
		//std::cerr << "      start 8 bytes do not contains expected size: " << std::endl;
		rxArray.clear();
	}

	//Cancel here if not enough bytes received
	if (array.size() < (static_cast<unsigned int>(expectedSize + Constants::Data::SIZE_OVERHEAD)))
	{
		return true;
	}

	//Check if the end marking is present. Only use the data if this is the case.
	if (checksumIsCorrect(array, expectedSize))
	{
		uint8_t type = getType(array);
		vector<uint8_t> dataArray = array;

		dataArray.erase(dataArray.begin(), dataArray.begin() + Constants::Data::SIZE_HEADER);

		//Remove checksum at the end
		dataArray.erase(dataArray.begin() + expectedSize,  dataArray.begin() + expectedSize + Constants::Data::SIZE_CHECKSUM);
		dataArray.erase(dataArray.begin() + expectedSize, dataArray.end());

		//Remove the remaining: thats the checksum --- is this needed? data already received properly, this should not needed!
		//array.erase(array.begin(), array.begin() + (expectedSize + Constants::Data::SIZE_CHECKSUM) );

		data_type = type;
		data_array = dataArray;
		//std::cout << "receiveDataSegment data_type: " << data_type << std::endl;
		//std::cout << "receiveDataSegment data_array.size(): " << data_array.size() << std::endl;

		return true;
	}else{
		std::cerr << "ERROR Connection::receiving corrupted data %s" << array.data() << std::endl;
		array.clear();
		return false;
	}
}

uint32_t Connection::calculateChecksum(const uint8_t *data, const uint32_t size)
{
	return calcCrc32_32(data, size);
}
