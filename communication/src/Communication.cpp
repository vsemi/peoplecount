#include "Communication.hpp"

#ifdef _WIN32
#include <windows.h>
#endif

#include <iostream>
#include <cstring>

#include "constants.h"
#include "util.h"

using namespace std;

Communication::Communication(Connection *_connection) : connection(_connection)
{
}
Communication::~Communication()
{
	delete connection;
}

/**
 * @brief Open the serial port
 *
 * @param id Id of the serial port = index of the list when the function "availableDevices" is called
 * @retval true Port is open
 * @retval false Port is not open
 * @return Port open or not
 */
bool Communication::open()
{
	bool connected = connection->openConnection();

	if(! connected){
		return false;
	}

	return connected;
}

/**
 * @brief Close the serial port
 *
 */
void Communication::close()
{
	connection->closeConnection();
}

/**
 * @brief Enable or disable power
 *
 * @param enabled True to enable the power, else false
 * @return Error code from ErrorNumber_e
 */
ErrorNumber_e Communication::setPower(const bool enabled)
{
	uint8_t controlByte = 0;

	//Set the control byte --> true != 1
	if (enabled)  controlByte = 1;

	return sendCommandSingleByte(Constants::CommandList::COMMAND_SET_POWER, controlByte);
}

/**
 * @start streaming
 *
 */
void Communication::startStream(){
	startStreamMode = true;
}

/**
 * @brief helper function to send the command
 *
 * All commands are sent over this function. Here also the timeouts
 * are handled.
 *
 * @param data Pointer to the already filled data to send
 * @param blocking set to true to run command blocking
 * @return Error code from ErrorNumber_e
 */
ErrorNumber_e Communication::sendCommand(uint8_t *data, int size, bool streamMode)
{  
	if(!streamMode || startStreamMode){
		connection->sendData(data);
		startStreamMode = false;
	}

	int sz = 0;
	int count = 0;
	connection->clearRxArray();

	for(int n= 0; n < size; n+= sz){
		//sz = connection->readData(size);
		//sz = connection->readData(size - count, size);
		//int read_bytes = size - count;
		//if (read_bytes > 128) read_bytes = 128;
		//sz = connection->readData(read_bytes, size);
		sz = connection->readData(size, size);
		//sz = connection->readData(size + Constants::Data::SIZE_OVERHEAD);
		// + SIZE_OVERHEAD
		if(sz == -1){
			//std::cerr << "Communication::sendCommand connection->readRxData sz=-1 count = " << count << std::endl;
			return ERROR_NUMBER_IO_ERROR;
		}else{
			count += sz;
		}
	}

	return ERROR_NUMMBER_NO_ERROR;
}

/**
 * @brief Send a command without data
 *
 * This function is used for commands without any payload.
 *
 * @param command Command to send
 * @param blocking set to true to run command blocking
 * @return Error code from ErrorNumber_e
 */
ErrorNumber_e Communication::sendCommandWithoutData(const uint8_t command, int size, bool streamMode)
{
	uint8_t output[Constants::Command::SIZE_TOTAL];

	memset(output, 0, sizeof(output));

	//Add the command
	output[Constants::Command::INDEX_COMMAND] = command;

	return sendCommand(output, size, streamMode);
}

/**
 * @brief Send single byte command
 *
 * This function is used for commands with only one byte of payload.
 *
 * @param command Command to send
 * @param payload Payload to send
 * @param blocking set to true tur run command blocking
 * @return Error code from ErrorNumber_e
 */
ErrorNumber_e Communication::sendCommandSingleByte(const uint8_t command, const uint8_t payload, int size, bool streaming)
{
	uint8_t output[Constants::Command::SIZE_TOTAL];

	memset(output, 0, sizeof(output));

	//Add the command
	output[Constants::Command::INDEX_COMMAND] = command;

	//Add the single byte at the first position
	output[Constants::Command::INDEX_DATA] = payload;

	return sendCommand(output, size, streaming);
}

/**
 * @brief Send command with 2 bytes payload
 *
 * This function is used for commands with two bytes of payload
 *
 * @param command Command to send
 * @param payload0 Payload byte 0 to send
 * @param payload1 Payload byte 1 to send
 * @param blocking set to true tur run command blocking
 * @return Error code from ErrorNumber_e
 */
ErrorNumber_e Communication::sendCommand2xByte(const uint8_t command, const uint8_t payload0, const uint8_t payload1, uint8_t type, bool blocking)
{
	uint8_t output[Constants::Command::SIZE_TOTAL];

	memset(output, 0, sizeof(output));

	//Add the command
	output[Constants::Command::INDEX_COMMAND] = command;

	//Add the first byte
	output[Constants::Command::INDEX_DATA] = payload0;

	//Add the second byte
	output[Constants::Command::INDEX_DATA+1] = payload1;

	return sendCommand(output, type, blocking);
}

/**
 * @brief Send 16bit / 2byte command
 *
 * This function is used for commands with one 16bit value as payload
 *
 * @param command Command to send
 * @param payload Payload to send
 * @param blocking set to true tur run command blocking
 * @return Error code from ErrorNumber_e
 */
ErrorNumber_e Communication::sendCommandUint16(const uint8_t command, const uint16_t payload)
{
	uint8_t output[Constants::Command::SIZE_TOTAL];

	memset(output, 0, sizeof(output));

	//Add the command
	output[Constants::Command::INDEX_COMMAND] = command;

	//Add the payload
	setUint16LittleEndian(output, Constants::Command::INDEX_DATA, payload);

	return sendCommand(output, Constants::Command::SIZE_PAYLOAD);
}

/**
 * @brief Send 16bit / 2byte command signed
 *
 * This function is used for commands with one 16bit value as payload
 *
 * @param command Command to send
 * @param payload Payload to send
 * @param blocking set to true tur run command blocking
 * @return Error code from ErrorNumber_e
 */
ErrorNumber_e Communication::sendCommandInt16(const uint8_t command, const int16_t payload)
{
	uint8_t output[Constants::Command::SIZE_TOTAL];

	memset(output, 0, sizeof(output));

	//Add the command
	output[Constants::Command::INDEX_COMMAND] = command;

	//Add the payload
	setInt16LittleEndian(output, Constants::Command::INDEX_DATA, payload);

	return sendCommand(output, Constants::Command::SIZE_PAYLOAD);
}

/**
 * @brief Send 2 x 16bit / 4byte command
 *
 * This function is used for commands with two 16bit value as payload
 *
 * @param command Command to send
 * @param payload0 First payload value to send
 * @param payload1 Second payload value to send
 * @param blocking set to true tur run command blocking
 * @return Error code from ErrorNumber_e
 */
ErrorNumber_e Communication::sendCommand2xUint16(const uint8_t command, const uint16_t payload0, const uint16_t payload1)
{
	uint8_t output[Constants::Command::SIZE_TOTAL];

	memset(output, 0, sizeof(output));

	//Add the command
	output[Constants::Command::INDEX_COMMAND] = command;

	//Add the payload values
	setUint16LittleEndian(output, Constants::Command::INDEX_DATA, payload0);
	setUint16LittleEndian(output, Constants::Command::INDEX_DATA+2, payload1);

	return sendCommand(output, Constants::Command::SIZE_PAYLOAD);
}

std::vector<uint8_t> Communication::data() {
	return connection->data();
}
