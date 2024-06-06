#include "SerialConnection.hpp"

#include <stdio.h>
#include <iostream>
#include <iterator>

#ifndef _WIN32
#include <termios.h>
#include <sys/ioctl.h>
#endif

#include <util.h>

using namespace std;

SerialConnection::SerialConnection(std::string _port) : port(_port)
{
#ifdef _WIN32
#else
	fileDescription = 0;
#endif
}

SerialConnection::~SerialConnection()
{
	closeConnection();
}

bool SerialConnection::openConnection()
{  
#ifdef _WIN32
	std::cout << "Try to open serial port: " << port << std::endl;
	serialHandle = CreateFile(port.data(), GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);
	if (serialHandle == INVALID_HANDLE_VALUE) {
		std::cerr << "Error opening serial port: " << port << std::endl;
		return false;
	}
	else {
		std::cout << "Serial port: " << serialHandle << " opened." << std::endl;
	}

#else
	if(fileDescription > 0) closeConnection();

	fileDescription = open(port.data(), O_RDWR | O_NOCTTY | O_SYNC);

	if(fileDescription <=0)
	{
		std::cerr << "Error opening serial port: " << errno << std::endl;
		return false;
	}
	ioctl(fileDescription, TIOCEXCL);

	//std::cout << "Open serial port: " << fileDescription << std::endl;
#endif

#ifdef _WIN32
	setInterfaceAttribs(4000000);  // set speed to 10000000 bps, 8n1 (no parity)
#else
	setInterfaceAttribs(B4000000);  // set speed to 10000000 bps, 8n1 (no parity)
#endif

	rxArray.clear();

	return true;
}

/**
 * @brief Close the port
 *
 */
void SerialConnection::closeConnection()
{
#ifdef _WIN32
	CloseHandle(serialHandle);
#else
	//std::cout << "Closing serial port: " << fileDescription << std::endl;
	fileDescription = close(fileDescription);
#endif
}

/**
 * @brief Send a command
 *
 * This function sends a command. It adds the marking and the checksum and sends it
 * to the device.
 *
 * Important: The size of the buffer must be big enough to add the markings an the checksum.
 *
 * @param data Pointer to the data to send
 */
size_t SerialConnection::sendData(uint8_t *data)
{
	//if(fileDescription <=0 ){
	//	std::cerr << "Error sendData fileDescription =0 \n" << std::endl;
	//	return 0;
	//}

	//Add the start buffer at the beginning
	data[0] = Constants::Command::START_MARK;

	//Calculate the CRC
	uint32_t crc = calculateChecksum(data, (Constants::Command::SIZE_TOTAL - sizeof(uint32_t)));

	//Add it to the buffer123
	setUint32LittleEndian(data, (Constants::Command::SIZE_TOTAL - sizeof(uint32_t)), crc);

	//This is just to print out the data
	//std::string str= "SEND DATA: ";
	//char buf[4];
	//for(int i=0; i<14; i++){
	//	sprintf(buf, "%x ", data[i]);
	//	str.append(buf);
	//}
	//std::cout << str << std::endl;

#ifdef _WIN32
	DWORD dwBytesWritten = 0;
	int status = WriteFile( serialHandle, (uint8_t *)(data), Constants::Command::SIZE_TOTAL, &dwBytesWritten, NULL );
	//std::cout << "write status: " << status << std::endl;
	//std::cout << "write dwBytesWritten: " << dwBytesWritten << std::endl;
	if (status != 0)
	{
		return (size_t) dwBytesWritten;
	}
	return -1;

#else
	return write(fileDescription, (uint8_t *)(data), Constants::Command::SIZE_TOTAL);
#endif
}

/**
 * @brief Slot called on reception
 *
 * This function is called, when data from the serial port has been received.
 */
int SerialConnection::readData(int readSize, int totalSize)
{
	int n = 0;
	uint8_t buf[50000] = { 0 };

#ifdef _WIN32
	DWORD dwBytesRead = 0;
	int status = ReadFile(serialHandle, buf, readSize, &dwBytesRead, NULL);
	//std::cout << "read status: " << status << std::endl;
	//std::cout << "read readSize: " << readSize << std::endl;
	//std::cout << "read totalSize: " << totalSize << std::endl;
	//std::cout << "read dwBytesRead: " << dwBytesRead << std::endl;
	//if (status != 0)
	//{
		n = (int) dwBytesRead;
	//} else
	//{
	//	n = -1;
	//}
	//std::cout << "read n: " << n << std::endl;

	if (n == -1) {
		std::cerr << "Error on  SerialConnection::readRxData." << std::endl;
		return -1;
}

	rxArray.insert(rxArray.end(), buf, buf + n); //Append the new data to the rxArray buffer

#else
	n = read(fileDescription, buf, readSize);

	if (n == -1) {
		//std::cerr << "Error on  SerialConnection::readRxData." << std::endl;
		return -1;
	}

	rxArray.insert(rxArray.end(), buf, buf + n); //Append the new data to the rxArray buffer

#endif

	if (receiveDataSegment(rxArray, totalSize))
	{
		//std::cout << "read valid n: " << n << std::endl;
		return n;
	} else
	{
		//std::cout << "read invalid n: " << n << std::endl;
		return -1;
	}
}

int SerialConnection::setInterfaceAttribs(int speed)
{
#ifdef _WIN32
	int status = 0;
	
	DCB serialParams = { 0 };
	serialParams.DCBlength = sizeof(serialParams);

	GetCommState(serialHandle, &serialParams);
	serialParams.BaudRate = (DWORD)0x4000000;
	serialParams.ByteSize = DATABITS_8;
	serialParams.StopBits = ONESTOPBIT;
	serialParams.Parity = NOPARITY;
	status = SetCommState(serialHandle, &serialParams);
	if (status == 0) {
		printf("Error setting comm params: %d", (int) GetLastError());
	}

	COMMTIMEOUTS        commtimeouts;

	status = GetCommTimeouts (serialHandle, &commtimeouts);
	if (status == 0) {
		printf("Error getting comm timeouts: %d", (int) GetLastError());
	}
	ZeroMemory(&commtimeouts, sizeof(commtimeouts));
	/*
	commtimeouts.ReadIntervalTimeout        = 0;
	commtimeouts.ReadTotalTimeoutConstant   = MAXDWORD;//30;
	commtimeouts.ReadTotalTimeoutMultiplier = MAXDWORD;//0;
	*/
	//commtimeouts.ReadIntervalTimeout = 1;
	//commtimeouts.ReadTotalTimeoutConstant = 0;
	//commtimeouts.ReadTotalTimeoutMultiplier = 0;

	commtimeouts.ReadIntervalTimeout = 0;
	commtimeouts.ReadTotalTimeoutConstant = 30;
	commtimeouts.ReadTotalTimeoutMultiplier = 0;

	commtimeouts.WriteTotalTimeoutConstant   = 0;
	commtimeouts.WriteTotalTimeoutMultiplier = 0;

	status = SetCommTimeouts (serialHandle, &commtimeouts);
	if (status == 0) {
		printf("Error setting comm timeouts: %d", (int) GetLastError());
	}
#else
	tcflush(fileDescription, TCIOFLUSH);

	struct termios tty;
	memset (&tty, 0, sizeof tty);
	bzero(&tty, sizeof(struct termios));

	cfsetospeed (&tty, speed);
	cfsetispeed (&tty, speed);

	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
	// disable IGNBRK for mismatched speed tests; otherwise receive break
	// as \000 chars
	tty.c_iflag &= ~IGNBRK;         // disable break processing
	tty.c_lflag = 0;                // no signaling chars, no echo,
	// no canonical processing
	tty.c_oflag = 0;                // no remapping, no delays
	tty.c_cc[VMIN]  = 1;            // read block
	tty.c_cc[VTIME] = 30;           // 0.5 seconds read timeout

	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl
	tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
	// enable reading
	tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
	//tty.c_cflag |= parity;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CRTSCTS;

	if (tcsetattr (fileDescription, TCSANOW, &tty) != 0){
		std::cerr << "Error from tcsetattr: " << errno << std::endl;
		return -1;
	}
#endif

	return 0;
}

