#include "lora.hpp"

#include <stdio.h>
#include <iostream>
#include <iterator>

#include <termios.h>
#include <sys/ioctl.h>

LoRaUart::LoRaUart()
{
	fileDescription = 0;
}

LoRaUart::~LoRaUart()
{
	closeConnection();
}

bool LoRaUart::openConnection(std::string port)
{  
	if(fileDescription > 0) closeConnection();

	fileDescription = open(port.data(), O_RDWR | O_NOCTTY | O_SYNC);

	if(fileDescription <=0)
	{
		std::cerr << "Error opening serial port: " << errno << std::endl;
		return false;
	}

	ioctl(fileDescription, TIOCEXCL);

	std::cout << "Open serial port: " << fileDescription << std::endl;

	setInterfaceAttribs(B115200);  // set speed to 115200 bps, 8n1 (no parity)

	//rxArray.clear();

	return true;
}

/**
 * @brief Close the port
 *
 */
void LoRaUart::closeConnection()
{
	rxArray.clear();
	std::cout << "Closing serial port: " << fileDescription << std::endl;
	fileDescription = close(fileDescription);
}

/**
 * @brief Send a command
 *
 * @param data Pointer to the data to send
 */
size_t LoRaUart::sendData(char *data, int len)
{
	rxArray.clear();

	return write(fileDescription, data, len);
}

/**
 * @brief Slot called on reception
 *
 * This function is called, when data from the serial port has been received.
 */
int LoRaUart::readData(int data_len)
{
	/*
	char buf[data_len] = { 0 };
	int n = read(fileDescription, buf, data_len);
	
	if (n == -1) {
		std::cerr << "Error on readData." << std::endl;
		return -1;
	}
	
	rxArray.insert(rxArray.end(), buf, buf + n);

	return n;
	*/

	rxArray.clear();
	
	char buffer[1024] = {0};
	int pos = 0;

	while( pos < 1023 ) {
		read(fileDescription, buffer+pos, 1);           // Note you should be checking the result
		if( buffer[pos] == '\n' ) break;
		pos++;
	}
	rxArray.insert(rxArray.end(), buffer, buffer + pos);

	return pos;
}

int LoRaUart::setInterfaceAttribs(int speed)
{
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

	return 0;
}

