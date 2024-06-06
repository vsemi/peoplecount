#include "MJPGStreamer.h"

#include <stdio.h>
#include <iostream>
#include <string>

using std::cerr;
using std::endl;

MJPGStreamer::Client::Client(int id)
{
	_id = id;
	_alive = true;
}
MJPGStreamer::Client::~Client()
{
}
void MJPGStreamer::Client::setStreamer(MJPGStreamer* streamer)
{
	_streamer = streamer;
}
bool  MJPGStreamer::Client::isLive()
{
	return _alive;
}
void MJPGStreamer::Client::start()
{
	m_thread_client_writer = std::thread([](Client* context) {
		context->run();
	}, this);
	m_thread_client_writer.detach();
}
void MJPGStreamer::Client::run()
{
	std::cout << "New client thread: " << _id << std::endl;
	std::unique_lock<std::mutex> lck(mutex_client_writer);

	while (_alive)
	{
		try 
		{
			condition_variable_client_writer.wait(lck);

			if (! _alive) break;
			
			if (buf_len > 0)
			{
				std::string s_head = "--mjpegstream\r\nContent-Type: image/jpeg\r\nContent-Length: " + std::to_string(buf_len) + "\r\n\r\n";
				char* head = const_cast<char*> (s_head.c_str());
				int r = _streamer->_write(_id, head, 0);
				if (r < 0)
				{
					break;
				}
				r = _streamer->_write(_id, (char*)(&outbuf[0]), buf_len);
				if (r < 0)
				{
					break;
				}
			}
		}
		catch (...)
		{
			std::cerr << "Client thread error, id: " << _id << std::endl;
		}
	}
	std::cout << "Client thread completed: " << _id << std::endl;
}
void MJPGStreamer::Client::stop()
{
	_alive = false;
	std::cout << "To stop client thread _alive: " << _alive << std::endl;
}
void MJPGStreamer::Client::feed_data(std::vector<uchar> data, int len)
{
	if (_alive)
	{
		std::unique_lock<std::mutex> lck(mutex_client_writer);
		outbuf.clear();
		outbuf = data;
		buf_len = len;
		condition_variable_client_writer.notify_all();
	}
}

MJPGStreamer::MJPGStreamer()
: sock(INVALID_SOCKET)
, timeout(200000)
, quality(61.8) {
}

MJPGStreamer::~MJPGStreamer()
{
}

void MJPGStreamer::setQuality(float quality)
{
	this->quality = quality;
}

bool MJPGStreamer::open()
{
	sock = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

	SOCKADDR_IN address;
	address.sin_addr.s_addr = INADDR_ANY;
	address.sin_family = AF_INET;

	address.sin_port = htons(port);

	if (::bind(sock, (SOCKADDR*)&address, sizeof(SOCKADDR_IN)) == SOCKET_ERROR)
	{
		cerr << "error : couldn't bind sock " << sock << " to port " << port << "!" << endl;
		return false;
	}
	if (::listen(sock, NUM_CONNECTIONS) == SOCKET_ERROR)
	{
		cerr << "error : couldn't listen on sock " << sock << " on port " << port << " !" << endl;
		return false;
	}
	FD_SET(sock, &master);

	const char       optVal = 1;
	const int optLen = sizeof(optVal);

	setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &optVal, optLen);

	std::cout << "MJPG streaming opened on sock: " << sock << " port: " << port << std::endl;

	return true;
}

bool MJPGStreamer::isOpened()
{
	return sock != INVALID_SOCKET;
}

void MJPGStreamer::start(int p) {
	port = p;

	FD_ZERO(&master);

	open();

	thread_listener = std::thread([](MJPGStreamer* context) {
		context->listen_new_client();
	}, this);
	thread_listener.detach();

	m_thread_writer = std::thread([](MJPGStreamer* context) {
		context->clientWrite();
	}, this);
	m_thread_writer.detach();

	m_thread_dead_conn_collector = std::thread([](MJPGStreamer* context) {
		context->clean_dead_connections();
		}, this);
	m_thread_dead_conn_collector.detach();
}

void MJPGStreamer::stop() {
	m_stopped = true;

	condition_variable_writer.notify_all();
	condition_dead_conn_collector.notify_all();

	std::cout << "MJPGStreamer stop lister and writer ..." << std::endl;

	std::cout << "MJPGStreamer shutdown all clients ..." << std::endl;
	std::map<int, std::shared_ptr<Client>>::iterator begin = m_clients.begin();
	std::map<int, std::shared_ptr<Client>>::iterator end = m_clients.end();

	for (std::map<int, std::shared_ptr<Client>>::iterator it = begin; it != end; ++it) {
		int a_client = it->first;
		std::cout << "   shutdown client socket: " << a_client << std::endl;
		it->second->stop();
		::shutdown(a_client, 2);
		FD_CLR(a_client, &master);
		std::cout << "   client socket: " << a_client << " has been shutdown." << std::endl;
	}

	std::cout << "MJPGStreamer shutdown master socket ..." << std::endl;
	if (sock != INVALID_SOCKET)
		::shutdown(sock, 2);
	sock = (INVALID_SOCKET);

	std::cout << "MJPGStreamer has been shutdown." << std::endl;
}

void MJPGStreamer::listen_new_client() 
{
	fd_set rread;
	SOCKET maxfd;

	while (!m_stopped) 
	{
		try {
			rread = master;
			struct timeval to = { 0,timeout };
			maxfd = sock + 1;

			int sel = select(maxfd, &rread, NULL, NULL, &to);

			if (sel > 0)
			{

				for (int s = 0; s < maxfd; s++)
				{
					if (m_stopped) break;

					if (!FD_ISSET(s, &rread)) {
						continue;
					}

					if (s == sock)
					{

						socklen_t addrlen = sizeof(SOCKADDR);

						SOCKADDR_IN address = { 0 };
						int sock_int = sock;

						SOCKET      client = ::accept(sock_int, (SOCKADDR*)&address, &addrlen);

						std::cout << "sock_int: " << sock_int << " maxfd: " << maxfd << " client: " << client << " addrlen: " << addrlen << std::endl;

						if (client == SOCKET_ERROR)
						{
							std::cerr << "error : couldn't accept connection on sock " << sock << " !" << std::endl;
						}
						else
						{
							maxfd = (maxfd > client ? maxfd : client);
							FD_SET(client, &master);

							std::vector<char> buf(5000); // you are using C++ not C
							int bytes = ::recv(client, buf.data(), buf.size(), 0);
							//std::cout << "==>::recv from sock: " << bytes << " data: \n" << buf.data() << std::endl;
							//std::cout << buf.data() << std::endl;

							if (m_stopped) break;
							
							std::cout << "request:\n\n " << buf.data() << std::endl;

							char pt[] = "HTTP/1.0 200 OK\r\n";
							int r = _write(client, pt, 0);
							//std::cout << "==>_write to sock: " << r << std::endl;
							if (r < 0) continue;

							char ct[] =
								"Server: Mozarella/2.2\r\n" \
								"Accept-Range: bytes\r\n" \
								"Connection: close\r\n" \
								"Max-Age: 0\r\n" \
								"Expires: 0\r\n" \
								"Cache-Control: no-cache, private\r\n" \
								"Pragma: no-cache\r\n" \
								"Content-Type: multipart/x-mixed-replace; boundary=mjpegstream\r\n" \
								"\r\n";
							r = _write(client, ct, 0);
							//std::cout << "==>_write to sock: " << r << std::endl;
							if (r < 0) continue;

							std::cout << "new client: " << client << std::endl;

							std::unique_lock<std::mutex> lock_clients(mutex_clients);
							
							std::cout << "Obtain lock for new client: " << client << std::endl;
							int client_int = client;
							std::shared_ptr<Client> c = std::shared_ptr<Client>(new Client(client_int));
							c->setStreamer(this);
							c->start();
							m_clients.insert(m_clients.begin(), std::pair<int, std::shared_ptr<Client>>(client_int, c));
							std::cout << "New client: " << client << " added" << std::endl;
							lock_clients.unlock();
							std::cout << "Total active clients: " << m_clients.size() << std::endl;
						}
					}
				}
			}
		}
		catch (std::exception& e)
		{
			cerr << "MJPG Streamer listen_new_client Error: " << endl;
			cerr << e.what() << endl;
		}
	}

	std::cout << "New client listener stopped." << std::endl;
}

int MJPGStreamer::_write(int sock, char* s, int len)
{
	int result = -1;
	try {
		if (len < 1) { len = strlen(s); }
		std::cout << "   _write -> sock: " << sock << " len: " << len << std::endl;
		result = ::send(sock, s, len, MSG_NOSIGNAL);
		//int nbytes = send(fd,msg.c_str(), msg.size(), MSG_NOSIGNAL);
		std::cout << "   _write -> sock: " << sock << " result: " << result << std::endl;

		if (result < len)
		{
			cerr << "   =>client disconnected: " << sock << endl;
			std::cout << "   shutdown client: " << sock << std::endl;
			std::unique_lock<std::mutex> lck(mutex_dead_conn_collector);
			
			if (auto c = m_clients.find(sock); c != m_clients.end()) {
				// found
				m_clients[sock]->stop();
				std::cout << "   shutdown sock: " << sock << std::endl;

				std::shared_ptr<MJPGStreamer::Client> client = c->second;
				if (client->isLive()) client->stop();
				
				//std::cout << "   pusdh to remove client: " << sock << std::endl;
				dead_connections.push_back(sock);
				//std::cout << "   pusdh to remove client done: " << sock << std::endl;
				condition_dead_conn_collector.notify_all();
				//std::cout << "   pusdh to remove client notified: " << sock << std::endl;

				::shutdown(sock, 2);
				FD_CLR(sock, &master);
				std::cout << "   client socket: " << sock << " has been shutdown." << std::endl;
				std::cout << "   total clients alive: " << m_clients.size() << std::endl;
			}
			else {
				// not found
				std::cout << "   client: " << sock << " somehow not found" << std::endl;
			}
			condition_dead_conn_collector.notify_all();
		}
	}
	catch ( ... ) {
		cerr << "Error _write client: " << sock << endl;
	}

	std::cout << "   _write done, sock: " << sock << std::endl;

	return result;
}

void MJPGStreamer::write(const cv::Mat & frame)
{
	try {
		std::unique_lock<std::mutex> lck(mutex_writer);
		if (!frame.empty()) {
			lastFrame.release();
			lastFrame = frame.clone();
		}
		condition_variable_writer.notify_all();
	}
	catch (std::exception& e)
	{
		cerr << "MJPG Streamer write Error: " << endl;
		cerr << e.what() << endl;
	}
}

void MJPGStreamer::clientWrite() {

	std::unique_lock<std::mutex> lck(mutex_writer);

	while (! m_stopped)
	{
		try {
			condition_variable_writer.wait(lck);

			if (m_stopped) break;

			if (!lastFrame.empty()) {

				std::vector<uchar> outbuf;
				std::vector<int> params;
				params.push_back(cv::IMWRITE_JPEG_QUALITY);
				params.push_back(quality);
				cv::imencode(".jpg", lastFrame, outbuf, params);
				int outlen = outbuf.size();

				std::unique_lock<std::mutex> lock_clients(mutex_clients);

				std::map<int, std::shared_ptr<Client>>::iterator begin = m_clients.begin();
				std::map<int, std::shared_ptr<Client>>::iterator end = m_clients.end();
				
				for (std::map<int, std::shared_ptr<Client>>::iterator it = begin; it != end; ++it) {
					int a_client = it->first;
					if (it->second->isLive()) it->second->feed_data(outbuf, outlen);
				}

				lock_clients.unlock();
				
			}
		}
		catch (std::exception& e)
		{
			cerr << "MJPG Streamer clientWrite Error: " << endl;
			cerr << e.what() << endl;
		}
	}

	std::cout << "Client Writer stopped." << std::endl;
}

void MJPGStreamer::clean_dead_connections()
{
	std::unique_lock<std::mutex> lck(mutex_dead_conn_collector);

	while (!m_stopped)
	{
		try {
			//std::cout << "   waiting for lock to remove client ... " << std::endl;
			condition_dead_conn_collector.wait(lck);

			if (m_stopped) break;
			
			std::cout << "   to remove disconnected client ... " << std::endl;

			//std::cout << "   dead_connections.size(): " << dead_connections.size() << std::endl;
			if (dead_connections.size() > 0) {
				//std::cout << "   to obtain lock for  mutex_clients ... " << std::endl;
				std::unique_lock<std::mutex> lock_clients(mutex_clients);
				//std::cout << "   lock for  mutex_clients obtained " << std::endl;
				for (size_t i = 0; i < dead_connections.size(); i++)
				{
					std::cout << "   remove client: " << dead_connections[i] << std::endl;
					m_clients.erase(dead_connections[i]);
					dead_connections.erase(dead_connections.begin() + i);
					i--;
					//std::cout << "   client removed " << std::endl;
				}
				lock_clients.unlock();
			}
		}
		catch (std::exception& e)
		{
			cerr << "MJPG Streamer clean_dead_connections Error: " << endl;
			cerr << e.what() << endl;
		}
	}

	std::cout << "clean_dead_connections thread stopped." << std::endl;
}
