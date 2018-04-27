#pragma once

#include <zmq.hpp>
#include <string>
#include <iostream>
#include <mutex>

using namespace std;
using namespace zmq;

class ZMQCommunicator
{
public:

	socket_t m_serverSocket;
	socket_t m_clientSocket;

	string m_receiveMessage;
	string m_sendMessage;

	mutex m_mutexForServer;
	mutex m_mutexForClient;

	ZMQCommunicator() = delete;
	ZMQCommunicator(context_t& context);
	~ZMQCommunicator();

	inline const mutex& get_mutex_for_server() const { return m_mutexForServer; }
	inline const mutex& get_mutex_for_client() const { return m_mutexForClient; }

	inline void bind_server(const string& address) { m_serverSocket.bind(address); }
	inline void connect_client(const string& address) { m_clientSocket.connect(address); }

	//For server
	string receive_message();
	bool reply_to_client(const string& msg);

	//For client
	bool send_message(const string& msg);
	string receive_reply();
};

