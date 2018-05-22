#pragma once

#include <zmq.hpp>
#include <string>
#include <iostream>
#include <mutex>

using namespace std;

class ZMQCommunicator
{
private:
	zmq::context_t context;
	zmq::socket_t m_serverSocket;
	zmq::socket_t m_clientSocket;

	string m_receiveMessage;
	mutex m_mutexForServer;

public:
	ZMQCommunicator();
	~ZMQCommunicator();

	inline const mutex& get_mutex_for_server() const { return m_mutexForServer; }

	inline void bind_server(const string& address) { m_serverSocket.bind(address); }
	inline void connect_client(const string& address) { m_clientSocket.connect(address); }

	inline void unbind_server(const string& address) { m_serverSocket.unbind(address); }
	inline void disconnect_client(const string& address) { m_clientSocket.disconnect(address); }

	//For server
	string receive_message();
	bool reply_to_client(const string& msg);

	//For client
	bool send_message_and_receive_reply(const string& msg, string& reply);
	bool send_message(const string& msg);
	string receive_reply();
};

