#include "ZMQCommunicator.h"


ZMQCommunicator::ZMQCommunicator(context_t& context)
	:m_serverSocket(context, ZMQ_REP), m_clientSocket(context, ZMQ_REQ)
{
}



ZMQCommunicator::~ZMQCommunicator()
{
}



string ZMQCommunicator::receive_message()
{
	message_t request;
	m_serverSocket.recv(&request);
	char* msg = new char[request.size() + 1];
	memcpy(msg, request.data(), request.size());
	msg[request.size()] = '\0';	//END OF MESSAGE
	m_receiveMessage = string(msg);
	delete[] msg;
	return m_receiveMessage;
}



bool ZMQCommunicator::reply_to_client(const string& msg)
{
	message_t message(msg.size());
	memcpy(message.data(), msg.data(), msg.size());
	bool result = m_serverSocket.send(message);
	return result;
}



bool ZMQCommunicator::send_message_and_receive_reply(const string& msg, string& reply)
{
	bool result = send_message(msg);
	if (result == true)
		reply = receive_reply();

	return result;
}



bool ZMQCommunicator::send_message(const string& msg)
{
	message_t message(msg.size());
	memcpy(message.data(), msg.data(), msg.size());
	bool result = m_clientSocket.send(message);
	return result;
}



string ZMQCommunicator::receive_reply()
{
	message_t request;
	m_clientSocket.recv(&request);
	char* msg = new char[request.size() + 1];
	memcpy(msg, request.data(), request.size());
	msg[request.size()] = '\0';	//END OF MESSAGE
	string reply(msg);
	delete[] msg;
	return reply;
}
