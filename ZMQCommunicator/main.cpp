// ZMQCommunicator.cpp: 콘솔 응용 프로그램의 진입점을 정의합니다.
//

#include "stdafx.h"
#include "zmq.hpp"
#include "ZMQCommunicator.h"
#include <string>
#include <windows.h>

using namespace std;
using namespace zmq;

int main()
{
	context_t context(2);
	ZMQCommunicator communicator(context);

	string pyClientAddress("tcp://*:5555");
	string pyServerAddress("tcp://localhost:5556");

	communicator.bind_server(pyClientAddress);
	communicator.connect_client(pyServerAddress);

	for (int i=0; i<100; i++)
	{
		cout << "Send message: " << i << endl;
		communicator.send_message(string("Vero_")+to_string(i));
		string reply = communicator.receive_reply();
		cout << "Received reply: " << reply << endl;

		Sleep(1000);

		cout << "Waiting for receiving message..." << endl;
		string message = communicator.receive_message();
		cout << "Received message: " << message << endl;
		communicator.reply_to_client(string("Reply_") + to_string(i));
		cout << "Cycle completed" << endl << endl;
	}

    return 0;
}

