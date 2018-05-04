// ZMQCommunicator.cpp: 콘솔 응용 프로그램의 진입점을 정의합니다.
//

#include "zmq.hpp"
#include "ZMQCommunicator.h"
#include <string>
#include <windows.h>
#include <thread>
#include <mutex>

using namespace std;
using namespace zmq;

ZMQCommunicator* communicator;
mutex senderMutex;
mutex receiverMutex;

string sendingMessage;
string receivingMessage;

void run_sender()
{
	while (true)
	{
		if (sendingMessage.size() > 0)
		{
			if (senderMutex.try_lock())
			{
				cout << "Send message: " << sendingMessage << endl;
				communicator->send_message(sendingMessage);
				string reply = communicator->receive_reply();
				cout << "Received reply: " << reply << endl;
				sendingMessage = "";
				senderMutex.unlock();
			}
		}

		Sleep(1000);
	}
}



void run_receiver()
{
	while (true)
	{
		cout << "Waiting for receiving message..." << endl;
		receivingMessage = communicator->receive_message();
		cout << "Received message: " << receivingMessage << endl;
		communicator->reply_to_client("Good!");
		cout << "Cycle completed" << endl << endl;

		Sleep(1000);
	}
}


void send_message(const string& message)
{
	while (!senderMutex.try_lock())
		Sleep(1000);

	sendingMessage = message;
	senderMutex.unlock();
}



int main()
{
	context_t context(2);
	communicator = new ZMQCommunicator(context);

	string pyClientAddress("tcp://*:5555");
	string pyServerAddress("tcp://localhost:5556");

	communicator->bind_server(pyClientAddress);
	communicator->connect_client(pyServerAddress);

	thread senderThread(&run_sender);
	thread receiverThread(&run_receiver);

	bool onRunning = true;
	while (onRunning)
	{
		int order = 0;
		cout << endl;
		cout << "1. Connect" << endl;
		cout << "2. Arm" << endl;
		cout << "3. Take off" << endl;
		cout << "4. Go to" << endl;
		cout << "5. Land" << endl;
		cout << "6. Get battery status" << endl;
		cout << "7. Get gps" << endl;
		cout << "0. Terminate program" << endl;
		cout << "Input order: ";
		cin >> order;

		switch (order)
		{
		case 1:
			send_message("connect 0 192.168.4.2:14550");
			break;
		case 2:
			send_message("arm 0");
			break;
		case 3:
			send_message("takeoff 0 5");
			break;
		case 4:
			send_message("goto 0 5 -34.364114 149.166022 5");
			break;
		case 5:
			send_message("land 0");
			break;
		case 6:
			send_message("battery 0");
			break;
		case 7:
			send_message("location_global 0");
			break;
		case 0:
			cout << "Terminate program" << endl;
			onRunning = false;
			break;
		default:
			cout << "Try again" << endl;
			break;
		}
	}

    return 0;
}

