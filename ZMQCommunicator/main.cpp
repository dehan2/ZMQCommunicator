// ZMQCommunicator.cpp: 콘솔 응용 프로그램의 진입점을 정의합니다.
//

#include "zmq.hpp"
#include "ZMQCommunicator.h"
#include <string>
#include <windows.h>
#include <thread>
#include <mutex>
#include <array>
#define _USE_MATH_DEFINES 
#include <math.h>

using namespace std;
using namespace zmq;

string send_message(ZMQCommunicator& communicator, const string& message)
{
	string reply;
	bool result = communicator.send_message_and_receive_reply(message, reply);
	return reply;
}


int main()
{
	ZMQCommunicator communicator;

	string pyServerAddress = string("tcp://localhost:5556");

	//communicator.bind_server(pyClientAddress);
	communicator.connect_client(pyServerAddress);

	//thread senderThread(&run_sender);
	//thread receiverThread(&run_receiver);

	bool onRunning = true;
	while (onRunning)
	{
		string message;
		cout << "Enter your order: ";
		getline(cin, message);
		string reply = send_message(communicator, message);
		cout << "Reply: " << reply << endl;

		/*int order = 0;
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
		}*/
	}

    return 0;
}

