import zmq
import time
from threading import *
from PyZMQCommunicator import PyZMQCommunicator
from DroneCommander import *

def run_receiver_parallel(communicator, drones):
    while True:
        print("Waiting for receiving message...")
        message = communicator.receive()
        print("Received msg: %s" % message)
        replyMessage = command_to_drone_parallel(message, drones)
        communicator.reply_to_client("Well received: %s" % replyMessage)
        time.sleep(0.1)


def run_receiver_serial(communicator, droneAddresses):
    while True:
        print("Waiting for receiving message...")
        message = communicator.receive()
        print("Received msg: %s" % message)
        replyMessage = command_to_drone_serial(message, droneAddresses)
        communicator.reply_to_client("Well received: %s" % replyMessage)
        time.sleep(0.1)


def command_to_drone_parallel(message, drones):
    commandList = message.split();
    droneID = int(commandList[1])

    if commandList[0]=='connect':
        address = commandList[2]
        drones.append(connect(address))
        return "Connected"

    if commandList[0]=='arm':
        drone = drones[droneID]
        change_mode(drone, "STABILIZE")
        arm_only(drone)
        return "Armed"

    if commandList[0]=='takeoff':
        drone = drones[droneID]
        altitude = float(commandList[2])
        change_mode(drone, "GUIDED")
        arm_and_takeoff(drone, altitude)
        return "Take off"

    if commandList[0]=='goto':
        drone = drones[droneID]
        speed = float(commandList[2])
        x = float(commandList[3])
        y = float(commandList[4])
        z = float(commandList[5])
        go_to(drone, speed, x, y, z)
        return "goto"

    if commandList[0]=='land':
        drone = drones[droneID]
        land(drone)
        return "Landing"

    if commandList[0]=='battery':
        drone = drones[droneID]
        battery = get_battery(drone)
        return 'battery '+str(droneID)+' '+str(battery.voltage)+' '+str(battery.current)+' '+str(battery.level)

    if commandList[0]=='location_global':
        drone = drones[droneID]
        location = get_location(drone)
        return 'location_global '+str(droneID)+' '+str(location.lat)+' '+str(location.lon)+' '+str(location.alt)






def command_to_drone_serial(message, droneAddresses):
    commandList = message.split();
    droneID = int(commandList[1])

    if commandList[0]=='connect':
        address = commandList[2]
        droneAddresses.append(address)
        return "Connected"

    if commandList[0]=='arm':
        address = droneAddresses[droneID]
        drone = connect(address)
        change_mode(drone, "STABILIZE")
        arm_only(drone)
        drone.close()
        return "Armed"

    if commandList[0]=='takeoff':
        address = droneAddresses[droneID]
        drone = connect(address)
        altitude = float(commandList[2])
        arm_and_takeoff(drone, altitude)
        drone.close()
        return "Take off"

    if commandList[0]=='land':
        address = droneAddresses[droneID]
        drone = connect(address)
        land(drone)
        return "Landing"

    if commandList[0]=='goto':
        address = droneAddresses[droneID]
        drone = connect(address)
        speed = float(commandList[2])
        x = float(commandList[3])
        y = float(commandList[4])
        z = float(commandList[5])
        go_to(drone, speed, x, y, z)
	time.sleep(0.01)
        drone.close()
        return "goto"

    if commandList[0]=='battery':
        address = droneAddresses[droneID]
        drone = connect(address)
        battery = get_battery(drone)
        drone.close()
        return 'battery '+str(droneID)+' '+str(battery.voltage)+' '+str(battery.current)+' '+str(battery.level)

    if commandList[0]=='location_global':
        address = droneAddresses[droneID]
        drone = connect(address)
        location = get_location(drone)
        drone.close()
        return 'location_global '+str(droneID)+' '+str(location.lat)+' '+str(location.lon)+' '+str(location.alt)








#### MAIN ####
communicator = PyZMQCommunicator()
communicator.bind_server("tcp://166.104.145.106:5556")
communicator.connect_client("tcp://localhost:5555")

bSerial = raw_input("Do you want serial communication?(y/n):")

drones = list()
droneAddresses = list()

if bSerial == 'y': 
    t1 = Thread(target=run_receiver_serial, args=(communicator, droneAddresses,))
    t1.daemon = True
    t1.start()
else:
    t1 = Thread(target=run_receiver_parallel, args=(communicator, drones,))
    t1.daemon = True
    t1.start()

while True:
    a = raw_input("command?:")
    if(a == 'q'):
        break
