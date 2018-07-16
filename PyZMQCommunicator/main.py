import zmq
import time
from threading import *
from PyZMQCommunicator import PyZMQCommunicator
from DroneCommander import *
from MissionDesigner import *

MAV_MODE_AUTO   = 4
MAV_MODE_GUIDED   = 8

def run_receiver(communicator, drones):
    while True:
        print("Waiting for receiving message...")
        message = communicator.receive()
        print("Received msg: %s" % message)
        replyMessage = command_to_drone(message, drones)
        communicator.reply_to_client("Well received: %s" % replyMessage)
        #time.sleep(0.1)


def command_to_drone(message, drones):
    commandList = message.split();
    
    if commandList[0]=='loadMission':
        fileName = commandList[1]
        load_mission(drones, fileName)
        return "mission loaded"

    droneID = int(commandList[1])

    if commandList[0]=='connect':
        systemID = int(commandList[2])
        address = commandList[3]
        if len(drones)>droneID:
            drones[droneID] = connect(address)
        else:
            vehicle = connect(address)
            drones.append(vehicle)
        return 'Connected '+str(droneID)

        '''try:
            if len(drones)>droneID:
                drones[droneID] = connect(address)
            else:
                vehicle = connect(address)
                drones.append(vehicle)
            return 'Connected '+str(droneID)
        except APIException:
            return 'Disconnected '+str(droneID)'''

    # the commands below requires connection to drone
    drone = drones[droneID]
    if check_connection(drone) == False:
         return 'ConnectionLost '+str(droneID)

    if commandList[0]=='arm':
        #change_mode(drone, "STABILIZE")
        arm_only(drone)
        return "Armed"

    if commandList[0]=='takeoff':
        altitude = float(commandList[2])
        PX4setMode(drone, MAV_MODE_GUIDED)
        result = arm_and_takeoff(drone, altitude)
        if result==True:
            return "Take off"
        else:
            return 'FailToTakeOff '+str(droneID)

    if commandList[0]=='goto':
        speed = float(commandList[2])
        x = float(commandList[3])
        y = float(commandList[4])
        z = float(commandList[5])
        go_to(drone, speed, x, y, z)
        return "goto"

    if commandList[0]=='land':
        land(drone)
        return "Landing"

    if commandList[0]=='battery':
        battery = get_battery(drone)
        return 'battery '+str(droneID)+' '+str(battery.voltage)+' '+str(battery.current)+' '+str(battery.level)

    if commandList[0]=='location_global':
        location = get_location(drone)
        return 'location_global '+str(droneID)+' '+str(location.lat)+' '+str(location.lon)+' '+str(location.alt)
        
    if commandList[0]=='mode':
        return 'mode '+str(droneID)+' '+str(drone.mode.name)

    if commandList[0]=='auto':
        #change_mode(drone, 'AUTO')
        PX4setMode(drone, MAV_MODE_AUTO)
        return 'changeMode '+str(droneID)+' '+str(drone.mode.name)

    if commandList[0]=='guided':
        PX4setMode(drone, MAV_MODE_GUIDED)
        return 'changeMode '+str(droneID)+' '+str(drone.mode.name)

    if commandList[0]=='speed':
        change_speed(drone, float(commandList[2]))
        return 'changeSpeed '+str(droneID)+' '+str(drone.groundspeed)

    if commandList[0]=='addstep':
        speed = float(commandList[2])
        latitude = float(commandList[3])
        longitude = float(commandList[4])
        altitude = float(commandList[5])
        loiterTime = float(commandList[6])
        add_change_speed(drone, speed)
        add_waypoint(drone, latitude, longitude, altitude)
        add_loiter(drone, loiterTime)
        return 'add step '+str(droneID)+' '+str(speed)+' '+str(latitude)+' '+str(longitude)+' '+str(altitude)+' '+str(loiterTime)
     
    if commandList[0]=='clearMission':
        clear_mission(drone)
        return 'clear mission '+str(droneID)

    if commandList[0]=='uploadMission':
        upload_mission(drone)
        return 'upload mission '+str(droneID)

    if commandList[0]=='doesArrive':
        comID = drone.commands.next
        if comID%3 == 0:
            return 'true'
        else:
            return 'false'

    if commandList[0]=='jumpStep':
        targetStepID = int(commandList[2])
        targetCommandID = 3*targetStepID+1
        drone.commands.next = targetCommandID
        return 'jumpStep '+str(droneID)+' '+str(targetStepID)

    if commandList[0]=='getStepID':
        comID = drone.commands.next
        sceneID = (comID-(comID-1)%3)/3
        return 'sceneID '+str(sceneID)

    if commandList[0]=='getCommandID':
        comID = drone.commands.next
        return 'commandID '+str(droneID)+' '+str(comID)

    if commandList[0]=='next':
        drone.commands.next = drone.commands.next+1
        comID = drone.commands.next
        return 'nextCommandID '+str(droneID)+' '+str(comID)

    

    if commandList[0]=='printMission':
        print_mission(drone, droneID)
        return 'print mission '+str(droneID)

    if commandList[0]=='adjustMission':
        adjust_mission(drone, droneID)
        return 'adjust mission '+str(droneID)

    if commandList[0]=='close':
        drone.close()
        return 'close mission '+str(droneID)

    if commandList[0]=='addwp':
        latitude = float(commandList[2])
        longitude = float(commandList[3])
        altitude = float(commandList[4])
        add_waypoint(drone, latitude, longitude, altitude)
        return 'add waypoint '+str(droneID)+' '+str(latitude)+' '+str(longitude)+' '+str(altitude)

    if commandList[0]=='addswp':
        latitude = float(commandList[2])
        longitude = float(commandList[3])
        altitude = float(commandList[4])
        add_spline_waypoint(drone, latitude, longitude, altitude)
        return 'add waypoint '+str(droneID)+' '+str(latitude)+' '+str(longitude)+' '+str(altitude)

    if commandList[0]=='addto':
        altitude = float(commandList[2])
        add_takeoff(drone, altitude)
        return 'add takeoff '+str(droneID)+' '+str(altitude)

    if commandList[0]=='addloiter':
        time = 0
        if(len(commandList) > 2):
            time = float(commandList[2])
        add_loiter(drone, time)
        return 'add loiter '+str(droneID)


def read_port(fileName):
    portFile = open(fileName, 'r')
    lines = portFile.readlines()
    ports = list()
    for line in lines:
        ports.append(int(line))
    return ports

#### MAIN ####
communicator = PyZMQCommunicator()
communicator.bind_server('tcp://*:5556')

portNums = read_port('port.txt') 

drones = list()
for portNum in portNums:
    drones.append(connect('127.0.0.1:'+str(portNum)))

t1 = Thread(target=run_receiver, args=(communicator, drones,))
t1.daemon = True
t1.start()   

while True:
    a = raw_input("command?:")
    if(a == 'q'):
        break
