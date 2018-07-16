from dronekit import Command, CommandSequence
from DroneCommander import *
from pymavlink import *

STADIUM_TILT = 0.21343877

def load_mission(drones, fileName):
    missionFile = open(fileName, 'r')
    lines = missionFile.readlines()

    #first line: home info
    headerInfo = lines[0].split()
    lines.pop(0)
    numDrones = int(headerInfo[0])
    centerPosition = [float(headerInfo[1]), float(headerInfo[2]) ]
    takeoffAltitude = 5

    for drone in drones:
        clear_mission(drone)
    
    lastPosition = list()
    for drone in drones:
        home = drone.location.global_relative_frame
        add_takeoff(drone, home.lat, home.lon, takeoffAltitude)
        lastPosition.append([home.lat, home.lon])

    for line in lines:
        missionCommand = line.split();
        droneID = int(missionCommand[0])
        drone = drones[droneID]

        x = float(missionCommand[1])
        y = float(missionCommand[2])
        altitude = float(missionCommand[3])
        speed = float(missionCommand[4])
        loiterTime = float(missionCommand[5])
        rotatedCoord = rotate_coord_by_angle(x, y, STADIUM_TILT);
        gps = local_to_gps(rotatedCoord[0], rotatedCoord[1], centerPosition[0], centerPosition[1])
        #add_change_speed(drone, latitude, longitude, altitude, speed)
        add_waypoint(drone, gps[0], gps[1], altitude, distance=0.5)
        add_loiter(drone, gps[0], gps[1], altitude, loiterTime)
        lastPosition[droneID] = gps
    
    missionFile.close()

    for i in range(0, len(drones)):
        add_land(drones[i], lastPosition[i][0], lastPosition[i][1], takeoffAltitude)

    for drone in drones:
        upload_mission(drone)


def upload_mission(drone):
    drone.commands.upload()
    drone.commands.wait_ready()
    print('upload complete')


def add_takeoff(drone, latitude, longitude, altitude):
    command = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
                        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 1, 0, 0, 0, 0, latitude, longitude, altitude)
    print('takeoff added')
    drone.commands.add(command)


def add_land(drone, latitude, longitude, altitude):
    command = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
                        mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 1, 0, 0, 0, 0, latitude, longitude, altitude)
    print('land added')
    drone.commands.add(command)


def add_waypoint(drone, latitude, longitude, altitude, delay=0.0, distance=1.0):
    gps = LocationGlobalRelative(latitude, longitude, altitude)
    command = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
                        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, delay, distance, 0, 0, gps.lat, gps.lon, gps.alt)
    drone.commands.add(command)
   

def add_change_speed(drone, latitude, longitude, altitude, speed):
    command = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_MISSION, 
                        mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 0, 1, 1, speed, 0, 0, 0, 0, 0)
    drone.commands.add(command)

def add_loiter(drone, latitude, longitude, altitude, time = 0):

    if time==0 :
        command = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
                        mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM, 0, 1, 0, 0, 0, 0, latitude, longitude, altitude)
    else:
        command = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
                        mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME, 0, 1, time, 0, 0, 0, latitude, longitude, altitude)
    drone.commands.add(command)


def clear_mission(drone):
    drone.commands.clear()
    drone.commands.upload()

def print_mission(drone, droneID):
    print "current mission:"
    cmds = drone.commands
    cmds.download()
    cmds.wait_ready()
    for cmd in cmds:
        print cmd