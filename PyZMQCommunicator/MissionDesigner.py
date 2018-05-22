from dronekit import Command, CommandSequence
from DroneCommander import *
from pymavlink import *

def load_mission(drones, homePosition, fileName):
    missionFile = open(fileName, 'r')
    lines = missionFile.readlines()
    
    for drone in drones:
        clear_mission(drone)
    
    count=1
    for line in lines:
        missionCommand = line.split();
        droneID = int(missionCommand[1])
        drone = drones[droneID]
        if(missionCommand[0] == 't'):
            altitude = int(missionCommand[2])
            add_takeoff(drone, altitude)
        if(missionCommand[0] == 'w'):
            #latitude = missionCommand[2]
            #longitude = missionCommand[3]
            altitude = float(missionCommand[4])
            gps = local_to_gps(float(missionCommand[2]), float(missionCommand[3]), homePosition[0], homePosition[1])
            latitude = gps[0]
            longitude = gps[1]
            add_change_speed(drone, count)
            add_waypoint(drone, latitude, longitude, altitude, distance=0.5)
            add_loiter(drone, 10)
            #add_guided_enable(drone)
            count = count+1
        if(missionCommand[0] == 'l'):
            add_land(drone)
    missionFile.close()
    for drone in drones:
        upload(drone)


def upload_mission(drone):
    drone.commands.upload()
    drone.commands.wait_ready()
    print('upload complete')

def add_takeoff(drone, altitude):
    command = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
                        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, altitude)
    print('takeoff added')
    drone.commands.add(command)

def add_land(drone):
    command = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
                        mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0, 0)
    print('land added')
    drone.commands.add(command)

def add_waypoint(drone, latitude, longitude, altitude, delay=0.0, distance=1.0):
    gps = LocationGlobalRelative(latitude, longitude, altitude)
    command = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
                        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, delay, distance, 0, 0, gps.lat, gps.lon, gps.alt)
    print('waypoint added')
    drone.commands.add(command)

def add_spline_waypoint(drone, latitude, longitude, altitude, delay=0):
    gps = LocationGlobalRelative(latitude, longitude, altitude)
    command = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
                        mavutil.mavlink.MAV_CMD_NAV_SPLINE_WAYPOINT, 0, 0, delay, 0, 0, 0, gps.lat, gps.lon, gps.alt)
    drone.commands.add(command)

def add_change_speed(drone, speed):
    command = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
                        mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 0, 0, 1, speed, -1, 0, 0, 0, 0)
    drone.commands.add(command)

def add_loiter(drone, time = 0):

    if time==0 :
        command = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
                        mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM, 0, 0, 0, 0, 0, 0, 0, 0, 0)
    else:
        command = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
                        mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME, 0, 0, time, 0, 0, 0, 0, 0, 0)
    drone.commands.add(command)

def add_guided_enable(drone):
    command = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
                    mavutil.mavlink.MAV_CMD_NAV_GUIDED_ENABLE, 0, 0, 1, 0, 0, 0, 0, 0, 0)
    drone.commands.add(command)

def add_condition_distance(drone, distance):
    command = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
                    mavutil.mavlink.MAV_CMD_CONDITION_DISTANCE, 0, 0, distance, 0, 0, 0, 0, 0, 0)
    drone.commands.add(command)

def clear_mission(drone):
    drone.commands.clear()