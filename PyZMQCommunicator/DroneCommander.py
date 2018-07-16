from dronekit import *
from math import sin, radians, cos

METER_PER_LATITUDE = 11E4
METER_PER_LONGITUDE = 9E4

def PX4setMode(drone, mavMode):
    drone._master.mav.command_long_send(drone._master.target_system, drone._master.target_component,
                                               mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                                               mavMode,
                                               0, 0, 0, 0, 0, 0)

def arm_and_takeoff(drone, aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not drone.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    drone.mode = VehicleMode("GUIDED")
    drone.armed = True

    # Confirm vehicle armed before attempting to take off
    for i in range(1, 5):
        if(drone.armed == False):
            print(" Waiting for arming...")
            time.sleep(1)
        else:
            print("Arm success")
            break

    if(drone.armed == False):
        return False

    print("Taking off!")
    drone.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    return True


def change_mode(drone, mode):
    drone.mode = VehicleMode(mode)

def change_speed(drone, speed):
    drone.groundspeed = speed


def arm_only(drone):
    drone.armed = True



def take_off(drone, targetAlt):
   drone.simple_takeoff(targetAlt)



def go_to(drone, groundSpeed, latitude, longitude, altitude):
    gps = LocationGlobalRelative(latitude, longitude, altitude)
    drone.simple_goto(gps, groundSpeed)



def land(drone):
        print 'landing..'
        drone.mode  = VehicleMode("LAND")
        time.sleep(0.05)
        #drone.close()

def get_battery(drone):
    return drone.battery

def get_location(drone):
    gps = drone.location.global_relative_frame
    coord = [gps.lat, gps.lon, gps.alt]
    return coord

def check_connection(drone):
    lastHB = drone.last_heartbeat
    if lastHB < 3:
        return True;
    else:
        return False;

def gps_to_local(latitude, longitude, homeLatitude, homeLongitude):
    x = (longitude - homeLongitude)*METER_PER_LONGITUDE
    y = (latitude - homeLatitude)*METER_PER_LATITUDE
    return [ x, y ]

def local_to_gps(x, y, homeLatitude, homeLongitude):
    latitude = homeLatitude + (y / METER_PER_LATITUDE)
    longitude = homeLongitude + (x / METER_PER_LONGITUDE)
    return [ latitude, longitude ]

def meter_per_latitude(homeLatitude):
    return 111132.92 - 559.82*cos(2 * homeLatitude) + 1.175*cos(4 * homeLatitude) - 0.0023*cos(6 * homeLatitude)

def meter_per_longitude(homeLatitude):
    return 111412.84*cos(homeLatitude) - 93.5*cos(3 * homeLatitude) + 0.118*cos(5 * homeLatitude)


def rotate_coord_by_angle(x, y, angle):
	x_rotate = x * cos(angle) + y * sin(angle);
	y_rotate = y * cos(angle) - x * sin(angle);
	return [x_rotate, y_rotate]