from dronekit import *
from math import sin, radians, cos

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
    x = (longitude - homeLongitude)*meter_per_longitude(homeLatitude)
    y = (latitude - homeLatitude)*meter_per_latitude(homeLatitude)
    return [ x, y ]

def local_to_gps(x, y, homeLatitude, homeLongitude):
    latitude = homeLatitude + (y / meter_per_latitude(homeLatitude))
    longitude = homeLongitude + (x / meter_per_longitude(homeLatitude))
    return [ latitude, longitude ]

def meter_per_latitude(homeLatitude):
    return 111132.92 - 559.82*cos(2 * homeLatitude) + 1.175*cos(4 * homeLatitude) - 0.0023*cos(6 * homeLatitude)

def meter_per_longitude(homeLatitude):
    return 111412.84*cos(homeLatitude) - 93.5*cos(3 * homeLatitude) + 0.118*cos(5 * homeLatitude)