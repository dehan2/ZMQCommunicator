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
    while not drone.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    #print("Taking off!")
    #drone.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto
    #  (otherwise the command after Vehicle.simple_takeoff will execute
    #   immediately).
    #while True:
    #    print(" Altitude: ", drone.location.global_relative_frame.alt)
    #    # Break and return from function just below target altitude.
    #    if drone.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
    #        print("Reached target altitude")
    #        break
    #    time.sleep(1)


def change_mode(drone, mode):
    drone.mode = VehicleMode(mode)


def arm_only(drone):
    drone.armed = True



def take_off(drone, targetAlt):
   drone.simple_takeoff(aTargetAltitude)



def go_to(drone, airspeed, targetLatitude, targetLongitude, targetAltitude):
    drone.airspeed=airspeed
    point = LocationGlobalRelative(targetLatitude, targetLongitude, targetAltitude)
    drone.simple_goto(point)



def get_target_gps(x, y, z, homeLat, homeLon, homeAlt):
    RADIUS_EARTH_EQUATOR = 6378245
    RADIUS_EARTH_POLAR = 6356863
    
    Radius_Earth = RADIUS_EARTH_POLAR + (RADIUS_EARTH_EQUATOR - RADIUS_EARTH_POLAR) * sin(radians(homeLat))
    Base_Dist_Longuitude = 2.0 * math.pi * Radius_Earth / 360.0 * cos(radians(homeLat))
    Base_Dist_Latitude = 2.0 * math.pi * Radius_Earth / 360.0

    gpsCoord = LocationGlobalRelative(homeLat + (y / Base_Dist_Latitude), homeLon + (x / Base_Dist_Longuitude), homeAlt + z )
    return gpsCoord



def land(drone):
        print 'landing..'
        drone.mode  = VehicleMode("LAND")
        time.sleep(0.05)
        drone.close()

def get_battery(drone):
    return drone.battery

def get_location(drone):
    return drone.location.global_relative_frame