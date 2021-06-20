import dronekit
import time
from pymavlink import mavutil
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from listcoordinate import getCoordinates
from coordinate import generate

import math
vehicle = connect('/dev/ttyACM0',wait_ready=True,baud=921600)

def get_distance_metres(aLocation1, aLocation2):
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def arm_checks_arm():
    '''
    while not vehicle.is_armable:
        print("Waiting to Initializa")
        time.sleep(1)
    '''
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        print("Arming Process")
        time.sleep(1)

def disarm_check():
    vehicle.armed = False
    while vehicle.armed:
        print("Disarming")
        time.sleep(1)

def get_distance_metresmy(x,y, aLocation2):
    dlat = aLocation2.lat - x
    dlong = aLocation2.lon - y
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def get_location_metres(original_location, dNorth, dEast):
    earth_radius = 6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    if type(original_location) is LocationGlobal:
        targetlocation=LocationGlobal(newlat, newlon,original_location.alt)
    elif type(original_location) is LocationGlobalRelative:
        targetlocation=LocationGlobalRelative(newlat, newlon,original_location.alt)
    else:
        raise Exception("Invalid Location object passed")
        
    return targetlocation;

def gotoLocation(x,y):
    vehicle.groundspeed = 0.1
    point = LocationGlobalRelative(x,y,vehicle.location.global_frame.alt)
    vehicle.simple_goto(point,groundspeed=0.1)
    print("Heading to Location")
    while get_distance_metresmy(x,y,vehicle.location.global_frame) > 3:
        print("Remaining Distance %s",get_distance_metresmy(x,y,vehicle.location.global_frame))
        time.sleep(0.1)

def printChannelVal():
    print(" Channel overrides: %s" % vehicle.channels.overrides) 

def set_velocity_body(Vx, Vy, Vz):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000111111000111,
        0, 0, 0,
        Vx, Vy, Vz,
        0, 0, 0,
        0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()  


def left(xsec,xspeed):
    set_velocity_body(0,-xspeed,0)
    time.sleep(xsec)
    set_velocity_body(0,0,0)
    time.sleep(0.2)

def right(xsec,xspeed):
    set_velocity_body(0,xspeed,0)
    time.sleep(xsec)
    set_velocity_body(0,0,0)
    time.sleep(0.2)

def front(xsec,xspeed):
    set_velocity_body(xspeed,0,0)
    time.sleep(xsec)
    set_velocity_body(0,0,0)
    time.sleep(0.2)

arm_checks_arm()



#front(2,0.5)
#for i in range (0,2):
#    left(2.2,0.5)
#right(2,0.3)
#while True:
#    print(" Global Location: %s" % vehicle.location.global_frame)

#This function tries to move the rover to the desired location obtained from reading json file 

generate(13.015816643359752, 77.66981557166832,13.016715616055903, 77.6692254856633)
time.sleep(2)
for i in getCoordinates():
    print(i[0],i[1])
    gotoLocation(i[0],i[1])
        
time.sleep(3)
disarm_check()
vehicle.close()