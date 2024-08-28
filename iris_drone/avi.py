import time
import math
from collections import OrderedDict
from pymavlink import mavutil
import numpy as np
import cv2
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='127.0.0.1:14550')
args = parser.parse_args()

# Connect to the Vehicle
print ('Connecting to vehicle on: %s' % args.connect)
vehicle = connect(args.connect, baud=921600, wait_ready=True, timeout=60)


temp_dict = {'OrderedDict': OrderedDict}
global x
global y
global clas
global hotcount
global detections
global in_x
global in_y
x = 0
y = 0
clas = 'v'
CAMERA_RESOLUTION = (1280, 720)
groundspeed = 0.25
cam_y = 17.129568167795526      #in m
cam_x = 40.01904440518039
short = []



def write_single_number_to_file(number):
    with open("xa.txt", 'w') as file:
        file.write(str(number))

def write_single_number_to_file(number):
    with open("yb.txt", 'w') as file:
        file.write(str(number))
def geta():    

 with open("xa.txt", "r") as file:

    last_line2 = file.readlines()[-1]

    a=int(last_line2)

    return a

def getb():    

 with open("yb.txt", "r") as file:

    last_line3 = file.readlines()[-1]

    b=int(last_line3)   

    return b  


"""
Arm and takeoff function to initialize the drone to 30 m height
"""

def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:  # Trigger just below target alt.
            print("Reached target altitude")
            break
        time.sleep(1)


def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the
    earth's poles. It comes from the ArduPilot test code:
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5

def goto_location_norm(to_lat, to_lon, alt):
    currentLocation = vehicle.location.global_frame
    target_Location = LocationGlobalRelative(to_lat, to_lon, alt)
    targetDistance = get_distance_metres(currentLocation, target_Location)
    Distance = get_distance_metres(vehicle.location.global_frame, target_Location)
    vehicle.simple_goto(target_Location, groundspeed=2.8)

    while vehicle.mode.name == "GUIDED":
        # print "DEBUG: mode: %s" % vehicle.mode.name
        remainingDistance = get_distance_metres(vehicle.location.global_frame, target_Location)
        print("Distance to wavepoint: ", remainingDistance)
        if remainingDistance <= 1:  # Just below target, in case of undershoot.
            print("Reached target")
            break
        time.sleep(2)


def send_ned_velocity(velocity_x, velocity_y, velocity_z):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms (not used)
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # frame
        0b10111000111,  # type_mask (only speeds enabled)
        0, 0, 0,  # x, y, z positions (not used)
        velocity_y, velocity_x, velocity_z,  # x, y, z velocity in m/s
        0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    vehicle.send_mavlink(msg)
    vehicle.flush()

def send_ned_velocity1(velocity_z, to_alt):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms (not used)
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
        0b0000111111000111,  # type_mask (only speeds enabled)
        0, 0, 0,  # x, y, z positions (not used)
        0, 0, velocity_z,  # x, y, z velocity in m/s
        0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    if to_alt == 0:
        vehicle.send_mavlink(msg)
        time.sleep(1)
        return

    while True:
        vehicle.send_mavlink(msg)
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= to_alt - 1 and vehicle.location.global_relative_frame.alt <= to_alt + 1:
            print("Reached target altitude")
            break
        time.sleep(1)

def drop_payload(PWM):
    msg = vehicle.message_factory.command_long_encode(
        0, 0,  # target_system, target_component
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,  # command
        0,  # confirmation
        9,  # servo number
        PWM,  # servo position between 1000 and 2000
        0, 0, 0, 0, 0)  # param 3 ~ 7 not used
    print("dropping payload...")
    # send command to vehicle
    vehicle.send_mavlink(msg)
    print("payload dropped...")

'''def landavi():
    global x
    global y
    print("Landing")
    h_x = 400
    h_y = 240
    x = geta()
    y=getb()

    while(1):
        x = geta()
        y=getb()
        time.sleep(0.1)
        x = geta()
        y=getb()
        print(x,y)
        if vehicle.location.global_frame.alt>=3.5:
            if x == h_x and y == h_y:
                send_ned_velocity(0, 0, 1)
                time.sleep(1)
			
            if x != h_x and y != h_y:
                if x > h_x and y > h_y:
                    send_ned_velocity(groundspeed, -groundspeed, groundspeed)
                    time.sleep(0.1)
                elif x < h_x and y < h_y:
                    send_ned_velocity(-groundspeed, groundspeed, groundspeed)
                    time.sleep(0.1)
                elif x < h_x and y > h_y:
                    send_ned_velocity(-groundspeed, -groundspeed, groundspeed)
                    time.sleep(0.1)
                elif x > h_x and y < h_y:
                    send_ned_velocity(groundspeed, groundspeed, groundspeed)
                    time.sleep(0.1)
            elif x == h_x and y != h_y:
                if y > h_y:
                    send_ned_velocity(0, -groundspeed, groundspeed)
                    time.sleep(0.1)
                elif y < h_y:
                    send_ned_velocity(0, groundspeed, groundspeed)
                    time.sleep(0.1)
            elif y == h_y and x != h_x:
                if x > h_x:
                    send_ned_velocity(groundspeed, 0, groundspeed)
                    time.sleep(0.1)
                elif x < h_x:
                    send_ned_velocity(-groundspeed, 0, groundspeed)
                    time.sleep(0.1)
        if vehicle.location.global_relative_frame.alt<=3.49 :
            break
        else:
            if x != h_x and y != h_y:
                if x > h_x and y > h_y:
                    send_ned_velocity(groundspeed, -groundspeed, 0)
                    time.sleep(0.1)
                elif x < h_x and y < h_y:
                    send_ned_velocity(-groundspeed, groundspeed,  0)
                    time.sleep(0.1)
                elif x < h_x and y > h_y:
                    send_ned_velocity(-groundspeed, -groundspeed, 0)
                    time.sleep(0.1)
                elif x > h_x and y < h_y:
                    send_ned_velocity(groundspeed, groundspeed, 0)
                    time.sleep(0.1)
            elif x == h_x and y != h_y:
                if y > h_y:
                    send_ned_velocity(0, -groundspeed, 0)
                    time.sleep(0.1)
                elif y < h_y:
                    send_ned_velocity(0, groundspeed, 0)
                    time.sleep(0.1)
            elif y == h_y and x != h_x:
                if x > h_x:
                    send_ned_velocity(groundspeed, 0, 0)
                    time.sleep(0.1)
                elif x < h_x:
                    send_ned_velocity(-groundspeed, 0, 0)
                    time.sleep(0.1)
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
'''
def landavi():
    h_x = 400
    h_y = 240

    def send_velocity_based_on_position(x, y, z_velocity,g_speed):
        if x == h_x and y == h_y:
            send_ned_velocity(0, 0, z_velocity)
        elif x > h_x and y > h_y:
            send_ned_velocity(g_speed, -g_speed, g_speed)
        elif x < h_x and y < h_y:
            send_ned_velocity(-g_speed, g_speed, g_speed)
        elif x < h_x and y > h_y:
            send_ned_velocity(-g_speed, -g_speed, g_speed)
        elif x > h_x and y < h_y:
            send_ned_velocity(g_speed, g_speed, g_speed)
        elif x == h_x and y != h_y:
            if y > h_y:
                send_ned_velocity(0, -g_speed, g_speed)
            elif y < h_y:
                send_ned_velocity(0, g_speed, g_speed)
        elif y == h_y and x != h_x:
            if x > h_x:
                send_ned_velocity(g_speed, 0, g_speed)
            elif x < h_x:
                send_ned_velocity(-g_speed, 0, g_speed)

    prev_x, prev_y = None, None

    while True:
        x = geta()
        y = getb()
        time.sleep(0.1)
        print(x, y)

        if prev_x is not None and prev_y is not None:
            if x == prev_x and y == prev_y:
                send_ned_velocity(0, 0,0.25)
            elif vehicle.location.global_frame.alt >= 2.5:
                send_velocity_based_on_position(x, y, 1,0.25)

        if vehicle.location.global_relative_frame.alt <= 2.59:
            break

        print("Altitude: ", vehicle.location.global_relative_frame.alt)
        prev_x, prev_y = x, y

    for i in range(1):
        x = geta()
        y = getb()
        if x == h_x and y == h_y:
            send_ned_velocity(0, 0, 0.25)
        else:
            send_velocity_based_on_position(x, y, 0, 0.2)
        print("Altitude: ", vehicle.location.global_relative_frame.alt)



"""Main mission call"""


def mission():

    alt = 8
    print(vehicle.location.global_frame.lat, vehicle.location.global_frame.lon)
    print(vehicle.heading)
    lat = vehicle.location.global_frame.lat
    lon = vehicle.location.global_frame.lon

    arm_and_takeoff(alt)            #takeoff to 5m

    goto_location_norm(12.9177947, 80.2402469, alt)
    write_single_number_to_file(400)
    write_single_number_to_file(240)

    time.sleep(5)
    #send_ned_velocity1(0.25,1, 0)
    #time.sleep(5)
    landavi()
    time.sleep(5)

    print("Payload drop")    
    drop_payload(950)
    time.sleep(5)

    #arm_and_takeoff(5)            #takeoff to 5m
    time.sleep(2)

    goto_location_norm(lat, lon, 5)
    time.sleep(5)
    print("complete")
    vehicle.mode = VehicleMode("LAND")


mission()

vehicle.close()


