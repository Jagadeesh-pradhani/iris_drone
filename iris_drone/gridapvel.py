import rospy
from std_msgs.msg import Float32MultiArray, String
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
vehicle = connect(args.connect, baud=57600, wait_ready=True, timeout=60)


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









def callback2(data):
    global clas
    clas= data.data
    #print(f"Received centroid values: x={clas}")
    return

def callback(data):
    global x
    global y
    x, y= data.data
    #print(f"Received centroid values: x={x}, y={y}")
    return

def call():
    rospy.init_node('centroid_subscriber')
    rospy.Subscriber('/detections/centroid', Float32MultiArray, callback)
    #rospy.Subscriber('/detections/class1', String, callback2)
    time.sleep(0.1)
    return



def intake():
    call()
    #time.sleep(5)
    #print("capturing data")
    call()
    global clas
    global detections
    global x
    global y
    xy = []
    detections = eval(clas, temp_dict)
    #print("number of objects detected {}".format(len(detections)))

    for detection in detections:
        x, y, w, h = detection['x'], detection['y'], detection['width'], detection['height']
        centroid_x = x + (w / 2) # calculate centroid x-coordinate
        centroid_y = y + (h / 2) # calculate centroid y-coordinate
        detection['x'] = centroid_x
        detection['y'] = centroid_y
        if detection['class'] == "frisbee":
            xy.append([centroid_x,centroid_y])
        else:
            xy.append([centroid_x,centroid_y])
        
              
        #print("x = ",centroid_x, "y = ",centroid_y, "class = ",detection['class'])
        #class2 = f"hotspot{num}"
    return xy


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
    vehicle.simple_goto(target_Location, groundspeed=1.5)

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

def calc(cam, alt):
    if cam  == True:
        x_ang = 53.5
        y_ang = 41.41
        h_x = 640
        h_y = 360
    else:
        x_ang = 62.2#148.1678413   # walksnail cam #62.2<- raspi    
        y_ang = 37.46#83.34441074#37.46
        h_x = 320
        h_y = 240

    a1 = 2 * alt * math.tan(math.radians(x_ang / 2))   #36.1943 x-axis full distance
    b1 = 2 * alt * math.tan(math.radians(y_ang / 2))  #20.3439 y-axis full distance : 46.65 for analaog cam : 37.46 angle in ros
    m = a1/(2*h_x)
    n = b1/(2*h_y)
    return m,n

def direct(x, y):
    x_diff = x - 640
    y_diff = y - 360
    m,n = calc(cam=1, alt=vehicle.location.global_relative_frame.alt)
    dx = m*x_diff
    dy = n*y_diff

    if dx > 0:
        send_ned_velocity(groundspeed, 0, 0)
        time.sleep(dx/groundspeed)
    elif dx < 0:
        send_ned_velocity(-groundspeed, 0, 0)
        time.sleep(dx/groundspeed)
    send_ned_velocity(0, 0, 0)
    
    if dy > 0:
        send_ned_velocity(0, -groundspeed, 0)
        time.sleep(dx/groundspeed)
    elif dy < 0:
        send_ned_velocity(0, groundspeed, 0)
        time.sleep(dx/groundspeed)
    
def landt():
    print("Landing")
    h_x = 640
    h_y = 360
    x_pre = 0
    y_pre = 0
    while(1):
        x = 0
        y = 0
        xy = intake()
        time.sleep(0.1)
        for i in xy:
            x = i[0]
            y = i[1]
        print(x,y)
        if x == 0 and y == 0:
            if x_pre and y_pre:
                print("going direct")
                direct(x_pre, y_pre)
            break
        else:
            x_pre = x
            y_pre = y
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
        else:
            send_ned_velocity(0, 0, 0)
            break
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)

def landavi():
    global x
    global y
    call()
    print("Landing")
    h_x = 320
    h_y = 240
    x_pre = 0
    y_pre = 0
    while(1):
        call()
        time.sleep(0.1)
        call()
        print(x,y)
        if x == 0 and y == 0:
            if x_pre and y_pre:
                print("going direct")
                direct(x_pre, y_pre)
            break
        else:
            x_pre = x
            y_pre = y
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
        else:
            send_ned_velocity(0, 0, 0)
            break
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)




"""___________________________________________________Main mission call_____________________________________________________"""


def mission():

    alt = 15
    print(vehicle.location.global_frame.lat, vehicle.location.global_frame.lon)
    print(vehicle.heading)

    arm_and_takeoff(alt)            #takeoff to 30m

    time.sleep(5)
    landavi()
 
    print("Mission complete")
    vehicle.mode = VehicleMode("LAND")

mission()

vehicle.close()
