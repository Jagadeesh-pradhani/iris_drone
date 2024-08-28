import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Twist, TransformStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from gazebo_msgs.msg import ModelStates
from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSProfile
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
import argparse
from pymavlink import mavutil
import time
import math
import threading
from rclpy.executors import MultiThreadedExecutor


class AutonomousMission(Node):
    def __init__(self):
        super().__init__('Autonomous_Mission')
        
        self.x = None
        self.y = None
        self.subscription = self.create_subscription(
            Point,
            '/xy_coordiantes',
            self.coordinates_callback,
            10)
        self.subscription  # prevent unused variable warning
        #self.get_logger().info(f'Received tuple: ({self.x}, {self.y})')

        

        self.h_x = 640
        self.h_y = 360
        self.groundspeed = 0.25
        self.mission_complete = False

    
    def ConnectToVehicle(self):
        parser = argparse.ArgumentParser()
        parser.add_argument('--connect', default='127.0.0.1:14550')
        args = parser.parse_args()

        # Connect to the Vehicle
        print ('Connecting to vehicle on: %s' % args.connect)
        vehicle = connect(args.connect, baud=57600, wait_ready=True, timeout=60)
        return vehicle
    
    def coordinates_callback(self, msg):
        self.x = msg.x
        self.y = msg.y
        #self.get_logger().info(f'Received tuple: ({self.x}, {self.y})')
    
    def ArmAndTakeoff(self, aTargetAltitude):
        """
        Arms vehicle and fly to aTargetAltitude.
        """

        print("Basic pre-arm checks")
        # Don't let the user try to arm until autopilot is ready
        while not self.vehicle.is_armable:
            print(" Waiting for vehicle to initialise...")
            time.sleep(1)

        print("Arming motors")
        # Copter should arm in GUIDED mode
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True

        while not self.vehicle.armed:
            print(" Waiting for arming...")
            time.sleep(1)

        print("Taking off!")
        self.vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

        # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
        #  after Vehicle.simple_takeoff will execute immediately).
        while True:
            print(" Altitude: ", self.vehicle.location.global_relative_frame.alt)
            if self.vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:  # Trigger just below target alt.
                print("Reached target altitude")
                break
            time.sleep(1)

    def get_distance_metres(self, aLocation1, aLocation2):
        """
        Returns the ground distance in metres between two LocationGlobal objects.
        """
        dlat = aLocation2.lat - aLocation1.lat
        dlong = aLocation2.lon - aLocation1.lon
        return math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5

    def GotoLocation(self, to_lat, to_lon, alt):
        target_Location = LocationGlobalRelative(to_lat, to_lon, alt)
        self.vehicle.simple_goto(target_Location, groundspeed=1.5)

        while self.vehicle.mode.name == "GUIDED":
            # print "DEBUG: mode: %s" % vehicle.mode.name
            remainingDistance = self.get_distance_metres(self.vehicle.location.global_frame, target_Location)
            print("Distance to wavepoint: ", remainingDistance)
            if remainingDistance <= 1:  # Just below target, in case of undershoot.
                print("Reached target")
                break
            time.sleep(2)

    def TriggerServo(self, num, PWM):
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,  # target_system, target_component
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,  # command
            0,  # confirmation
            num,  # servo number
            PWM,  # servo position between 1000 and 2000
            0, 0, 0, 0, 0)  # param 3 ~ 7 not used
        print("dropping payload...")
        # send command to vehicle
        self.vehicle.send_mavlink(msg)
        print("payload dropped...")

    def send_ned_velocity(self, velocity_x, velocity_y, velocity_z):
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,  # time_boot_ms (not used)
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # frame
            0b10111000111,  # type_mask (only speeds enabled)
            0, 0, 0,  # x, y, z positions (not used)
            velocity_y, velocity_x, velocity_z,  # x, y, z velocity in m/s
            0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()

    def send_ned_velocity1(self, velocity_z, to_alt):
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,  # time_boot_ms (not used)
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
            0b0000111111000111,  # type_mask (only speeds enabled)
            0, 0, 0,  # x, y, z positions (not used)
            0, 0, velocity_z,  # x, y, z velocity in m/s
            0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

        if to_alt == 0:
            self.vehicle.send_mavlink(msg)
            time.sleep(1)
            return

        while True:
            self.vehicle.send_mavlink(msg)
            print(" Altitude: ", self.vehicle.location.global_relative_frame.alt)
            if self.vehicle.location.global_relative_frame.alt >= to_alt - 1 and self.vehicle.location.global_relative_frame.alt <= to_alt + 1:
                print("Reached target altitude")
                break
            time.sleep(1)

    def send_velocity_based_on_position(self, x, y, z_velocity,g_speed):
        h_x = self.h_x
        h_y = self.h_y
        if x == h_x and y == h_y:
            self.send_ned_velocity(0, 0, z_velocity)
        elif x > h_x and y > h_y:
            self.send_ned_velocity(g_speed, -g_speed, g_speed)
        elif x < h_x and y < h_y:
            self.send_ned_velocity(-g_speed, g_speed, g_speed)
        elif x < h_x and y > h_y:
            self.send_ned_velocity(-g_speed, -g_speed, g_speed)
        elif x > h_x and y < h_y:
            self.send_ned_velocity(g_speed, g_speed, g_speed)
        elif x == h_x and y != h_y:
            if y > h_y:
                self.send_ned_velocity(0, -g_speed, g_speed)
            elif y < h_y:
                self.send_ned_velocity(0, g_speed, g_speed)
        elif y == h_y and x != h_x:
            if x > h_x:
                self.send_ned_velocity(g_speed, 0, g_speed)
            elif x < h_x:
                self.send_ned_velocity(-g_speed, 0, g_speed)

    def landt(self):
        groundspeed = 0.25
        print("Landing")
        h_x = 640
        h_y = 360
        x_pre = 0
        y_pre = 0
        while(1):
            x = 0
            y = 0
            xy = [self.x, self.y]
            x = self.x
            y = self.y
            time.sleep(0.1)
            
            print(x,y)
            if x == 0 or x == None and y == 0 or y == None:
                if x_pre and y_pre:
                    print("going direct")
                    x = x_pre
                    y = y_pre
                    # continue
                else:
                    break
            else:
                x_pre = x
                y_pre = y
            if x != h_x and y != h_y:
                if x > h_x and y > h_y:
                    self.send_ned_velocity(groundspeed, -groundspeed, groundspeed)
                    time.sleep(0.1)
                elif x < h_x and y < h_y:
                    self.send_ned_velocity(-groundspeed, groundspeed, groundspeed)
                    time.sleep(0.1)
                elif x < h_x and y > h_y:
                    self.send_ned_velocity(-groundspeed, -groundspeed, groundspeed)
                    time.sleep(0.1)
                elif x > h_x and y < h_y:
                    self.send_ned_velocity(groundspeed, groundspeed, groundspeed)
                    time.sleep(0.1)
            elif x == h_x and y != h_y:
                if y > h_y:
                    self.send_ned_velocity(0, -groundspeed, groundspeed)
                    time.sleep(0.1)
                elif y < h_y:
                    self.send_ned_velocity(0, groundspeed, groundspeed)
                    time.sleep(0.1)
            elif y == h_y and x != h_x:
                if x > h_x:
                    self.send_ned_velocity(groundspeed, 0, groundspeed)
                    time.sleep(0.1)
                elif x < h_x:
                    self.send_ned_velocity(-groundspeed, 0, groundspeed)
                    time.sleep(0.1)
            else:
                self.send_ned_velocity(0, 0, 0)
                break
            print(" Altitude: ", self.vehicle.location.global_relative_frame.alt)
            if self.vehicle.location.global_relative_frame.alt <=2.5:
                print(f"Reached target : {self.vehicle.location.global_frame.lat}, {self.vehicle.location.global_frame.lon}")
                return


    def mission(self):
        self.vehicle = self.ConnectToVehicle()
        alt = 10
        print(self.vehicle.location.global_frame.lat, self.vehicle.location.global_frame.lon)
        print(self.vehicle.heading)
        self.ArmAndTakeoff(alt)
        time.sleep(4)
        self.GotoLocation(-35.3630969, 149.1651725, alt)
        time.sleep(4)
        self.landt()
        print("Mission complete")
        self.vehicle.mode = VehicleMode("RTL")
        self.vehicle.close()
        self.mission_complete = True





def main(args=None):
    rclpy.init(args=args)
    node = AutonomousMission()


    # Start the mission in a separate thread
    mission_thread = threading.Thread(target=node.mission)
    mission_thread.start()

    # Spin the node to keep receiving the messages
    while rclpy.ok() and not node.mission_complete:
        rclpy.spin_once(node, timeout_sec=1)

    # Mission completed, clean up
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()