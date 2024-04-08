#!/usr/bin/env python

"""
Author: Haroon Rasheed
Date: 8 April 2024
"""

import rospy
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from flight_task_control import FlightTaskControl
from param import Param
from param_list import Parameters
from flight_mode_list import FlightModeList
from flight_mode import FlightMode
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64


class HotpointMission:
    def __init__(self):
        self.waypoint_list = WaypointList()
        self.waypoints = None
        self.latitude = None
        self.longitude = None
        self.altitude = None
        self.flight_mode = FlightMode()
        rospy.Subscriber('/mavros/global_position/global', NavSatFix,
                         self.callback_gps)  # get the current latlng (LatLng)

        rospy.Subscriber('/mavros/global_position/rel_alt', Float64,
                         self.callback_alt)  # get the current altitude in meter


    def callback_gps(self, msg):
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        


    def callback_alt(self, msg):
        self.altitude = msg.data
        

    def handle_waypoint(self, waypoints):
        self.waypoints = waypoints

        # change speed
        waypoint = Waypoint()
        waypoint.frame = 2
        waypoint.command = 178
        waypoint.is_current = True
        waypoint.autocontinue = False
        waypoint.x_lat = 0
        waypoint.y_long = 0
        waypoint.z_alt = 0
        waypoint.param1 = 1
        waypoint.param2 = 2
        waypoint.param3 = -1
        waypoint.param4 = 0
        self.waypoint_list.waypoints.append(waypoint)

        # takeoff
        waypoint = Waypoint()
        waypoint.frame = 3
        waypoint.command = 22
        waypoint.is_current = False
        waypoint.autocontinue = False
        waypoint.x_lat = 0  # latitude
        waypoint.y_long = 0  # longitude
        waypoint.z_alt = 5  # altitude
        waypoint.param1 = 0
        waypoint.param2 = 1
        waypoint.param3 = 0
        waypoint.param4 = 0
        self.waypoint_list.waypoints.append(waypoint)


        waypoint = Waypoint()
        waypoint.frame = 3
        waypoint.command = 18 # loiter turns 
        waypoint.is_current = True
        waypoint.autocontinue = True
        waypoint.x_lat = self.waypoints[0] # latitude
        waypoint.y_long = self.waypoints[1] # longitude
        waypoint.z_alt = self.waypoints[2]  # altitude
        waypoint.param1 = 1 # turns (N * 360)
        waypoint.param2 = 0
        waypoint.param3 = 10 # radius uints are in meters
        waypoint.param4 = 0
        self.waypoint_list.waypoints.append(waypoint)

        # push waypoints on the flight control unit
        self.push_waypoints(self.waypoint_list.waypoints)


    def push_waypoints(self, waypoints):
        rospy.wait_for_service('/mavros/mission/push')
        try:
            push_service = rospy.ServiceProxy('/mavros/mission/push', WaypointPush)
            response = push_service(0, waypoints)  # 0 indicates a new mission
            if response.success:
                rospy.loginfo("Waypoints push successfully...!")
            else:
                rospy.loginfo("Failed to push waypoints.")

        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

    def clear_waypoints(self):
        rospy.wait_for_service('/mavros/mission/clear')
        try:
            clear_service = rospy.ServiceProxy('/mavros/mission/clear', WaypointClear)
            response = clear_service()
            if response.success:
                rospy.loginfo("Waypoints are cleared successfully...!")
            else:
                rospy.loginfo("Failed to clear waypoints.")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

    def set_current(self):
        rospy.wait_for_service('/mavros/mission/set_current')
        try:
            set_current = rospy.ServiceProxy('/mavros/mission/set_current', WaypointSetCurrent)
            request = WaypointSetCurrentRequest()
            request.wp_seq = 0
            response = set_current(request)
            if response.success:
                rospy.loginfo("Set waypoints index successfully...!")
            else:
                rospy.loginfo("Failed to set waypoint index...")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

    def pull_waypoints(self):
        rospy.wait_for_service('/mavros/mission/push')
        try:
            pull_service = rospy.ServiceProxy('/mavros/mission/pull', WaypointPull)
            response = pull_service()
            if response.success:
                rospy.loginfo("wp_received: {}".format(response.wp_received))
            else:
                rospy.loginfo("Failed to pull waypoints.")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

    def start_waypoint_mission(self):
        if self.flight_mode.get_vehicle_mode() is not FlightModeList.MODE_AUTO:
            self.flight_mode.set_vehicle_mode(FlightModeList.MODE_AUTO)
            # setting mode auto will start the mission


# Example Usage:
if __name__ == '__main__':
    rospy.init_node('waypoint_mission_node', anonymous=True)
    wp_mission = HotpointMission() # init Hotpoint Mission class
    param = Param() # init param class
    param.set_param(Parameters.PARAM_AUTO_OPTIONS, 3, 0) # change the auto mode setting
    flight_control = FlightTaskControl() # init flight task control
    flight_control.turn_on_motors() # turn on motors
    waypoints = [-35.36215075, 149.16507305, 100] # sample waypoints Latitude Longitude Altitude
    wp_mission.handle_waypoint(waypoints) # waypoints to be executed
    wp_mission.start_waypoint_mission() # start waypoint mission
    rospy.spin()