#!/usr/bin/env python

"""
Author: Haroon Rasheed
Date: 11 March 2024
"""

import rospy
from mavros_msgs.srv import *
from mavros_msgs.msg import *
import time
from flight_mode_list import FlightModeList
from flight_mode import FlightMode


class FlightTaskControl:
    def __init__(self):
        self.get_flight_mode = FlightMode()

    def start_landing(self):
        rospy.wait_for_service('/mavros/cmd/land', timeout=10)
        try:
            land_service = rospy.ServiceProxy('/mavros/cmd/land', mavros_msgs.srv.CommandTOL)
            is_landing = land_service(altitude=0, latitude=0, longitude=0, min_pitch=0, yaw=0)
            if is_landing:
                rospy.loginfo("Start Landing")
            else:
                rospy.loginfo("Landing Failed")
        except rospy.ServiceException as e:
            rospy.loginfo("service land call failed: %s. The vehicle cannot land " % e)

    def turn_on_motors(self):
        self.get_flight_mode.set_vehicle_mode(FlightModeList.MODE_GUIDED)
        rospy.wait_for_service('/mavros/cmd/arming', timeout=10)
        try:
            arm_service = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            request = CommandBoolRequest()
            request.value = True
            response = arm_service(request)
            if response.success:
                rospy.loginfo("Armed Success")
            else:
                rospy.loginfo("Armed Failed... {}".format(response.result))
        except rospy.ServiceException as e:
            rospy.loginfo("Service arm call failed: %s" % e)

    def turn_off_motors(self):
        rospy.wait_for_service('/mavros/cmd/arming', timeout=10)
        try:
            arm_service = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            arm_service(value=False)
        except rospy.ServiceException as e:
            rospy.loginfo("Service arm call failed: %s" % e)

    def start_taking_off(self):
        self.turn_on_motors()
        rospy.wait_for_service('/mavros/cmd/takeoff')
        try:
            take_off_service = rospy.ServiceProxy('/mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
            take_off_service(altitude=2, latitude=0, longitude=0, min_pitch=0, yaw=0)
            if take_off_service:
                rospy.loginfo("Take off Success")
            else:
                rospy.loginfo("Take off Failed")
        except rospy.ServiceException as e:
            rospy.loginfo("Service takeoff call failed: %s" % e)

    def return_to_home(self):
        self.get_flight_mode.set_vehicle_mode(FlightModeList.MODE_RTL)

    def emergency_brake(self):
        self.get_flight_mode.set_vehicle_mode(FlightModeList.MODE_BRAKE)

# Example Usage:
if __name__ == '__main__':
     rospy.init_node("flight_task_control", anonymous=True)
     flight_task_control = FlightTaskControl()
     flight_task_control.start_taking_off()
     # After 10 seconds delay, land.
     time.sleep(10)
     flight_task_control.start_landing()
