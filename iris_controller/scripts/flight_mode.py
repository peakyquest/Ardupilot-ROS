#!/usr/bin/env python

"""
Author: Haroon Rasheed
Date: 11 March 2024
"""

import rospy
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode
from flight_mode_list import FlightModeList

class FlightMode:
    def __init__(self):
        self.mode_state = rospy.Subscriber("/mavros/state", State, self.vehicle_mode_cb)
        self.current_mode = None  # Initialize current_mode to None

    def set_vehicle_mode(self, mode):
        rospy.wait_for_service('/mavros/set_mode', timeout= 5)
        try:
            flight_mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
            is_mode_changed = flight_mode_service(custom_mode=mode)
            if is_mode_changed:
                rospy.loginfo("Flight mode changed to {}".format(mode))
            else:
                rospy.loginfo("Failed to change flight mode")
        except rospy.ServiceException as e:
            rospy.loginfo("service set_mode call failed: %s. {} Mode could not be set.".format(mode), str(e))

    def vehicle_mode_cb(self, data):
        self.current_mode = data.mode  # Store the current mode

    def get_vehicle_mode(self):
        while self.current_mode is None:
            rospy.sleep(0.1)
        return self.current_mode

# Example Usage
if __name__ == '__main__':
    rospy.init_node('flight_mode', anonymous=True)
    flight_mode = FlightMode()
    flight_mode.set_vehicle_mode(FlightModeList.MODE_GUIDED)
        

