#!/usr/bin/env python

"""
Author: Haroon Rasheed
Date: 13 March 2024
"""


class Parameters:
    # RTL PARAMETERS
    PARAM_RTL_ALT = 'RTL_ALT'  # The minimum alt above home the vehicle will climb to before returning. Range 200-8000 units in cm
    PARAM_RTL_SPEED = 'RTL_SPEED'  # Defines the speed in cm/s which the aircraft will attempt to maintain horizontally while flying home.  Range 0 to 2000
    PARAM_RTL_LOIT_TIME = 'RTL_LOIT_TIME'  # Time (in milliseconds) to loiter above home before beginning final descent
    PARAM_RTL_ALT_FINAL = 'RTL_ALT_FINAL'  # This is the altitude the vehicle will move to as the final stage of Returning to Launch or after completing a mission. Set to zero to land.

    # WAYPOINT PARAMETERS
    PARAM_WPNAV_SPEED = 'WPNAV_SPEED'  # Defines the speed in cm/s which the aircraft will attempt to maintain horizontally during a WP mission.  Range 20 to 2000
    PARAM_WPNAV_RADIUS = 'WPNAV_RADIUS'  # Defines the distance from a waypoint, that when crossed indicates the wp has been hit. Range 5 to 1000
    PARAM_WPNAV_SPEED_UP = 'WPNAV_SPEED_UP'  # Defines the speed in cm/s which the aircraft will attempt to maintain while climbing during a WP mission. Range 10 to 1000
    PARAM_WPNAV_SPEED_DN = 'WPNAV_SPEED_DN'  # Defines the speed in cm/s which the aircraft will attempt to maintain while descending during a WP mission. Range 10 to 500
    PARAM_WPNAV_ACCEL = 'WPNAV_ACCEL'  # Defines the horizontal acceleration in cm/s/s used during missions. Range 50 to 500
    PARAM_WP_YAW_BEHAVIOR = 'WP_YAW_BEHAVIOR'  # Determines how the autopilot controls the yaw during missions and RTL  Never Change Yaw: 0 , Face Next Waypoint: 1 , Face next waypoint except RTL: 2 , Face along GPS course: 3

    # Failsafe
    PARAM_RC_FS_TIMEOUT = 'RC_FS_TIMEOUT'  # RC failsafe will trigger this many seconds after loss of RC. Range 0.5 to 10.0 seconds
    PARAM_FS_GCS_TIMEOUT = 'FS_GCS_TIMEOUT'  # Timeout before triggering the GCS failsafe Range 2 to 120 seconds
    PARAM_BATT_FS_LOW_ACT = 'BATT_FS_LOW_ACT'  # What action the vehicle should perform if it hits a low battery failsafe  0: None 1: Land 2: RTL
    PARAM_BATT_FS_CRT_ACT = 'BATT_FS_CRT_ACT'  # What action the vehicle should perform if it hits a critical battery failsafe 0: None 1: Land 2: RTL
    PARAM_FS_GCS_ENABLE = 'FS_GCS_ENABLE'  # Controls whether failsafe will be invoked (and what action to take) when connection with Ground station is lost for at least 5 seconds 0: None 1: RTL
    PARAM_FS_THR_ENABLE = 'FS_THR_ENABLE'  # The throttle failsafe allows you to configure a software failsafe activated by a setting on the throttle input channel
    PARAM_BATT_CRT_VOLT = 'BATT_CRT_VOLT'  # Battery voltage that triggers a critical battery failsafe.
    PARAM_BATT_LOW_VOLT = 'BATT_LOW_VOLT'  # Battery voltage that triggers a low battery failsafe.
    PARAM_SIM_RC_FAIL = 'SIM_RC_FAIL'      # To Simulate Radio Failsafe
    

    # AUTO MODE
    PARAM_AUTO_OPTIONS = 'AUTO_OPTIONS'   # Allow Copter to arm or takeoff in the auto mode. 