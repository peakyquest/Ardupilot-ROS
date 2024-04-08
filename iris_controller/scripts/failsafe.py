#!/usr/bin/env python

"""
Author: Haroon Rasheed
Date: 8 April 2024
"""


import rospy
from param import Param
from param_list import Parameters


class Failsafe:
    def __init__(self):
        self.parameter = Param()
      

    def gcs_failsafe_timeout(self):
        # Timeout before triggering the GCS failsafe (unit timeout in seconds)
        self.parameter.set_param(Parameters.PARAM_FS_GCS_TIMEOUT, 0, 5.0) # Interger: 0 Real: 5.0 seconds 

    def action_gcs_fs(self):
        # 0 Disabled/NoAction
        # 1 RTL
        # 2 RTL or Continue with Mission in Auto Mode (Removed in 4.0+-see FS_OPTIONS)
        # 3 SmartRTL or RTL
        # 4 SmartRTL or Land
        # 5 Land 
        # 6 Auto DO_LAND_START or RTL
        # 7 Brake or Land 
        self.parameter.set_param(Parameters.PARAM_FS_GCS_ENABLE, 1, 0) # to simulate set the mavproxy heartbeat to 0 


    def rc_fs_timeout(self):
        # # Timeout before triggering the radio failsafe (unit timeout in seconds)
        self.parameter.set_param(Parameters.PARAM_RC_FS_TIMEOUT, 0, 0.5) # timeout radio signal loss


    def rc_fs(self):
        # 0 Disabled/NoAction
        # 1 Enabled always RTL
        # 2 Enabled Continue with Mission in Auto Mode (Removed in 4.0+)
        # 3 Enabled always Land
        # 4 Enabled always SmartRTL or RTL
        # 5 Enabled always SmartRTL or Land
        # 6 Enabled Auto DO_LAND_START or RTL
        # 7 Enabled always Brake or Land
        self.parameter.set_param(Parameters.PARAM_FS_THR_ENABLE, 1, 0) # vehicle perform RTL it hits a loss of rc failsafe.


    def simulate_radio_failsafe(self):
        # 0 Disabled/NoAction
        # 1 No RC pusles
        # 2 All Channels neutral except Throttle is 950us
        self.parameter.set_param(Parameters.PARAM_SIM_RC_FAIL, 0, 0) # Simulate whether the radio failsafe is working or not






# Example Usage
def main():
    rospy.init_node('failsafe', anonymous= True)
    failsafe = Failsafe()
    failsafe.gcs_failsafe_timeout()
    failsafe.action_gcs_fs()
    failsafe.rc_fs_timeout()
    failsafe.rc_fs()
    failsafe.simulate_radio_failsafe()
    rospy.spin()

if __name__ == '__main__':
    main()




        


    