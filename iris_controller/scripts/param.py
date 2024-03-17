#!/usr/bin/env python

"""
Author: Haroon Rasheed
Date: 13 March 2024
"""


import rospy
from mavros_msgs.srv import *
from param_list import Parameters


class Param:
    def __init__(self):
        pass
    def set_param(self, param: str, value_integer: int, value_real: float):
        rospy.wait_for_service('/mavros/param/set')
        try:
            param_set = rospy.ServiceProxy('/mavros/param/set', ParamSet)
            data = ParamSetRequest()
            data.param_id = param
            data.value.integer = value_integer
            data.value.real = value_real
            response = param_set(data)
            if response.success:
                rospy.loginfo("The param value set successfully ")
            else:
                rospy.loginfo("Failed to set param.")
        except Exception as e:
            rospy.loginfo(e)


    def get_param(self, param: str):
        rospy.wait_for_service('/mavros/param/get')
        try:
            param_get = rospy.ServiceProxy('/mavros/param/get', ParamGet)
            data = ParamGetRequest()
            data.param_id = param
            response = param_get(data)
            if response.success:
                rospy.loginfo("The parameter is {} and the value is {}".format(param, response.value))
                return response.value
            else:
                rospy.loginfo("Failed to get parameter.")
        except Exception as e:
            rospy.loginfo(e)


# Example Usage:
if __name__ == '__main__':
    rospy.init_node("param_node", anonymous=True)
    param = Param()
    result = param.get_param(Parameters.PARAM_RTL_ALT)