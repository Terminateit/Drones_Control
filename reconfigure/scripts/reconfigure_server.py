#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from reconfigure.cfg import ReconfigureConfig

def callback(config, level):
    rospy.loginfo("""Configure : {k_p}, {k_d}, {force}, {tolerance}""".format(**config))
    return config

if __name__ == "__main__":
    rospy.init_node("gains", anonymous = False)
    srv = Server(ReconfigureConfig, callback)
    rospy.spin()