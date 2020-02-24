#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from dynamic_tutorials.cfg import TutorialsConfig

def callback(config, level):
    rospy.loginfo("""PD params Request: {Kp}, {Kd}""".format(**config))
    return config

if __name__ == "__main__":
    rospy.init_node("PD", anonymous = False)

    srv = Server(TutorialsConfig, callback)
    rospy.spin()