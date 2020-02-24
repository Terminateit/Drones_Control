#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from dynamic_tutorials.cfg import TutorialsTwistConfig

def callback(config, level):
    rospy.loginfo("""Twist params Request:{Vx},{Vy},{Vz},{Wz}""".format(**config))
    return config

if __name__ == "__main__":
    rospy.init_node("field", anonymous = False)

    srv = Server(TutorialsTwistConfig, callback)
    rospy.spin()