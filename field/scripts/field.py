#!/usr/bin/python

import math
import rospy
import mavros

from geometry_msgs.msg import PoseArray, PoseStamped, TwistStamped
from tf.transformations import euler_from_quaternion

from dynamic_reconfigure.msg import Config # dynamics reconfigure

mavros.set_namespace()

pose = PoseStamped()
velocity = TwistStamped()
obstacles = PoseArray()

config = Config() # dynamics reconfigure

field = TwistStamped()

def pose_cb(data):
    global pose
    pose = data

def velocity_cb(data):
    global velocity
    velocity = data

def obstacles_cb(data):
    global obstacles
    obstacles = data

def reconfigure_cb(data): # dynamics reconfigure callback
    global config
    config = data


pose_topic = mavros.get_topic('local_position', 'pose')
pose_sub = rospy.Subscriber(pose_topic, PoseStamped, pose_cb)

velocity_topic = mavros.get_topic('local_position', 'velocity_local')
velocity_sub = rospy.Subscriber(velocity_topic, TwistStamped, velocity_cb)

obstacles_sub = rospy.Subscriber('/obstacles', PoseArray, obstacles_cb)

reconfigure_sub = rospy.Subscriber('/reconfigure_server/parameter_updates', Config, reconfigure_cb) # dynamics reconfigure subscriber

field_pub = rospy.Publisher('/field', TwistStamped, queue_size=10)

def power_of_field(force, tolerance, distance, c=1):
    return force*(c/distance - c/tolerance)**2

def potential_field(force = 20, tolerance = 2):
    rospy.init_node('potential_field', anonymous=True)
    rate = rospy.Rate(20.0)  # MUST be more then 2Hz

    while not rospy.is_shutdown():
        for i in range(len(config.doubles)):
            if config.doubles[i].name == 'force':
                force = config.doubles[i].value
            if config.doubles[i].name == 'tolerance':
                tolerance = config.doubles[i].value

        length = len(obstacles.poses)
        diff_x = [0] * length
        diff_y = [0] * length
        distance = [0] * length
        for i in range(length):
            diff_x[i] = pose.pose.position.x - obstacles.poses[i].position.x
            diff_y[i] = pose.pose.position.y - obstacles.poses[i].position.y
            distance[i] = math.sqrt(diff_x[i]**2 + diff_y[i]**2)

        # find the closest obstacle
        closest = distance.index(min(distance))
        print(closest, distance[closest])

        # set the linear part of the potential filed
        if distance[closest] < tolerance:
            dir_x = diff_x[closest] / math.sqrt(diff_x[closest]**2 + diff_y[closest]**2)
            dir_y = diff_y[closest] / math.sqrt(diff_x[closest]**2 + diff_y[closest]**2)
            power = power_of_field(force, tolerance, distance[closest])

            field.twist.linear.x = power * dir_x
            field.twist.linear.y = power * dir_y
            field.twist.linear.z = 0
        else:
            field.twist.linear.x = 0
            field.twist.linear.y = 0
            field.twist.linear.z = 0

        # set the angular part of the potential filed
        field.twist.angular.x = 0
        field.twist.angular.y = 0
        field.twist.angular.z = 0

        # update timestamp
        field.header.stamp = rospy.Time.now()
        
        # publish control
        field_pub.publish(field)

        rate.sleep()


if __name__ == '__main__':
    try:
        potential_field()
    except rospy.ROSInterruptException:
        pass