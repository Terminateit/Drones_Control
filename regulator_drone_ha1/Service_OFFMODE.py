#!/usr/bin/python
import rospy
import mavros
from geometry_msgs.msg import PoseStamped, TwistStamped, AccelWithCovarianceStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from nav_msgs.msg import Odometry
import numpy as np
import math
from tf.transformations import euler_from_quaternion

mavros.set_namespace()

## CALLBACK FUNCTRIONS
current_state = State() 
offb_set_mode = SetMode
def state_cb(state):
    global current_state
    current_state = state

current_pose = PoseStamped() 
def pose_cb(pose):
    global current_pose
    current_pose = pose

current_odom = Odometry() 
def odom_cb(odom):
    global current_odom
    current_odom = odom

current_acc = AccelWithCovarianceStamped() 
def acc_cb(accel):
    global current_accel
    current_accel = accel

current_vel_loc = TwistStamped() 
def vel_loc_cb(velocity_local):
    global current_vel_loc
    current_vel_loc = velocity_local

current_goal = PoseStamped() 
def goal_cb(goal):
    global current_goal
    current_goal = goal
# Function to limit the number between max and min (modul)

def limit_number(number,min = -100,max = 100):
    if number > max:
        number = max
    if number < min:
        number = min 
    return number   
# Define Subscribers, Publishers.
local_pos_pub = rospy.Publisher(mavros.get_topic('setpoint_position', 'local'), PoseStamped, queue_size=10)
state_sub = rospy.Subscriber(mavros.get_topic('state'), State, state_cb)
arming_client = rospy.ServiceProxy(mavros.get_topic('cmd', 'arming'), CommandBool)
set_mode_client = rospy.ServiceProxy(mavros.get_topic('set_mode'), SetMode) 

pose_sub = rospy.Subscriber(mavros.get_topic('local_position', 'pose'), PoseStamped, pose_cb)
odom_sub = rospy.Subscriber(mavros.get_topic('local_position','odom'), Odometry, odom_cb)
accel_sub = rospy.Subscriber(mavros.get_topic('local_position','accel'), AccelWithCovarianceStamped, acc_cb)
velocity_sub = rospy.Subscriber(mavros.get_topic('local_position','velocity_local'), TwistStamped, vel_loc_cb)
goal_sub = rospy.Subscriber('/goal', PoseStamped, goal_cb)

vel_control = TwistStamped()
vel_cmd_pub = rospy.Publisher(mavros.get_topic('setpoint_velocity', 'cmd_vel'), TwistStamped, queue_size=10)

## Function for the whole control. Drone goes to the goal point
def position_control():
    rospy.init_node('drone_to_go', anonymous=True)
    prev_state = current_state
    rate = rospy.Rate(20.0) # MUST be more then 2Hz
    
    # wait for FCU connection
    while not current_state.connected:
        rate.sleep()

    last_request = rospy.get_rostime()

    # Parameters of PD-controller 
    p = 5
    d = 0.5
    
    orientation_goal = (current_goal.pose.orientation.x, current_goal.pose.orientation.y, current_goal.pose.orientation.z, current_goal.pose.orientation.w)
    phi_goal, theta_goal, tau_goal = euler_from_quaternion(orientation_goal)

    # Check OFFBOAD or Armed
    while not rospy.is_shutdown():
        now = rospy.get_rostime()
        if current_state.mode != "OFFBOARD" and (now - last_request > rospy.Duration(5.)):
            set_mode_client(base_mode=0, custom_mode="OFFBOARD")
            last_request = now 
        else:
            if not current_state.armed and (now - last_request > rospy.Duration(5.)):
               arming_client(True)
               last_request = now 
        if prev_state.armed != current_state.armed:
            rospy.loginfo("Vehicle armed: %r" % current_state.armed)
        if prev_state.mode != current_state.mode: 
            rospy.loginfo("Current mode: %s" % current_state.mode)
        prev_state = current_state
        
        ## VELOCITY CONTROL

        # Get current coal point, orientation
        orientation_goal = (current_goal.pose.orientation.x, current_goal.pose.orientation.y, current_goal.pose.orientation.z, current_goal.pose.orientation.w)
        phi_goal, theta_goal, tau_goal = euler_from_quaternion(orientation_goal)

        # Linear Velocity Control 
        vel_control_x = p*(current_goal.pose.position.x-current_pose.pose.position.x)+d*(0-current_vel_loc.twist.linear.x)
        vel_control_y = p*(current_goal.pose.position.y-current_pose.pose.position.y)+d*(0-current_vel_loc.twist.linear.y)
        vel_control_z = p*(current_goal.pose.position.z-current_pose.pose.position.z)+d*(0-current_vel_loc.twist.linear.z)
        vel_control.twist.linear.x = limit_number(vel_control_x,min = -2,max = 2)
        vel_control.twist.linear.y = limit_number(vel_control_y,min = -2,max = 2)
        vel_control.twist.linear.z = limit_number(vel_control_z,min = -1.5,max = 1.5)
        # Angular Velocity Control
        orientation = (current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w)
        phi, theta, tau  = euler_from_quaternion(orientation)

        vel_control.twist.angular.x = 0
        vel_control.twist.angular.y = 0
        vel_control.twist.angular.z = p*(tau_goal-tau)
        
        vel_control.header.stamp = rospy.Time.now()
        vel_cmd_pub.publish(vel_control)
        rate.sleep()


if __name__ == '__main__':
    try:
        position_control()
    except rospy.ROSInterruptException:
        pass
