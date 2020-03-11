#!/usr/bin/python

import rospy
import mavros

from mavros.command import arming
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

from geometry_msgs.msg import PoseStamped, TwistStamped
from tf.transformations import euler_from_quaternion

from dynamic_reconfigure.msg import Config # dynamics reconfigure



mavros.set_namespace()

current_state = State()

pose = PoseStamped()
velocity = TwistStamped()
goal = PoseStamped()
field = TwistStamped()

control_input = TwistStamped()


config = Config() # dynamics reconfigure

def state_cb(state):
    global current_state
    current_state = state

def pose_cb(data):
    global pose
    pose = data

def velocity_cb(data):
    global velocity
    velocity = data

def goal_cb(data):
    global goal
    goal = data

def field_cb(data):
    global field
    field = data

def reconfigure_cb(data): # dynamics reconfigure callback
    global config
    config = data

state_sub = rospy.Subscriber(mavros.get_topic('state'), State, state_cb)
arming_client = rospy.ServiceProxy(mavros.get_topic('cmd', 'arming'), CommandBool)
set_mode_client = rospy.ServiceProxy(mavros.get_topic('set_mode'), SetMode)

pose_topic = mavros.get_topic('local_position', 'pose')
pose_sub = rospy.Subscriber(pose_topic, PoseStamped, pose_cb)

velocity_topic = mavros.get_topic('local_position', 'velocity_local')
velocity_sub = rospy.Subscriber(velocity_topic, TwistStamped, velocity_cb)

goal_sub = rospy.Subscriber('/goal', PoseStamped, goal_cb)
field_sub = rospy.Subscriber('/field', TwistStamped, field_cb)

velocity_control_topic = mavros.get_topic('setpoint_velocity', 'cmd_vel')
velocity_control_pub = rospy.Publisher(velocity_control_topic, TwistStamped, queue_size=10)

reconfigure_sub = rospy.Subscriber('/reconfigure_server/parameter_updates', Config, reconfigure_cb) # dynamics reconfigure subscriber

def velocity_control(k_p = 3, k_d = 0.05):
    rospy.init_node('offboard_control', anonymous=True)
    prev_state = current_state
    rate = rospy.Rate(20.0)  # MUST be more then 2Hz

    # wait for connection
    while not current_state.connected:
        rate.sleep()

    last_request = rospy.get_rostime()

    while not rospy.is_shutdown():
        for i in range(len(config.doubles)):
            if config.doubles[i].name == 'k_p':
                k_p = config.doubles[i].value
            if config.doubles[i].name == 'k_d':
                k_d = config.doubles[i].value
        
        now = rospy.get_rostime()
        if current_state.mode != "OFFBOARD" and (now - last_request > rospy.Duration(5.)):
            set_mode_client(base_mode=0, custom_mode="OFFBOARD")
            last_request = now
        else:
            if not current_state.armed and (now - last_request > rospy.Duration(5.)):
                arming_client(True)
                last_request = now

        # check the status
        if prev_state.armed != current_state.armed:
            rospy.loginfo("Vehicle armed: %r" % current_state.armed)
        if prev_state.mode != current_state.mode:
            rospy.loginfo("Current mode: %s" % current_state.mode)
        prev_state = current_state

        # set the linear part of the control input
        control_input.twist.linear.x = k_p * (goal.pose.position.x - pose.pose.position.x) - k_d*(velocity.twist.linear.x) + field.twist.linear.x
        control_input.twist.linear.y = k_p * (goal.pose.position.y - pose.pose.position.y) - k_d*(velocity.twist.linear.y) + field.twist.linear.y
        control_input.twist.linear.z = k_p * (goal.pose.position.z - pose.pose.position.z) - k_d*(velocity.twist.linear.z) + field.twist.linear.z

        # set the angular part of the control input
        orientation = pose.pose.orientation
        orientation = [orientation.x, orientation.y, orientation.z, orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation)

        goal_orientation = goal.pose.orientation
        goal_orientation = [goal_orientation.x, goal_orientation.y, goal_orientation.z, goal_orientation.w]
        (goal_roll, goal_pitch, goal_yaw) = euler_from_quaternion(goal_orientation)

        control_input.twist.angular.x = 0
        control_input.twist.angular.y = 0
        control_input.twist.angular.z = k_p * (goal_yaw - yaw)

        # check the limits
        if control_input.twist.linear.x > 2:
            control_input.twist.linear.x = 2
        elif control_input.twist.linear.x < -2:
            control_input.twist.linear.x = -2
            
        if control_input.twist.linear.y > 2:
            control_input.twist.linear.y = 2
        elif control_input.twist.linear.y < -2:
            control_input.twist.linear.y = -2

        if control_input.twist.linear.z > 1.5:
            control_input.twist.linear.z = 1.5
        elif control_input.twist.linear.z < -1.5:
            control_input.twist.linear.z = -1.5
    
        # update timestamp
        control_input.header.stamp = rospy.Time.now()
        
        # publish control
        velocity_control_pub.publish(control_input)

        rate.sleep()


if __name__ == '__main__':
    try:
        velocity_control()
    except rospy.ROSInterruptException:
        pass