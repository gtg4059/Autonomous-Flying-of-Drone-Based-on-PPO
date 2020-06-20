#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, Thrust
from mavros_msgs.srv import CommandBool, SetMode
from squaternion import Quaternion

# callback method for state sub
current_state = State() 
offb_set_mode = SetMode
def state_cb(state):
    global current_state
    current_state = state

#count=0
#local_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)
local_pos_pub = rospy.Publisher("/mavros/setpoint_attitude/attitude", PoseStamped, queue_size=10)
local_thr_pub = rospy.Publisher("/mavros/setpoint_attitude/thrust", Thrust, queue_size=10)
state_sub = rospy.Subscriber("/mavros/state", State, state_cb)
arming_client = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
set_mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode) 

T = Thrust()
T.thrust = 0.2
pose = PoseStamped()
# pose.pose.position.x = 0
# pose.pose.position.y = 0
# pose.pose.position.z = 0
q = Quaternion.from_euler(30, 30, 90, degrees=True)
pose.pose.orientation.w = q.w
pose.pose.orientation.x = q.x
pose.pose.orientation.y = q.y
pose.pose.orientation.z = q.z
def position_control():
    # global count
    rospy.init_node('offb_node', anonymous=True)
    prev_state = current_state
    rate = rospy.Rate(20.0) # MUST be more then 2Hz

    # send a few setpoints before starting
    for i in range(100):
        local_thr_pub.publish(T)
        local_pos_pub.publish(pose)
        rate.sleep()
    
    # wait for FCU connection
    while not current_state.connected:
        rate.sleep()

    last_request = rospy.get_rostime()
    while not rospy.is_shutdown():
        now = rospy.get_rostime()
        if current_state.mode != "OFFBOARD" and (now - last_request > rospy.Duration(5.)):
            set_mode_client(base_mode=0, custom_mode="OFFBOARD")
            last_request = now 
        else:
            if not current_state.armed and (now - last_request > rospy.Duration(5.)):
               arming_client(True)
               last_request = now 

        # older versions of PX4 always return success==True, so better to check Status instead
        if prev_state.armed != current_state.armed:
            rospy.loginfo("Vehicle armed: %r" % current_state.armed)
        if prev_state.mode != current_state.mode: 
            rospy.loginfo("Current mode: %s" % current_state.mode)
        prev_state = current_state

        # Update timestamp and publish pose 
        pose.header.stamp = rospy.Time.now()
        T.header.stamp = rospy.Time.now()
        # pose.header.seq = count
        # count+=1
        # pose.header.frame_id="1"
        local_thr_pub.publish(T)
        local_pos_pub.publish(pose)
        rate.sleep()

if __name__ == '__main__':
    try:
        position_control()
    except rospy.ROSInterruptException:
        pass