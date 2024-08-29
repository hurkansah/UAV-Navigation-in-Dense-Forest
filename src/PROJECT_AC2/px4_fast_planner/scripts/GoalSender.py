#!/usr/bin/env python3

"""@trajectory_msg_converter.py
It is a node, to send goals to the path planner. Aim send to goals in an array. 
Authors: Sebnem Sariozkan
"""

# Imports
import rospy 
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import time


# Global variables
stater_fp = ""
current_pose_z = 0

def state_cb(data):
    global current_state
    current_state = data

def pose_cb_z(data):
    global current_pose_z
    current_pose_z = data.pose.position.z

def fp_stater(data):
    global stater_fp 
    stater_fp = data.data

def movebase_client():
    rospy.init_node('movebase_publisher_py')
    rate = rospy.Rate(1)  # Adjust the rate as needed
    pub_goal = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)
    mavros_pose_z = rospy.Subscriber('mavros/local_position/pose', PoseStamped, callback=pose_cb_z)
    fp_states = rospy.Subscriber('/planning/fp_state', String, callback=fp_stater)
    
    # Your other initialization steps
    frame_id = "map"
    mapx = 20
    mapy = 20
    zcurr = 3
    Wps = scenario2(mapx, mapy, zcurr)

    # Send the first waypoint immediately
    goal = PoseStamped()
    goal.header.frame_id = frame_id
    goal.header.stamp = rospy.Time.now()
    goal.pose.position.x = 5
    goal.pose.position.y = 0
    goal.pose.position.z = zcurr
    pub_goal.publish(goal)
    rate.sleep()
    
    for array in Wps:

        while stater_fp != "WAIT_TARGET" and pub_goal.get_num_connections() != 0:
            rate.sleep()

        if stater_fp == "WAIT_TARGET":
            goal = PoseStamped()
            goal.header.frame_id = frame_id
            goal.header.stamp = rospy.Time.now()
            goal.pose.position.x = array[0]
            goal.pose.position.y = array[1]
            goal.pose.position.z = zcurr
            rospy.loginfo(array)
            pub_goal.publish(goal)
            rate.sleep()

        rospy.loginfo("Received WAIT_TARGET message, sending next goal.")


def scenario3(curr_x, curr_y, z_curr):
    a = 2
    m = 2
    len_m = 40
    wps = []
    for i in range(1, len_m):
        wps.append([7,   7,3])
        wps.append([7,   30,3])
        wps.append([17,  40,3])
        wps.append([30,  40,3])
        wps.append([40,  30,3])
        wps.append([40, -30,3])
        wps.append([30, -40,3])
        wps.append([17, -40,3])
        wps.append([7,  -30,3])


       
    return wps

def scenario2(curr_x, curr_y, z_curr):
    a = 2
    m = 2
    len_m = 40
    wps = []
    for i in range(1, len_m):
        wps.append([10,  5,3])
        wps.append([10,  20,3])
        wps.append([17,  30,3])
        wps.append([30,  30,3])
        wps.append([40,  20,3])
        wps.append([40, -20,3])
        wps.append([30, -30,3])
        wps.append([17, -30,3])
        wps.append([10, -20,3])
    return wps

def scenario1(curr_x, curr_y, z_curr):
    a = 2
    m = 2
    len_m = 40
    wps = []
    for i in range(1, len_m):
        wps.append([50, 0, z_curr])
        wps.append([2, 0, z_curr])
    return wps


if __name__ == '__main__':
    try: 
        movebase_client()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")

