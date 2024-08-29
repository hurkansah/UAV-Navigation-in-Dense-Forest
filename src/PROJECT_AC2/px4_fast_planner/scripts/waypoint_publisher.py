#!/usr/bin/env python3

"""@trajectory_msg_converter.py
It is a node, to send goals to the path planner. Aim send to goals in an array. 
Authors: Sebnem Sariozkan
"""

# Imports
import rospy 
import actionlib
import mavros
from mavros_msgs.msg import State
from geometry_msgs.msg import  PoseStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal # type: ignore
from std_msgs.msg import String
from math import radians, degrees
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion

def state_cb(data):
    global current_state
    current_state = data

def pose_cb_z(data):
    global current_pose_z
    current_pose_z = data.pose.position.z;

def fp_stater(data):
    global stater_fp 
    stater_fp = data.data


def fp_emer(data):
    global state_emer
    state_emer = data.data


def goal_creater(map_x,map_y,z_curr):
    a = 2
    len_m = 10
    wps = []
    for i in range(1,len_m):
        wps.append([a*i,-map_y+a*i,z_curr])
        wps.append([map_x-a*i,map_y+a*i,z_curr])
        wps.append([map_x-a*i,a*i,z_curr])
        wps.append([a*i,a*i,z_curr])
    return wps

def scenario2(curr_x,curr_y,z_curr):
    a = 2
    m = 2
    len_m = 40
    wps = []
    for i in range(1,len_m):
        #wps.append([5,0,z_curr])
        wps.append([30,0,z_curr])
        wps.append([30,-30,z_curr])
        wps.append([10,-30,z_curr])
        wps.append([10, 0,z_curr])
        wps.append([10, 30,z_curr])
        wps.append([28,28,z_curr]) 

    return wps

def scenario1(curr_x,curr_y,z_curr):
    a = 2
    m = 2
    len_m = 40
    wps = []
    # Goals for the dense forest, path planner test 
    for i in range(1,len_m):
        #wps.append([5,0,z_curr])
        wps.append([60,0,z_curr]) #60
        wps.append([0,0,z_curr])
    return wps

def movebase_client():
    # position and yaw
    #Subscribers and publishers 
    rospy.init_node('movebase_publisher_py')
    rate = rospy.Rate(1)
    pub_goal = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)
    frame_id = rospy.get_param('~goals_move_frame', 'map')
    mavros_state = rospy.Subscriber('mavros/state',State,callback = state_cb)
    fp_states = rospy.Subscriber('/planning/fp_state',String,callback = fp_stater)
    emer_states = rospy.Subscriber('/planning/fp_emergency_state',String,callback = fp_emer)
    mavros_pose_z = rospy.Subscriber('mavros/local_position/pose',PoseStamped,callback = pose_cb_z)
    stater_fp = ""#!/usr/bin/env python3

"""@trajectory_msg_converter.py
It is a node, to send goals to the path planner. Aim send to goals in an array. 
Authors: Sebnem Sariozkan
"""

# Imports
import rospy 
import actionlib
import mavros
from mavros_msgs.msg import State
from geometry_msgs.msg import  PoseStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal # type: ignore
from std_msgs.msg import String
from math import radians, degrees
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion

def state_cb(data):
    global current_state
    current_state = data

def pose_cb_z(data):
    global current_pose_z
    current_pose_z = data.pose.position.z;

def fp_stater(data):
    global stater_fp 
    stater_fp = data.data


def fp_emer(data):
    global state_emer
    state_emer = data.data


def goal_creater(map_x,map_y,z_curr):
    a = 2
    len_m = 10
    wps = []
    for i in range(1,len_m):
        wps.append([a*i,-map_y+a*i,z_curr])
        wps.append([map_x-a*i,map_y+a*i,z_curr])
        wps.append([map_x-a*i,a*i,z_curr])
        wps.append([a*i,a*i,z_curr])
    return wps

def scenario2(curr_x,curr_y,z_curr):
    a = 2
    m = 2
    len_m = 40
    wps = []
    for i in range(1,len_m):
        #wps.append([5,0,z_curr])
        wps.append([30,0,z_curr])
        wps.append([30,-30,z_curr])
        wps.append([10,-30,z_curr])
        wps.append([10, 0,z_curr])
        wps.append([10, 30,z_curr])
        wps.append([28,28,z_curr]) 

    return wps

def scenario1(curr_x,curr_y,z_curr):
    a = 2
    m = 2
    len_m = 40
    wps = []
    # Goals for the dense forest, path planner test 
    for i in range(1,len_m):
        #wps.append([5,0,z_curr])
        wps.append([60,0,z_curr]) #60
        wps.append([0,0,z_curr])
    return wps

def movebase_client():
    # position and yaw
    #Subscribers and publishers 
    rospy.init_node('movebase_publisher_py')
    rate = rospy.Rate(1)
    pub_goal = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)
    frame_id = rospy.get_param('~goals_move_frame', 'map')
    mavros_state = rospy.Subscriber('mavros/state',State,callback = state_cb)
    fp_states = rospy.Subscriber('/planning/fp_state',String,callback = fp_stater)
    emer_states = rospy.Subscriber('/planning/fp_emergency_state',String,callback = fp_emer)
    mavros_pose_z = rospy.Subscriber('mavros/local_position/pose',PoseStamped,callback = pose_cb_z)
    stater_fp = ""
    goal = PoseStamped()
    mapx = 20
    mapy = 20
    rate.sleep()
    zcurr = 3
    Wps = scenario2(mapx,mapy,zcurr)
    count = 0
    flag_exec = 0
    for array in Wps:
        flag_exec =0
        while flag_exec==0: 
            rate.sleep()
            rospy.loginfo(stater_fp)
            if count == 0: 
                goal.header.frame_id = frame_id
                goal.header.stamp = rospy.Time.now()
                goal.pose.position.x = array[0]
                goal.pose.position.y = array[1]
                goal.pose.position.z = zcurr
                # take zaw goal 
                #angle = radians(array[3])
                #quat = quaternion_from_euler(0.0, 0.0, angle)
                #goal.pose.orientation = Quaternion(*quat.tolist())
                pub_goal.publish(goal)
                rate.sleep()
                flag_exec = 1

            if stater_fp == "WAIT_TARGET": #height>2.0 and 
                # Adjust the goal message 
                goal.header.frame_id = frame_id
                goal.header.stamp = rospy.Time.now()
                goal.pose.position.x = array[0]
                goal.pose.position.y = array[1]#!/usr/bin/env python3

"""@trajectory_msg_converter.py
It is a node, to send goals to the path planner. Aim send to goals in an array. 
Authors: Sebnem Sariozkan
"""

# Imports
import rospy 
import actionlib
import mavros
from mavros_msgs.msg import State
from geometry_msgs.msg import  PoseStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal # type: ignore
from std_msgs.msg import String
from math import radians, degrees
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion

def state_cb(data):
    global current_state
    current_state = data

def pose_cb_z(data):
    global current_pose_z
    current_pose_z = data.pose.position.z;

def fp_stater(data):
    global stater_fp 
    stater_fp = data.data


def fp_emer(data):
    global state_emer
    state_emer = data.data


def goal_creater(map_x,map_y,z_curr):
    a = 2
    len_m = 10
    wps = []
    for i in range(1,len_m):
        wps.append([a*i,-map_y+a*i,z_curr])
        wps.append([map_x-a*i,map_y+a*i,z_curr])
        wps.append([map_x-a*i,a*i,z_curr])
        wps.append([a*i,a*i,z_curr])
    return wps

def scenario2(curr_x,curr_y,z_curr):
    a = 2
    m = 2
    len_m = 40
    wps = []
    for i in range(1,len_m):
        #wps.append([5,0,z_curr])
        wps.append([30,0,z_curr])
        wps.append([30,-30,z_curr])
        wps.append([10,-30,z_curr])
        wps.append([10, 0,z_curr])
        wps.append([10, 30,z_curr])
        wps.append([28,28,z_curr]) 

    return wps

def scenario1(curr_x,curr_y,z_curr):
    a = 2
    m = 2
    len_m = 40
    wps = []
    # Goals for the dense forest, path planner test 
    for i in range(1,len_m):
        #wps.append([5,0,z_curr])
        wps.append([60,0,z_curr]) #60
        wps.append([0,0,z_curr])
    return wps

def movebase_client():
    # position and yaw
    #Subscribers and publishers 
    rospy.init_node('movebase_publisher_py')
    rate = rospy.Rate(1)
    pub_goal = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)
    frame_id = rospy.get_param('~goals_move_frame', 'map')
    mavros_state = rospy.Subscriber('mavros/state',State,callback = state_cb)
    fp_states = rospy.Subscriber('/planning/fp_state',String,callback = fp_stater)
    emer_states = rospy.Subscriber('/planning/fp_emergency_state',String,callback = fp_emer)
    mavros_pose_z = rospy.Subscriber('mavros/local_position/pose',PoseStamped,callback = pose_cb_z)
    stater_fp = ""
    goal = PoseStamped()
    mapx = 20
    mapy = 20
    rate.sleep()
    zcurr = 3
    Wps = scenario2(mapx,mapy,zcurr)
    count = 0
    flag_exec = 0
    for array in Wps:
        flag_exec =0
        while flag_exec==0: 
            rate.sleep()
            rospy.loginfo(stater_fp)
            if count == 0: 
                goal.header.frame_id = frame_id
                goal.header.stamp = rospy.Time.now()
                goal.pose.position.x = array[0]
                goal.pose.position.y = array[1]
                goal.pose.position.z = zcurr
                # take zaw goal 
                #angle = radians(array[3])
                #quat = quaternion_from_euler(0.0, 0.0, angle)
                #goal.pose.orientation = Quaternion(*quat.tolist())
                pub_goal.publish(goal)
                rate.sleep()
                flag_exec = 1

            if stater_fp == "WAIT_TARGET": #height>2.0 and 
                # Adjust the goal message 
                goal.header.frame_id = frame_id
                goal.header.stamp = rospy.Time.now()
                goal.pose.position.x = array[0]
                goal.pose.position.y = array[1]
                goal.pose.position.z = zcurr
                
                pub_goal.publish(goal)
                rate.sleep()
                rospy.spin()
                
            while stater_fp == "EXEC_TRAJ": 
                rospy.loginfo("It is creating trajectory")
                rospy.loginfo(array)
                rate.sleep()
                #if stater_fp == "WAIT_TARGET" or state_emer== "STOP": 
                #break
                flag_exec = 1
                rospy.spin()
        rospy.spin()
 
        count = count + 1 
        rospy.loginfo(count)

    
if __name__ == '__main__':
    try: 
        movebase_client()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
                goal.pose.position.z = zcurr
                
                pub_goal.publish(goal)
                rate.sleep()
                rospy.spin()
                
            while stater_fp == "EXEC_TRAJ": 
                rospy.loginfo("It is creating trajectory")
                rospy.loginfo(array)
                rate.sleep()
                #if stater_fp == "WAIT_TARGET" or state_emer== "STOP": 
                #break
                flag_exec = 1
                rospy.spin()
        rospy.spin()
 
        count = count + 1 
        rospy.loginfo(count)

    
if __name__ == '__main__':
    try: 
        movebase_client()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
    goal = PoseStamped()
    mapx = 20
    mapy = 20
    rate.sleep()
    zcurr = 3
    Wps = scenario2(mapx,mapy,zcurr)
    count = 0
    flag_exec = 0
    for array in Wps:
        flag_exec =0
        while flag_exec==0: 
            rate.sleep()
            rospy.loginfo(stater_fp)
            if count == 0: 
                goal.header.frame_id = frame_id
                goal.header.stamp = rospy.Time.now()
                goal.pose.position.x = array[0]
                goal.pose.position.y = array[1]
                goal.pose.position.z = zcurr
                # take zaw goal 
                #angle = radians(array[3])
                #quat = quaternion_from_euler(0.0, 0.0, angle)
                #goal.pose.orientation = Quaternion(*quat.tolist())
                pub_goal.publish(goal)
                rate.sleep()
                flag_exec = 1

            if stater_fp == "WAIT_TARGET": #height>2.0 and 
                # Adjust the goal message 
                goal.header.frame_id = frame_id
                goal.header.stamp = rospy.Time.now()
                goal.pose.position.x = array[0]
                goal.pose.position.y = array[1]
                goal.pose.position.z = zcurr
                
                pub_goal.publish(goal)
                rate.sleep()
                rospy.spin()
                
            while stater_fp == "EXEC_TRAJ": 
                rospy.loginfo("It is creating trajectory")
                rospy.loginfo(array)
                rate.sleep()
                #if stater_fp == "WAIT_TARGET" or state_emer== "STOP": 
                #break
                flag_exec = 1
                rospy.spin()
        rospy.spin()
 
        count = count + 1 
        rospy.loginfo(count)

    
if __name__ == '__main__':
    try: 
        movebase_client()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")