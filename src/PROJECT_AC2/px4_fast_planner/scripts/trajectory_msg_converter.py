#!/usr/bin/env python3

"""@trajectory_msg_converter.py
This node converts Fast-Planner reference trajectory message to MultiDOFJointTrajectory which is accepted by geometric_controller
Authors: Mohamed Abdelkader
"""

# Imports
import rospy
import numpy as np 
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint # for geometric_controller
from quadrotor_msgs.msg import PositionCommand # for Fast-Planner
from geometry_msgs.msg import Transform, Twist
from tf.transformations import quaternion_from_euler
from scipy.signal import butter, lfilter
from scipy.interpolate import interp1d
import math

class MessageConverter:
    def __init__(self):
        self.data_buffer = [] 
        self.data_buffer_rate = []
        self.window_size = 100
        cutoff_freq = 30  # Adjust cutoff frequency as needed
        fs = 100  # Sampling frequency, adjust according to your system
        order = 2  # Filter order, adjust as needed
        self.b, self.a = self.butter_lowpass(cutoff_freq, fs, order)
        
        rospy.init_node('trajectory_msg_converter')

        fast_planner_traj_topic = rospy.get_param('~fast_planner_traj_topic', 'planning/ref_traj')
        traj_pub_topic = rospy.get_param('~traj_pub_topic', 'command/trajectory')
        new_topic = 'command/rpy'

        # Publisher for geometric_controller
        self.traj_pub = rospy.Publisher(traj_pub_topic, MultiDOFJointTrajectory, queue_size=1)
        self.filter_pub = rospy.Publisher(new_topic, PositionCommand, queue_size=1)

        # Subscriber for Fast-Planner reference trajectory
        rospy.Subscriber(fast_planner_traj_topic, PositionCommand, self.fastPlannerTrajCallback, tcp_nodelay=True)

        rospy.spin()

    def YawSmoother(self,data):
        if len(data) >= self.window_size:
            smoothed_yaw = np.mean(data[-self.window_size:])
        else:
            smoothed_yaw = data[-1]; 
        return smoothed_yaw
    
    def MovingAverage(self,data):
        if len(data) >self.window_size:
            data = data[self.window_size:]
            window_size = 60
            padded_signal = np.pad(data, (window_size // 2, window_size // 2), mode='edge')
            filtered_signal = np.convolve(padded_signal, np.ones(window_size)/window_size, mode='valid')
            filtered_signal = filtered_signal[-1]
        else:
            filtered_signal = data[-1]
        return filtered_signal
    
    def MedianFilter(self, data):
        if len(data) > self.window_size:
            data = data[self.window_size:]
            window_size = 20
            filtered_signal = np.median(data[-window_size:])
        else:
            filtered_signal = data[-1]
        return filtered_signal
    


    
    def butter_lowpass(self, cutoff, fs, order=5):
        nyquist = 0.5 * fs
        normal_cutoff = cutoff / nyquist
        b, a = butter(order, normal_cutoff, btype='low', analog=False)
        return b, a
    
    def lowpass_filter(self,data):
        if len(data) >= self.window_size:
            filtered_data = lfilter(self.b, self.a, data)
        else:
            filtered_data = data
        return filtered_data
        

    def fastPlannerTrajCallback(self, msg):
        # position and yaw
        pose = Transform()
        pose.translation.x = msg.position.x
        pose.translation.y = msg.position.y
        pose.translation.z = msg.position.z
        self.data_buffer.append(msg.yaw)
        raw_yaw = msg.yaw
        #smoothed_angle =  self.lowpass_filter(raw_yaw)[-1]
        #smoothed_angle = self.YawSmoother(self.data_buffer)
        #rospy.logerr('Real yaw:'+str(msg.yaw)+'filtered:'+str(smoothed_angle))
        smoothed_angle = self.MedianFilter(self.data_buffer)
        q = quaternion_from_euler(0, 0, smoothed_angle) # RPY
        pose.rotation.x = q[0]
        pose.rotation.y = q[1]
        pose.rotation.z = q[2]
        pose.rotation.w = q[3]

        pose_read = PositionCommand()
        pose_read.yaw = smoothed_angle
        self.filter_pub.publish(pose_read)




        # velocity
        vel = Twist()
        vel.linear = msg.velocity
        # TODO: set vel.angular to msg.yaw_dot
        raw_yaw_dot = msg.yaw_dot
        self.data_buffer_rate.append(raw_yaw_dot)
        #smoothed_rate= self.YawSmoother(self.data_buffer_rate)
        smoothed_rate= self.MedianFilter(self.data_buffer_rate)
        ang_vel = smoothed_rate
        vel.angular.x = 0
        vel.angular.y = 0
        vel.angular.z = ang_vel

        # acceleration
        acc = Twist()
        acc.linear = msg.acceleration

        traj_point = MultiDOFJointTrajectoryPoint()
        traj_point.transforms.append(pose)
        traj_point.velocities.append(vel)
        traj_point.accelerations.append(acc)

        traj_msg = MultiDOFJointTrajectory()

        traj_msg.header = msg.header
        traj_msg.points.append(traj_point)
        self.traj_pub.publish(traj_msg)

if __name__ == '__main__':
    obj = MessageConverter()
