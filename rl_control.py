#!/usr/bin/env python

import math
import rospy
import numpy
import tf
import tf2_ros
from geometry_msgs.msg import Transform
import time
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelState
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
from sensor_msgs.msg import PointCloud

def lidar_converter(data):
	a = []
	a.append(data.x)
	a.append(data.y)
	a.append(data.z)
	return a

class State_feedback(object):
	def __init__(self):
		self.command = rospy.Publisher("/command/pose", PoseStamped, queue_size=10)
		self.command_pose = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=10)
		rospy.Subscriber("/ground_truth/state", Odometry, self.position_cb)
		rospy.Subscriber("/raw_imu", Imu, self.imu_cb)
		rospy.Subscriber("/slam_cloud", PointCloud, self.lidar_cb)
		rospy.Subscriber("/scan", LaserScan, self.laser)
		self.command_vel = rospy.Publisher("/cmd_vel",Twist, queue_size=10)
		
		
	def position_cb(self,callback):
		'''
		# define position		
		self.true_position_x = callback.pose.pose.position.x
		self.true_position_y = callback.pose.pose.position.y
		self.true_position_z = callback.pose.pose.position.z

		# define orientation
		self.true_orientation_x = callback.pose.pose.orientation.x	
		self.true_orientation_y = callback.pose.pose.orientation.y
		self.true_orientation_z = callback.pose.pose.orientation.z
		self.true_orientation_w = callback.pose.pose.orientation.w					
		
		
		#define velocity
		self.true_velocity = callback.twist
		'''
		self.pos = (callback.pose.pose.position.x,
		          callback.pose.pose.position.y,
		          callback.pose.pose.position.z,
		          callback.pose.pose.orientation.x,
		          callback.pose.pose.orientation.y,
		          callback.pose.pose.orientation.z,
		          callback.pose.pose.orientation.w)

		self.vel = (callback.twist.twist.linear.x,
		          callback.twist.twist.linear.y,
		          callback.twist.twist.linear.z,
		          callback.twist.twist.angular.x,
		          callback.twist.twist.angular.y,
		          callback.twist.twist.angular.z)


	def set_position(self,Pose = [0,0,0,0,0,0,0]):		
		self.p = ModelState()
		self.p.model_name = "quadrotor"
		self.p.pose.position.x = Pose[0]
		self.p.pose.position.y = Pose[1]
		self.p.pose.position.z = Pose[2]
		self.p.pose.orientation.x = Pose[3]
		self.p.pose.orientation.y = Pose[4]
		self.p.pose.orientation.z = Pose[5]
		self.p.pose.orientation.w = Pose[6]

		Start = time.clock()
		while True:
			self.command_pose.publish(self.p)
			if time.clock() - Start >=1:
				break
		print(1)	
	
	def give_vel(self,vel = [0.5,0.5,0.5,0,0,0],Time = 0.1):
		
		msg = Twist()
		
		msg.linear.x = vel[0]
		msg.linear.y = vel[1]
		msg.linear.z = vel[2]
		msg.angular.x = vel[3]
		msg.angular.y = vel[4]
		msg.angular.z = vel[5]
		T = time.clock()
		while True : 

			self.command_vel.publish(msg)
			if time.clock() - T > Time:
				break
	
			
	def imu_cb(self,callback):
		self.i_ori = (callback.orientation.x,
		              callback.orientation.y,
		              callback.orientation.z,
		              callback.orientation.w)
		              
		self.i_vel_ang = (callback.angular_velocity.x,
		                  callback.angular_velocity.y,
		                  callback.angular_velocity.z)
		                  
		self.i_acc_lin = (callback.linear_acceleration.x,
		                  callback.linear_acceleration.y,
		                  callback.linear_acceleration.z)



	def lidar_cb(self,callback):
		self.lidar_points = list(map(lidar_converter,callback.points))	


	def laser(self, callback):
		self.laser_ranges = callback.ranges
		self.obs_max = max(callback.ranges)
		self.obs_min = min(callback.ranges)

if __name__ == '__main__':
	rospy.init_node('rl_msgs', anonymous=True)
	msg_cb = State_feedback()

	rospy.spin()

	
			
