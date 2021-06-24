#! /usr/bin/env python
import sys
import copy
import numpy as np
import cv2 
from cv_bridge import CvBridge, CvBridgeError
import math

import signal
import os
import subprocess, shlex, psutil
import csv 
import tf

import rospy
import rospkg
import roslib
#import rosbag
import rviz
import actionlib
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker

from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5 import QtCore
from PyQt5 import uic

from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped, PointStamped, Point
from actionlib_msgs.msg import GoalID
from control_msgs.msg import PointHeadAction, PointHeadGoal
from darknet_ros_msgs.msg import BoundingBoxes #darknet_ros
from sensor_msgs.msg import Image,CompressedImage, Imu, NavSatFix, BatteryState
from nav_msgs.msg import Odometry
from std_msgs.msg import String

dialog_class = uic.loadUiType("dialogwidget.ui")[0]


class ControlRos(QObject) :
	def __init__(self) :
		super(ControlRos, self).__init__()
		rospy.init_node('visualizer', anonymous=True)
		#test
	


	#test
	send_imu = pyqtSignal(float, float, float)
	def imuCallBack(self, data) :
		"""
		#calculate
		quaternion = (data.orientation.x,data.orientation.y, data.orientation.z,data.orientation.w  )
		euler = tf.transformations.euler_from_quaternion(quaternion)	
		PI = 3.141592
		pitch = round(euler[0]*180/PI, 4) #y
		roll = round(euler[1]*180/PI, 4) #x
		yaw = round(euler[2]*180/PI, 4) #z
		"""
		#test
		roll = data.pose.pose.position.z
		pitch = -data.pose.pose.position.z
		yaw = 0.0
		quaternion = (data.pose.pose.position.z,-data.pose.pose.position.z,data.pose.pose.position.z,-data.pose.pose.position.z  )
		euler = tf.transformations.euler_from_quaternion(quaternion)	

		self.send_imu.emit(roll, pitch, yaw)



	send_rviz_goal_pos = pyqtSignal(str)
	def rviz_goal_pos_callback(self, data) :
		pos_txt = "X : {}   Y : {}".format(round(data.x,5), round(data.y,5))
		self.send_rviz_goal_pos.emit(pos_txt)

	send_rviz_goal_head = pyqtSignal(str)
	def rviz_goal_head_callback(self, data) :
		yaw_txt = "{}".format(round(data.z,3))
		self.send_rviz_goal_head.emit(yaw_txt)

	send_rviz_goal_node = pyqtSignal(str)
	def rviz_goal_node_callback(self, data) :
		self.send_rviz_goal_node.emit(str(data.data))
	
	send_car_pos = pyqtSignal(str)
	def c_pos_callback(self, ros_data) :
		pos_txt = "lat : {}   lng : {}".format(round(ros_data.pose.pose.position.y,5), round(ros_data.pose.pose.position.x,5))
		self.send_car_pos.emit(pos_txt)

	send_car_yaw = pyqtSignal(str)
	def c_yaw_callback(self, ros_data) :
		yaw_txt = "{}".format(round(ros_data.twist.twist.angular.z,3))
		self.send_car_yaw.emit(yaw_txt) 

	send_car_vel = pyqtSignal(str)
	def c_vel_callback(self, ros_data) :
		velstr = str(ros_data.pose.pose.position.x)
		self.send_car_vel.emit(velstr+" m/s") #modify dimension

	send_car_dist = pyqtSignal(str)
	def c_dist_callback(self, ros_data) :
		diststr = str(round(ros_data.twist.twist.linear.z,3))
		self.send_car_dist.emit(diststr+" m remain") 

	send_car_obs = pyqtSignal(str)
	def c_obs_callback(self, ros_data) :
		obsstr = str(int(ros_data.pose.pose.position.x))
		self.send_car_obs.emit(obsstr+" detected") 

	send_car_cam_1 = pyqtSignal(object)
	def c_cam_callback(self, data) :
		np_arr = np.fromstring(data.data, np.uint8)
		cv2_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
		rgbImage = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2RGB)
		self.send_car_cam_1.emit(rgbImage)

	send_car_cam_2 = pyqtSignal(object)
	def c_lane_cam_callback(self, data) :
		np_arr = np.fromstring(data.data, np.uint8)
		cv2_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
		rgbImage = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2RGB)
		self.send_car_cam_2.emit(rgbImage)

	send_c_bbox_num = pyqtSignal(str, str)
	def c_bboxes_callback(self, data) :
		s_count = 0
		d_count = 0
		for box in data.bounding_boxes:
			if box.Class == 'PERSON' :
				s_count = s_count+1

			if box.Class == 'CONE' :
				d_count = d_count+1
		self.send_c_bbox_num.emit(str(s_count),str(d_count))

	send_drone_pos = pyqtSignal(str)
	def d_pos_callback(self, data) :
		pos_txt = "lat : {}, lng : {}".format(round(data.latitude,5), round(data.longitude,5))
		self.send_drone_pos.emit(pos_txt)

	send_drone_ypr = pyqtSignal(str)
	def d_imu_callback(self, data) :
		#quaternion = (data.orientation.x,data.orientation.y, data.orientation.z,data.orientation.w  )
		quaternion = (data.linear_acceleration.x,data.linear_acceleration.y, data.linear_acceleration.z,data.orientation.w  )
		euler = tf.transformations.euler_from_quaternion(quaternion)

		PI = 3.141592
		pitch = round(euler[0]*180/PI, 4)
		roll = round(euler[1]*180/PI, 4)
		yaw = round(euler[2]*180/PI, 4)

		ypr_txt = "Y:{}   P:{}  R:{}".format(yaw, pitch, roll)
		self.send_drone_ypr.emit(ypr_txt)
		self.send_imu.emit(roll,pitch,yaw)

	send_drone_alt = pyqtSignal(str)
	def d_alt_callback(self, data) :
		alt_txt = "{} m".format(round(data.pose.position.z,3))
		self.send_drone_alt.emit(alt_txt)

	send_drone_batt = pyqtSignal(str) 
	def d_batt_callback(self, data) :
		bat_txt = "{} v".format(round(data.voltage,3))
		self.send_drone_batt.emit(bat_txt)

	send_drone_cam = pyqtSignal(object)
	def d_cam_callback(self, data) :
		np_arr = np.fromstring(data.data, np.uint8)
		cv2_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
		rgbImage = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2RGB)
		self.send_drone_cam.emit(rgbImage)

	send_label_18 = pyqtSignal(str)
	def warning_callback(self,data) :
		self.send_label_18.emit(data.data)


	def initialize_ros(self, frame_name, topic_name, msg_type) : 
		#self.testt()
		if(msg_type == "Odometry") :
			if(frame_name == "erp_pos") :
				rospy.Subscriber(topic_name, Odometry , self.c_pos_callback)
			if(frame_name == "erp_yaw") :
				rospy.Subscriber(topic_name, Odometry , self.c_yaw_callback)
			if(frame_name == "erp_dist") :
				rospy.Subscriber(topic_name, Odometry , self.c_dist_callback)
			if(frame_name == "obstacle_number") :
				rospy.Subscriber(topic_name, Odometry , self.c_obs_callback)
		if(msg_type == "CompressedImage") :
			if(frame_name == "erp_cam01") :
				rospy.Subscriber(topic_name, CompressedImage , self.c_cam_callback)
			if(frame_name == "erp_cam02") :
				rospy.Subscriber(topic_name, CompressedImage , self.c_lane_cam_callback)
			if(frame_name == "dro_cam01") :
				rospy.Subscriber(topic_name, CompressedImage, self.d_cam_callback)
		if(msg_type == "BoundingBoxes") :
			if(frame_name == "erp_detect") :
				rospy.Subscriber(topic_name, BoundingBoxes , self.c_bboxes_callback)
		if(msg_type == "BatteryState") :
			if(frame_name == "dro_bat") :
				rospy.Subscriber(topic_name, BatteryState , self.d_batt_callback)
		if(msg_type == "NavSatFix") :
			if(frame_name == "dro_pos") :
				rospy.Subscriber(topic_name, NavSatFix, self.d_pos_callback)
		if(msg_type == "Imu") :
			if(frame_name == "dro_ypr") :
				rospy.Subscriber("/snappy_imu", Imu, self.d_imu_callback)
		if(msg_type == "PoseStamped") :
			if(frame_name == "dro_alt") :
				rospy.Subscriber(topic_name, PoseStamped, self.d_alt_callback)
		if(msg_type == "Point") :
			if(frame_name == "goal_pos") :
				rospy.Subscriber(topic_name, Point, self.rviz_goal_pos_callback)
			if(frame_name == "goal_head") :
				rospy.Subscriber(topic_name, Point, self.rviz_goal_head_callback)
		if(msg_type == "String") :
			if(frame_name == "goal_node") :
				rospy.Subscriber(topic_name, String, self.rviz_goal_node_callback)

		rospy.Subscriber('/warning_msg', String, self.warning_callback)
