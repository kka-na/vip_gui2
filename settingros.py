#! /usr/bin/env python
import sys
import copy
import numpy as np
import cv2 
from cv_bridge import CvBridge, CvBridgeError
import math
from time import sleep
from datetime import datetime
from pytz import timezone

import signal
import os
import subprocess, shlex, psutil
import csv 

import rospy
import rospkg
import roslib
import rosbag
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

# from pr2_controllers_msgs.msg import PointHeadActionGoal
from control_msgs.msg import PointHeadAction, PointHeadGoal

#unique
from darknet_ros_msgs.msg import BoundingBoxes #darknet_ros

from sensor_msgs.msg import Image,CompressedImage
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu #drone-imu
from sensor_msgs.msg import NavSatFix #global-position
from sensor_msgs.msg import BatteryState #battery 
from std_msgs.msg import String
from geometry_msgs.msg import Point

dialog_class = uic.loadUiType("dialogwidget.ui")[0]


class TopicDialog(QDialog, dialog_class) :
	def __init__(self) :
		super(TopicDialog, self).__init__()
		self.setupUi(self)
		self.buttonBox.accepted.connect(self.push_ok)

	def save_topics_csv(self, path) :
		file, ok = QInputDialog.getText(self, 'File name', 'Enter topic lists file\'s name')
		self.will_save = " "
		if ok :
			self.will_save = str(path) +"/"+ str(file)+".csv"
			print(self.will_save)

	def push_ok(self) :
		#~erp 
		erp_pos_n = str(self.erp_pos_n.toPlainText())
		erp_pos_t = str(self.erp_pos_t.currentText())
		erp_vel_n = str(self.erp_vel_n.toPlainText())
		erp_vel_t = str(self.erp_vel_t.currentText())
		erp_yaw_n = str(self.erp_yaw_n.toPlainText())
		erp_yaw_t = str(self.erp_yaw_t.currentText())
		erp_dist_n = str(self.erp_dist_n.toPlainText())
		erp_dist_t = str(self.erp_dist_t.currentText())
		erp_obs_n = str(self.erp_obs_n.toPlainText())
		erp_obs_t = str(self.erp_obs_t.currentText())
		erp_detect_n = str(self.erp_detect_n.toPlainText())
		erp_detect_t = str(self.erp_detect_t.currentText())
		erp_cam01_n = str(self.erp_cam01_n.toPlainText())
		erp_cam01_t = str(self.erp_cam01_t.currentText())
		erp_cam02_n = str(self.erp_cam02_n.toPlainText())
		erp_cam02_t = str(self.erp_cam02_t.currentText())
		#~drone
		dro_pos_n = str(self.dro_pos_n.toPlainText())
		dro_pos_t = str(self.dro_pos_t.currentText())
		dro_ypr_n = str(self.dro_ypr_n.toPlainText())
		dro_ypr_t = str(self.dro_ypr_t.currentText())
		dro_alt_n = str(self.dro_alt_n.toPlainText())
		dro_alt_t = str(self.dro_alt_t.currentText())
		dro_bat_n = str(self.dro_bat_n.toPlainText())
		dro_bat_t = str(self.dro_bat_t.currentText())
		dro_cam01_n = str(self.dro_cam01_n.toPlainText())
		dro_cam01_t = str(self.dro_cam01_t.currentText())

		#~goal
		goal_pos_n = str(self.goal_pos_n.toPlainText())
		goal_pos_t = str(self.goal_pos_t.currentText())
		goal_head_n = str(self.goal_head_n.toPlainText())
		goal_head_t = str(self.goal_head_t.currentText())
		goal_node_n = str(self.goal_node_n.toPlainText())
		goal_node_t = str(self.goal_node_t.currentText())

		f = open(self.will_save, 'w')
		wr = csv.writer(f)
		wr.writerow(['erp_pos', erp_pos_n, erp_pos_t])
		wr.writerow(['erp_vel', erp_vel_n, erp_vel_t])
		wr.writerow(['erp_yaw', erp_yaw_n, erp_yaw_t])
		wr.writerow(['erp_dist', erp_dist_n, erp_dist_t])
		wr.writerow(['erp_obs', erp_obs_n, erp_obs_t])
		wr.writerow(['erp_detect', erp_detect_n, erp_detect_t])
		wr.writerow(['erp_cam01', erp_cam01_n, erp_cam01_t])
		wr.writerow(['erp_cam02', erp_cam02_n, erp_cam02_t])
		wr.writerow(['dro_pos', dro_pos_n, dro_pos_t])
		wr.writerow(['dro_ypr', dro_ypr_n, dro_ypr_t])
		wr.writerow(['dro_alt', dro_alt_n, dro_alt_t])
		wr.writerow(['dro_bat', dro_bat_n, dro_bat_t])
		wr.writerow(['dro_cam01', dro_cam01_n, dro_cam01_t])
		wr.writerow(['goal_pos', goal_pos_n, goal_pos_t])
		wr.writerow(['goal_head', goal_head_n, goal_head_t])
		wr.writerow(['goal_node', goal_node_n, goal_node_t])

		mbox = QMessageBox()
		mbox.setStyleSheet("background-color: rgb(40, 42, 54);\ncolor:rgb(248,248,242);")
		mbox.setWindowTitle("Topic List Create")
		mbox.setText("A new topic list csv file was created     ")
		mbox.exec_()