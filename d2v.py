#! /usr/bin/env python
import sys
import copy
import tf
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

#Get moving!
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped, PointStamped
from actionlib_msgs.msg import GoalID
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

import settingros
import controlros
import imugl

#call ui file
form_class = uic.loadUiType("d2v.ui")[0]

playbar_start = 0
playbar_temp = 0
playbar_end = 0
play_btn_clicked = False
pause_btn_clicked = False

save_as_dataset = False
img_path = ""
car1_cnt = 0
car2_cnt = 0
drone_cnt = 0



class WindowClass(QMainWindow, form_class) :
	def __init__(self) : 
		super(WindowClass, self).__init__()
		self.setupUi(self)
		self.rosbag_proc = subprocess
		self.tdia = settingros.TopicDialog()
		self.ctrlros = controlros.ControlRos()
		self.loadImageFromFile() 
		self.connect_signals()

	def connect_signals(self) :
		self.actionReady.triggered.connect(self.actionready_triggered) #ready state
		self.actionReady_2.triggered.connect(self.actionready_2_triggered)
		self.actionRecord.triggered.connect(self.actionrecord_triggered)
		self.actionSave_as_Dataset.triggered.connect(self.actionSave_as_Dataset_triggered)
		self.actionStart.triggered.connect(self.actionstart_triggered) #start state
		self.actionStart_2.triggered.connect(self.actionstart_2_triggered) #start state
		self.actionStop.triggered.connect(self.actionstop_triggered) #stop state

		self.ctrlros.send_rviz_goal_pos.connect(self.update_rviz_goal_pos)
		self.ctrlros.send_rviz_goal_head.connect(self.update_rviz_goal_head)
		self.ctrlros.send_rviz_goal_node.connect(self.update_rviz_goal_node)
		self.ctrlros.send_car_pos.connect(self.update_car_pos)
		self.ctrlros.send_car_yaw.connect(self.update_car_yaw)
		self.ctrlros.send_car_vel.connect(self.update_car_vel)
		self.ctrlros.send_car_dist.connect(self.update_car_dist)
		self.ctrlros.send_car_obs.connect(self.update_car_obs)
		self.ctrlros.send_car_cam_1.connect(self.update_car_cam_1)
		self.ctrlros.send_car_cam_2.connect(self.update_car_cam_2)
		self.ctrlros.send_c_bbox_num.connect(self.update_c_bbox_num)
		self.ctrlros.send_drone_pos.connect(self.update_drone_pos)
		self.ctrlros.send_drone_ypr.connect(self.update_drone_ypr)
		self.ctrlros.send_drone_alt.connect(self.update_drone_alt)
		self.ctrlros.send_drone_batt.connect(self.update_drone_batt)
		self.ctrlros.send_drone_cam.connect(self.update_drone_cam)
		self.ctrlros.send_label_18.connect(self.update_label_18)


	def actionready_triggered(self):
		print("Action Ready Triggered")
		self.topic_list_create()
		self.initialize_all() #initialize rviz and ros nodes

	def actionrecord_triggered(self):
		print("Action Record Triggered")
		self.set_record()

	def actionstart_triggered(self):
		print("Action Start Triggered")
		self.get_topic_list()
		#Open GL
		""" 
		self.carGL = imugl.ImuGL("Car.obj",1, self.ctrlros)
		self.droGL = imugl.ImuGL("Drone.obj",2, self.ctrlros)
		self.car_gl.addWidget(self.carGL)
		self.dro_gl.addWidget(self.droGL)
		self.carGL.show()
		self.droGL.show()
		"""

	def actionstop_triggered(self) :
		print("Action Stop Triggered")
		self.monitoring_stop()

	def loadImageFromFile(self) :
		self.qPixmapCar = QPixmap()
		self.qPixmapCar.load("./icon/car.png")
		self.qPixmapCar = self.qPixmapCar.scaledToWidth(50)
		self.car_png.setPixmap(self.qPixmapCar)
		self.qPixmapDrone = QPixmap()
		self.qPixmapDrone.load("./icon/drone.png")
		self.qPixmapDrone = self.qPixmapDrone.scaledToWidth(50)
		self.drone_png.setPixmap(self.qPixmapDrone)

		self.qPixmapDynamic = QPixmap()
		self.qPixmapDynamic.load("./icon/person.png")
		self.qPixmapDynamic = self.qPixmapDynamic.scaledToWidth(50)
		self.c_static_png.setPixmap(self.qPixmapDynamic)
		#self.d_static_png.setPixmap(self.qPixmapDynamic)
		self.qPixmapStatic = QPixmap()
		self.qPixmapStatic.load("./icon/cone.png")
		self.qPixmapStatic = self.qPixmapStatic.scaledToWidth(50)
		self.qPixmapStatic_2 = QPixmap()
		self.qPixmapStatic_2.load("./icon/banana.png")
		self.qPixmapStatic_2 = self.qPixmapStatic_2.scaledToWidth(50)
		self.c_dynamic_png.setPixmap(self.qPixmapStatic)
		#self.d_dynamic_png.setPixmap(self.qPixmapStatic_2)
		self.label_18.hide()


	def topic_list_create(self) :
		mbox = QMessageBox()
		mbox.setStyleSheet("background-color: rgb(40, 42, 54);\ncolor:rgb(248,248,242);")
		mbox.setWindowTitle("ROS Topic List Create")
		mbox.setText("Create a topic to use.     \nCreate or Not?     ")
		mbox.setStandardButtons(QMessageBox.Ok | QMessageBox.Cancel)
		buttonC = mbox.button(QMessageBox.Ok)
		buttonC.setText('Create')
		returnv = mbox.exec_()
		if returnv == QMessageBox.Ok :
			path =  QFileDialog.getExistingDirectory(None, 'Select folder to save topic setting information csv',QDir.currentPath(), QFileDialog.ShowDirsOnly)
			self.tdia.save_topics_csv(path)
			self.tdia.exec_()

		


	def initialize_all(self) :
		#self.publish_nodes() #publish send NAV GOAL message from rviz 
		#setting rviz visulizer
		self.rviz_frame = rviz.VisualizationFrame()
		self.rviz_frame.setSplashPath("")
		self.rviz_frame.initialize()
		#setting rviz frame reader
		reader = rviz.YamlConfigReader()
		config = rviz.Config() 
		reader.readFile(config, "./default.rviz") #this is saved rviz frame for d2v project
		#setting rviz tools 
		self.rviz_frame.load(config)
		self.rviz_frame.setMenuBar(None)
		self.rviz_frame.setStatusBar(None)
		#setting rviz manager
		self.manager = self.rviz_frame.getManager()
		self.grid_display = self.manager.getRootDisplayGroup().getDisplayAt( 0 )
		self.rviz_layout.addWidget(self.rviz_frame)

		mbox = QMessageBox()
		mbox.setStyleSheet("background-color: rgb(40, 42, 54);\ncolor:rgb(248,248,242);")
		mbox.setWindowTitle("Ready State")
		mbox.setText("All Process were Initialized     ")
		mbox.exec_()

	def set_record(self) :
		mbox = QMessageBox()
		mbox.setStyleSheet("background-color: rgb(40, 42, 54);\ncolor:rgb(248,248,242);")
		mbox.setWindowTitle("Record State")
		mbox.setText("All information will be recorded.     \nWould you like to record it?     ")
		mbox.setStandardButtons(QMessageBox.Ok | QMessageBox.Cancel)
		returnv = mbox.exec_()
		path = ""
		if returnv == QMessageBox.Ok :
			path =  QFileDialog.getExistingDirectory(None, 'Select folder to save recorded information',QDir.currentPath(), QFileDialog.ShowDirsOnly)
			now_utc = datetime.now(timezone('UTC'))
			now_kst = now_utc.astimezone(timezone('Asia/Seoul'))
			dirname = "{}/D2V_{}_dataset".format(str(path),str(now_kst.strftime('%Y%m%d%H%M%S')))
			os.mkdir(dirname)
			command ="rosbag record -o "+dirname+"/ -a"
			command = shlex.split(command)
			self.rosbag_proc = subprocess.Popen(command)

	def publish_nodes(self) :
		#for sending nav goals
		nav_topic = rospy.get_param("remote_nav/nav_topic", "/move_base_simple/goal")
		self.nav_pub = rospy.Publisher(nav_topic, PoseStamped, queue_size=10)

	def get_topic_list(self) :
		mbox = QMessageBox()
		mbox.setStyleSheet("background-color: rgb(40, 42, 54);\ncolor:rgb(248,248,242);")
		mbox.setWindowTitle("ROS Topic Setting")
		mbox.setText("Import topic list files to Use     ")
		mbox.setStandardButtons(QMessageBox.Ok | QMessageBox.Cancel)
		buttonC = mbox.button(QMessageBox.Ok)
		buttonC.setText('Import')
		returnv = mbox.exec_()
		if returnv == QMessageBox.Ok :
			file =  QFileDialog.getOpenFileName(None, 'Select topic setting information csv files to setting ROS','./')[0]
			self.get_topics_csv(file)
			
	def get_topics_csv(self, file) :
		f = open(str(file), 'r')
		rdr = csv.reader(f)
		mbox = QMessageBox()
		mbox.setStyleSheet("background-color: rgb(40, 42, 54);\ncolor:rgb(248,248,242);")
		mbox.setWindowTitle("Start State")
		mbox.setText("Monitoring Start!     ")
		mbox.exec_()
		for line in rdr :
			self.ctrlros.initialize_ros(line[0], line[1], line[2])

	def moveNav(self) :
		goal = PoseStamped()
		goal.header.frame_id = "/start"
		goal.pose.orientation.z = 1.0
		goal.pose.orientation.w = 0.0
		self._send_nav_goal(goal)

	def _send_nav_goal(self, pose) : 
		self.nav_pub.publish(pose)

	def testt(self):
		pos_txt = " X : 6.3712 Y : 53.9518"
		self.goal_pos_label.setText(pos_txt)
		yaw_txt = "0.0"
		self.goal_yaw_label.setText(yaw_txt)
		self.goal_node_label.setText("21")

	@pyqtSlot(str)
	def update_rviz_goal_pos(self, string) :
		self.goal_pos_label.setText(string)

	@pyqtSlot(str)
	def update_rviz_goal_head(self, string) :
		self.goal_yaw_label.setText(string)

	@pyqtSlot(str)
	def update_rviz_goal_node(self, string) :
		self.goal_node_label.setText(string)

	@pyqtSlot(str)
	def update_car_pos(self, string) :
		self.car_pos_label.setText(string)

	@pyqtSlot(str)
	def update_car_yaw(self, string) :
		self.car_yaw_label.setText(string)

	@pyqtSlot(str)
	def update_car_vel(self, string) :
		self.car_vel_label.setText(string)

	@pyqtSlot(str)
	def update_car_dist(self, string) :
		self.car_distance_label.setText(string)

	@pyqtSlot(str)
	def update_car_obs(self, string) :
		self.car_obs_label.setText(string)

	@pyqtSlot(object)
	def update_car_cam_1(self, rgbImage) :
		global save_as_dataset,img_path, car1_cnt
		if save_as_dataset:
			ts_path = "{}/car1/timestamp.txt".format(str(img_path))
			ts = int(str(ros_data.header.stamp))
			ts /= 1000000000
			#ts = datetime.utcfromtimestamp(ts).strptime('%Y-%m-%d %H:%M:%S.%f')
			ts = str(datetime.fromtimestamp(ts, timezone('Asia/Seoul')))
			#ts = datetime.strptime(ts, '%Y-%m-%d %H:%M:%S.%f')
			#ts = str(ts.astimezone(timezone('Asia/Seoul')))
			f = open(ts_path, 'a')
			f.write(ts+'\n')
			f.close()
			cv2.imwrite("{}/car1/{}.jpg".format(str(img_path),str(car1_cnt)), rgbImage)
			car1_cnt+=1

		h,w,ch = rgbImage.shape
		bpl = ch * w
		converToQtFormat = QImage(rgbImage.data, w, h, bpl, QImage.Format_RGB888)
		qimage = converToQtFormat.scaled(400,300, Qt.KeepAspectRatio)
		self.car_cam_label_1.setPixmap(QPixmap.fromImage(qimage))
		self.car_cam_label_1.show()
		QApplication.processEvents()

	@pyqtSlot(object)
	def update_car_cam_2(self, rgbImage) :
		global save_as_dataset, img_path, car2_cnt
		if save_as_dataset:
			ts_path = "{}/car2/timestamp.txt".format(str(img_path))
			ts = int(str(data.header.stamp))
			ts /= 1000000000
			#ts = datetime.utcfromtimestamp(ts).strptime('%Y-%m-%d %H:%M:%S.%f')
			ts = str(datetime.fromtimestamp(ts, timezone('Asia/Seoul')))
			#ts = datetime.strptime(ts, '%Y-%m-%d %H:%M:%S.%f')
			#ts = str(ts.astimezone(timezone('Asia/Seoul')))
			f = open(ts_path, 'a')
			f.write(ts+'\n')
			f.close()
			cv2.imwrite("{}/car2/{}.jpg".format(str(img_path),str(car2_cnt)), rgbImage)
			car2_cnt+=1
		h,w,ch = rgbImage.shape
		bpl = ch * w
		converToQtFormat = QImage(rgbImage.data, w, h, bpl, QImage.Format_RGB888)
		qimage = converToQtFormat.scaled(480,300, Qt.KeepAspectRatio)
		self.car_cam_label_2.setPixmap(QPixmap.fromImage(qimage))
		self.car_cam_label_2.show()
		QApplication.processEvents()

	@pyqtSlot(str,str)
	def update_c_bbox_num(self, strings, stringd) :
		self.c_static_num.setText(strings)
		self.c_dynamic_num.setText(stringd)

	@pyqtSlot(str)
	def update_drone_pos(self, string) :
		self.drone_pos_label.setText(string)

	@pyqtSlot(str)
	def update_drone_ypr(self, string) :
		self.drone_ypr_label.setText(string)

	@pyqtSlot(str)
	def update_drone_alt(self, string) :
		self.drone_alt_label.setText(string)

	@pyqtSlot(str)
	def update_drone_batt(self, string) :
		self.drone_batt_label.setText(string)

	@pyqtSlot(object)
	def update_drone_cam(self, rgbImage) :
		global img_path, save_as_dataset, drone_cnt
		if save_as_dataset:
			ts_path = "{}/drone/timestamp.txt".format(str(img_path))
			ts = int(str(data.header.stamp))
			ts /= 1000000000
			#ts = datetime.utcfromtimestamp(ts).strptime('%Y-%m-%d %H:%M:%S.%f')
			ts = str(datetime.fromtimestamp(ts, timezone('Asia/Seoul')))
			#ts = datetime.strptime(ts, '%Y-%m-%d %H:%M:%S.%f')
			#ts = str(ts.astimezone(timezone('Asia/Seoul')))
			f = open(ts_path, 'a')
			f.write(ts+'\n')
			f.close()
			cv2.imwrite("{}/drone/{}.jpg".format(str(img_path),str(drone_cnt)), rgbImage)
			drone_cnt+=1
		h,w,ch = rgbImage.shape
		bpl = ch * w
		converToQtFormat = QImage(rgbImage.data, w, h, bpl, QImage.Format_RGB888)
		qimage = converToQtFormat.scaled(480,300, Qt.KeepAspectRatio)
		self.drone_cam_label.setPixmap(QPixmap.fromImage(qimage))
		self.drone_cam_label.show()
		QApplication.processEvents()

	@pyqtSlot(str)
	def update_label_18(self, string) :
		self.label_18.setText(string)
		self.label_18.show()
		sleep(0.05)
		self.label_18.hide()


	def monitoring_stop(self) :
		for proc in psutil.process_iter() :
			if "record" in proc.name() : 
				proc.send_signal(subprocess.signal.SIGINT)
		self.rosbag_proc.send_signal(subprocess.signal.SIGINT)
		rospy.on_shutdown(self.myhook) #shut down the nodes
		mbox = QMessageBox()
		mbox.setStyleSheet("background-color: rgb(40, 42, 54);\ncolor:rgb(248,248,242);")
		mbox.setWindowTitle("Stop State")
		mbox.setText("All Process were Ended     ")
		mbox.exec_()

	#shut down the ros nodes
	def myhook(self):
		print("Shut down all processes cleary!")

# ~ - ~ - ~ - ~ - ~ - ~ - ~ - ~ - ~ - End of Monitoring Processs - ~ - ~ - ~ - ~ - ~ - ~ - ~ - ~ #

	def actionready_2_triggered(self):
		print("Action Ready_2 Triggered")
		self.PlayBarThread = PlayBarThread()
		self.PlayBarThread.change_playbar_value.connect(self.change_playbar_state)
		self.setting_playback()
		self.initialize_all()

	def setting_playback(self) :
		self.bagfile =  QFileDialog.getOpenFileName(self, 'Select .bag files to playback', './')[0]
		self.bag = rosbag.Bag(self.bagfile)
		b_start = self.bag.get_start_time()
		b_end = self.bag.get_end_time()
		self.b_total_sec = int(b_end-b_start)

		self.slider.setRange(0, self.b_total_sec)
		self.slider.valueChanged.connect(self.setBagPosition)
		self.play_btn.clicked.connect(self.push_play_btn)
		self.pause_btn.clicked.connect(self.push_pause_btn)

	def actionSave_as_Dataset_triggered(self) :
		global save_as_dataset, img_path
		global car1_cnt, car2_cnt, drone_cnt
	#	print("Action Save as Dataset Triggered")
		mbox = QMessageBox()
		mbox.setStyleSheet("background-color: rgb(40, 42, 54);\ncolor:rgb(248,248,242);")
		mbox.setWindowTitle("Dataset Saving State")
		mbox.setText("Saving .bag data as Dataset.     \nWould you like to save it?     ")
		mbox.setStandardButtons(QMessageBox.Ok | QMessageBox.Cancel)
		returnv = mbox.exec_()
		path = ""
		if returnv == QMessageBox.Ok :
			path =  QFileDialog.getExistingDirectory(None, 'Select folder to save Dataset',QDir.currentPath(), QFileDialog.ShowDirsOnly)
			img_path = "{}/images".format(str(path))
			os.mkdir(img_path)
			img_path_list = ["{}/images/drone".format(str(path)), "{}/images/car1".format(str(path)), "{}/images/car2".format(str(path))] 
			for img_dir in img_path_list : 
				os.mkdir(img_dir)

			car1_cnt = 0
			car2_cnt = 0
			drone_cnt = 0 
			save_as_dataset = True
			

	def actionstart_2_triggered(self) :
		global playbar_start, playbar_end
		playbar_start = 0
		playbar_end = self.b_total_sec
		self.get_topic_list()
		command ="rosbag play "+self.bagfile
		command = shlex.split(command)
		self.rosbag_proc = subprocess.Popen(command)
		self.PlayBarThread.start()


	def setBagPosition(self) :
		global playbar_start
		playbar_start = self.slider.value()
		self.ts_txt.setText(str(self.slider.value()))

	def push_play_btn(self) :
		global playbar_start
		global play_btn_clicked
		print("play btn pushed")
		play_btn_clicked = True
		command ="rosbag play -s"+str(playbar_start)+" "+self.bagfile
		command = shlex.split(command)
		self.rosbag_proc = subprocess.Popen(command)
		self.PlayBarThread.start()

	def push_pause_btn(self) :
		global pause_btn_clicked
		print("pause btn pushed")
		pause_btn_clicked = True
		for proc in psutil.process_iter() :
			if "play" in proc.name() :
				proc.send_signal(subprocess.signal.SIGINT)
		self.rosbag_proc.send_signal(subprocess.signal.SIGINT)
		self.PlayBarThread.stop()


	@pyqtSlot(int)
	def change_playbar_state(self, value) :
		self.slider.setValue(value)
		self.ts_txt.setText(str(value))

class PlayBarThread(QThread) :
	change_playbar_value = pyqtSignal(int)
	def __init__(self) :
		super(PlayBarThread, self).__init__()
		self.running = True

	def run(self) :
		global playbar_start, playbar_end, playbar_temp
		global play_btn_clicked, pause_btn_clicked
		if pause_btn_clicked :
			b_cnt = playbar_temp
			pause_btn_clicked = False
		else :
			b_cnt = playbar_start
		if play_btn_clicked :
			self.running = True

		while b_cnt in range(playbar_start, playbar_end+1) :
			if self.running == False : break
			if(b_cnt > playbar_end) :
				self.stop()
			self.change_playbar_value.emit(b_cnt)
			QApplication.processEvents()
			sleep(1)
			playbar_temp = b_cnt
			b_cnt += 1

	def stop(self) :
		self.running = False
		self.quit()
		self.wait(5000)


if __name__ == "__main__" :
	signal.signal(signal.SIGINT, signal.SIG_DFL)
	app = QApplication(sys.argv)
	myWindow = WindowClass()
	myWindow.showMaximized()
	app.exec_()