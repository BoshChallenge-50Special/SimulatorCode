#!/usr/bin/python

"""
These are the libraries you need to import in your project in order to
be able to communicate with the Gazebo simulator
"""

# Necessari per ROS
import rospy
from std_msgs.msg import String # Ne esistono di molti altri tipi di messaggi che si possono usare

import sys
sys.settrace

import os 

#sys.path.insert(0, '/home/{YOUR_USERNAME}/Documents/BFMC_Simulator/startup_workspace/src/startup_package/src')
sys.path.insert(0, '/home/marco/Documents/BFMC_Simulator/startup_workspace/src/startup_package/src/')

from bfmclib.gps_s import Gps
from bfmclib.bno055_s import BNO055
from bfmclib.camera_s import CameraHandler
from bfmclib.controller_p import Controller
from bfmclib.trafficlight_s import TLColor, TLLabel, TrafficLight

import rospy


class Processer(object):

	def __init__(self, cam, car, sem, gps, bno):
		self.cam=cam
		self.car=car
		self.sem=sem
		self.gps=gps
		self.bno=bno

		# Qua si possono salvare gli stati dei vari processi
		self.HorizontalState = "Safe"


	def start(self):

		# Far eseguire il file come se fosse eseguito da terminale
		os.system("python ~/Documents/BoschChallenge/SimulatorCode/LineTracking/horizontal_line.py &")

		rospy.Subscriber("HorizontalLine", String, self.HorizontalCheck)


		speed = 0.1
		steering = 0
		self.car.drive(speed, steering)
		print("Sending move with speed "+ str(speed) + " , steering " + str(steering))


		rospy.spin()

	def HorizontalCheck(self, data):
		if(data.data != self.HorizontalState):
			self.HorizontalState = data.data
			rospy.loginfo('Horizontal: %s', data.data)