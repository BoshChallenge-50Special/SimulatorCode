#!/usr/bin/python

"""
These are the libraries you need to import in your project in order to
be able to communicate with the Gazebo simulator
"""

# Necessari per ROS
import rospy
from std_msgs.msg import String # Ne esistono di molti altri tipi di messaggi che si possono usare
import json
import sys
sys.settrace

import os
import traceback

#sys.path.insert(0, '/home/{YOUR_USERNAME}/Documents/BFMC_Simulator/startup_workspace/src/startup_package/src')
sys.path.insert(0, '../../BFMC_Simulator/startup_workspace/src/startup_package/src/')
sys.path.insert(0, './src/startup_package/src/')

from bfmclib.gps_s import Gps
from bfmclib.bno055_s import BNO055
from bfmclib.camera_s import CameraHandler
from bfmclib.controller_p import Controller
from bfmclib.trafficlight_s import TLColor, TLLabel, TrafficLight

import cv2 as cv
from time import sleep
import rospy

from SimulatorCode.templates import Consumer



class Processer(Consumer):

	def __init__(self):#, cam, car, sem, gps, bno):
		super(Processer, self).__init__()
		#self.cam=cam
		#self.car=car
		#self.sem=sem
		#self.gps=gps
		#self.bno=bno
		rospy.init_node("PROCESSOR", anonymous=True)
		self.car=Controller()
		#print(os.getcwd())
		# Qua si possono salvare gli stati dei vari processi
		self.HorizontalState = "Safe"


	def start(self):
		# Far eseguire il file come se fosse eseguito da terminale
		os.system("rosrun startup_package horizontal_line.py &")
		os.system("rosrun startup_package ParticleFilter.py &")
		os.system("rosrun startup_package PidControl.py &")

		self.subscribe("HorizontalLine", "horizontal_line")
		self.subscribe("StreetLane", "street_lines")
		self.subscribe("PidControlValues", "velocity_steer")
		#rospy.Subscriber("HorizontalLine", String, self.HorizontalCheck)


		while not rospy.is_shutdown():
			#cv.imshow("Processer", cam.getImage())
			#cv.waitKey(1)
			speed = 0.18
			steering = 0
			self.car.drive(speed, steering)
			sleep(0.1)
			#print("Sending move with speed "+ str(speed) + " , steering " + str(steering))
			if("street_lines" in self.data):
				lines = json.loads(self.data["street_lines"])
				#print(lines[0][0])
			if("velocity_steer" in self.data):
				velocity_steer = json.loads(self.data["velocity_steer"])
				steering = -velocity_steer["steer"]
			self.car.drive(speed, steering)
			sleep(0.1)


		rospy.spin()

	#def HorizontalCheck(self, data):
	#	print("HorizontalCheck")
	#	if(data.data != self.HorizontalState):
	#		self.HorizontalState = data.data
	#		rospy.loginfo('Horizontal: %s', data.data)

if __name__ == '__main__':
	try:
		print("Loaded processer")

		processer=Processer()
		processer.start()

		#while not rospy.is_shutdown():
		#	img = cam.getImage()
		#	horizonLine.check_horizontal(img)

		#	horizonLine.rate.sleep()
	except Exception as e:
		print("Error in processer.py")
		print(e)
