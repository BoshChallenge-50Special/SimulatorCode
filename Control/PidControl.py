#!/usr/bin/env python2

#import cv2 as cv
#import numpy as np
#import math

import rospy
from std_msgs.msg import String
from time import sleep
import sys
import json
#sys.path.insert(0, '/home/{YOUR_USERNAME}/Documents/BFMC_Simulator/startup_workspace/src/startup_package/src')
sys.path.insert(0, './src/startup_package/src/')
#sys.path.insert(0, '../../src')

from SimulatorCode.templates import Producer, Consumer
from SimulatorCode.LineTracking.utils import Utils
import PID

class PidControl(Producer, Consumer):

	def __init__(self, stop_criteria):
		super(PidControl, self).__init__("PidController")

		self.set_publisher("PidControlValues")
		self.stop_criteria=stop_criteria
		P = 0.08
		I = 0.002
		D = 0.002
		self.distance_from_base = 0.6       #it's a percentage
		self.pid = PID.PID(P, I, D)
		self.size = 640 ###### TODO --> PUT IT IN A CONFIG FILE
		self.targetT = 100
		self.verbose = False

		self.subscribe("StreetLane", "street_lines")

	def get_current_trajectory_point(self, lines):
		dist = []

		if(lines[0] != None):
			dist.append([lines[0][i][0] for i in range(0, len(lines[0]))])
		if(lines[1] != None):
			dist.append([lines[1][i][0]-self.size/2 for i in range(0, len(lines[1]))])

		if(len(dist)==0):
			return None
		elif(len(dist)==1):
			index = int(self.distance_from_base * len(dist[0]))
			return dist[0][index]
		else:
			too_near=False
			dist_avg = [(dist[0][i] + dist[1][i])/2 for i in range(0, len(dist[0])) ]
			index = int(self.distance_from_base * len(dist[0]))
			return dist_avg[index]

	def get_steer(self, point_objective):
		self.pid.SetPoint = self.targetT
		self.pid.setSampleTime(0.1)
		self.pid.update(point_objective)#central_line[17])#/img.shape[1])
		targetPwm = self.pid.output

		return targetPwm

	def run(self):
		while not self.stop_criteria():
			lines=[None, None]
			if("street_lines" in self.data):
				lines = json.loads(self.data["street_lines"])
			actual_trajectory=self.get_current_trajectory_point(lines)
			if(actual_trajectory==None):
				actual_trajectory=self.targetT
			steer=self.get_steer(actual_trajectory)
			command = json.dumps({"velocity" : 0, "steer" : steer})
			self.pub.publish(command)
			sleep(0.1)

			if(self.verbose):
				steer_dir=""
				if(-steer > 0):
					steer_dir="right"
				else:
					steer_dir="left"
				print("COMMAND : VELOCITY = 0.17 | STEER = "+steer_dir+ " --> " +str(steer) + " ERROR : "+str(self.targetT-actual_trajectory))

if __name__ == '__main__':
	try:
		print("Starting PidControl")
		pidControl = PidControl(stop_criteria=rospy.is_shutdown)
		pidControl.run()
		#while not rospy.is_shutdown():
		#	img = cam.getImage()
		#	horizonLine.check_horizontal(img)
		#	sleep(0.1)
		#	#horizonLine.rate.sleep()
	except Exception as e:
		print("Error in PidControl.py")
		print(e)
