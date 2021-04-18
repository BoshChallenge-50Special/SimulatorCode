#!/usr/bin/env python2

#import cv2 as cv
#import numpy as np
#import math

import rospy
from std_msgs.msg import String
from time import sleep
import sys
from scipy import interpolate
import json
import math
import Control.CarModel as car_model
#sys.path.insert(0, '/home/{YOUR_USERNAME}/Documents/BFMC_Simulator/startup_workspace/src/startup_package/src')
sys.path.insert(0, './src/startup_package/src/')
#sys.path.insert(0, '../../src')

from SimulatorCode.templates import Producer, Consumer
from Control import PID

class PidControl():#Producer, Consumer):

	def __init__(self, verbose=False):
		#@super(PidControl, self).__init__("PidController")

		#self.set_publisher("PidControlValues")
		#self.stop_criteria=stop_criteria
		P = 0.08
		I = 0.002
		D = 0.002
		self.distance_from_base = 0.6       #it's a percentage
		self.pid = PID.PID(P, I, D)
		self.size = 640 ###### TODO --> PUT IT IN A CONFIG FILE
		self.targetT = 0#100
		self.verbose = verbose

		self.distance_point_of_interest=10
		self.trajectory=[]

		#self.subscribe("StreetLane", "street_lines")

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

	def get_distance(slef, point1, point2):
		return math.sqrt((point1[0]-point2[0])**2+(point1[1]-point2[1])**2)

	def run(self,  car_position):
		#while not self.stop_criteria():
		#lines=[None, None]
		#if("street_lines" in self.data):
		#	lines = json.loads(self.data["street_lines"])
		#actual_trajectory=self.get_current_trajectory_point(lines)
		#if(actual_trajectory==None):
		#	actual_trajectory=self.targetT

		if(self.get_distance(car_position, self.trajectory[0])< self.distance_point_of_interest*0.9):
			self.trajectory.pop(0)

		#x=[car_position[0]]+[t[0] for t in trajectory]
		#y=[car_position[1]]+[t[1] for t in trajectory]

		next_point_distance = car_model.world_to_car(car_position[0], car_position[1], car_position[2], self.trajectory[0])
		print("distance given to PID:" + str(next_point_distance[1]))
		#car_to_world


		#Interpolate is more elegant but problem with vertical trajectories
		#tck, u = interpolate.splprep([x, y], k = int(self.order), s = 0)
        #xi_temp, yi_temp = interpolate.splev(np.linspace(0, self.distance_point_of_interest, 100), tck)

		#trajectory_splline = interpolate([car_position] + trajectory)
		#next_trajectory_point= trajectory_splline(s=distance_point_of_interest)
		#error=distance_pitagora(next_trajectory_point,
		#						objective_point_world)
		#evaluate_pid(error)


		steer=self.get_steer(next_point_distance[1])

		#command = json.dumps({"velocity" : 0, "steer" : steer})
		#self.pub.publish(command)
		#sleep(0.1)

		if(self.verbose):
			steer_dir=""
			if(-steer > 0):
				steer_dir="right"
			else:
				steer_dir="left"
			print("COMMAND : VELOCITY = 0.17 | STEER = "+steer_dir+ " --> " +str(steer) + " ERROR : "+str(next_point_distance))
		return steer

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
