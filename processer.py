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
from SimulatorCode.StateMachine.stateMachineSteer import StateMachineSteer
from SimulatorCode.StateMachine.stateMachineVelocity import StateMachineVelocity


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
		os.system("rosrun startup_package traffic.py &")

		self.subscribe("HorizontalLine", "horizontal_line")
		self.subscribe("StreetLane", "street_lines")
		self.subscribe("PidControlValues", "velocity_steer")
		self.subscribe("Sign", "sign")

		state_machine=False

		stateMachineSteer = StateMachineSteer()
		stateMachineVelocity = StateMachineVelocity()

		speed = 0.22
		steering = 0
		current_speed = 0.22   # For incrementing or decrementing to reach a target

		while not rospy.is_shutdown():
			if(state_machine):
				data_state_machine={}

				data_state_machine["moving"] = True
				data_state_machine["horizontal_line"] = True
				data_state_machine["turning"] = True
				data_state_machine["stop_signal"] = False
				data_state_machine["pedestrian"] = True
				data_state_machine["pedestrian_line"] = True
				data_state_machine["state_steer"] = True

				state_steer=stateMachineSteer.runOneStep(data_state_machine)
				if(state_steer=="Steady"):
					steering = 0
				elif(state_steer=="OnLane"):
					velocity_steer = json.loads(self.data["velocity_steer"])
					steering = -velocity_steer["steer"]
				elif(state_steer=="NearCrossroad"):
					velocity_steer = json.loads(self.data["velocity_steer"])
					steering = -velocity_steer["steer"]
				elif(state_steer=="OnCrossroad"):
					steering = turn_right()	# RIGHT
					steering = 0			# STRAIGHT
					steering = turn_left()	# LEFT

				state_velocity=stateMachineVelocity.runOneStep(data_state_machine)
				if(state_velocity=="OnSteady"):
					speed = 0
				elif(state_velocity=="Slow"):
					if(current_speed > 0.2):
						speed = min(0.2, current_speed-0.1)
					else:
						speed = max(0.2, current_speed+0.1)
				elif(state_velocity=="Fast"):
					if(current_speed > 0.8):
						speed = min(0.8, current_speed-0.1)
					else:
						speed = max(0.8, current_speed+0.1)
				current_speed = speed
			else:
				#cv.imshow("Processer", cam.getImage())
				#cv.waitKey(1)

				#print("Sending move with speed "+ str(speed) + " , steering " + str(steering))
				if("street_lines" in self.data):
					lines = json.loads(self.data["street_lines"])
					#print(lines[0][0])
				#if("sign" in self.data):
				#	# lines = json.loads(self.data["sign"])
				#	#print("FROM PROCESSER" + self.data["sign"])
				if("velocity_steer" in self.data):
					velocity_steer = json.loads(self.data["velocity_steer"])
					steering = -velocity_steer["steer"]
				self.car.drive(speed, steering)

			self.car.drive(speed, steering)
			sleep(0.1)


		#rospy.spin()

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
