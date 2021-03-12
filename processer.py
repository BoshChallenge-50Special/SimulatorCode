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

import math  

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
		
		# Salva i dati per la macchina nel Processer
		self.speed = 0
		self.steering = 0 
		self.state = "Straight"


	def start(self):
		# Far eseguire il file come se fosse eseguito da terminale
		os.system("rosrun startup_package horizontal_line.py &")
		os.system("rosrun startup_package ParticleFilter.py &")
		os.system("rosrun startup_package PidControl.py &")

		self.subscribe("HorizontalLine", "horizontal_line")
		self.subscribe("StreetLane", "street_lines")
		self.subscribe("PidControlValues", "velocity_steer")
		self.subscribe("Sign", "sign")

		state_machine=False

		stateMachineSteer = StateMachineSteer()
		stateMachineVelocity = StateMachineVelocity()

		self.speed = 0.22
		self.steering = 0
		current_speed = 0.22   # For incrementing or decrementing to reach a target

		# Wait 2 seconds for the processes to actually start
		sleep(2)

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
					self.steering = 0
				elif(state_steer=="OnLane"):
					velocity_steer = json.loads(self.data["velocity_steer"])
					self.steering = -velocity_steer["steer"]
				elif(state_steer=="NearCrossroad"):
					velocity_steer = json.loads(self.data["velocity_steer"])
					self.steering = -velocity_steer["steer"]
				elif(state_steer=="OnCrossroad"):
					self.steering = self.turn('right')
					#self.steering = turn_right()	# RIGHT
					#self.steering = 0			# STRAIGHT
					#self.steering = turn_left()	# LEFT

				state_velocity=stateMachineVelocity.runOneStep(data_state_machine)
				if(state_velocity=="OnSteady"):
					self.speed = 0
				elif(state_velocity=="Slow"):
					if(current_speed > 0.2):
						self.speed = min(0.2, current_speed-0.1)
					else:
						self.speed = max(0.2, current_speed+0.1)
				elif(state_velocity=="Fast"):
					if(current_speed > 0.8):
						self.speed = min(0.8, current_speed-0.1)
					else:
						self.speed = max(0.8, current_speed+0.1)
				current_speed = self.speed
			else:
				#cv.imshow("Processer", cam.getImage())
				#cv.waitKey(1)
				# COMMENTED TO TEST THE TURNING PROCEDURE
				#print("Sending move with speed "+ str(speed) + " , self.steering " + str(self.steering))
				if(self.state == "Straight"):
					if("street_lines" in self.data):
						lines = json.loads(self.data["street_lines"])
						#print(lines[0][0])
					if("velocity_steer" in self.data):
						velocity_steer = json.loads(self.data["velocity_steer"])
						self.steering = -velocity_steer["steer"]

				if ("horizontal_line" in self.data):
					if(self.data["horizontal_line"] == "Stop"):
						direction = "left"
						self.turn(direction)

			self.car.drive(self.speed, self.steering)

			sleep(0.1)


		#rospy.spin()

	# TODO Fix the time for the sleep in the manuevres
	def turn(self, direction):
		# Define that you want to turn
		self.state = "Turn"
		print("Turn " + direction)

		self.car.drive(0.2, self.steering) # Slow down to be more sure about the time to be waited

		# Wait for the stop signal to be surpassed
		while(self.data["horizontal_line"] != "Safe"):
			sleep(0.1)

		# Act based on the decision you want
		if(direction == 'right'):
			sleep(2)
			curve_radius = 66.6 # curve_radius = 66.5 # most common radius in the circuit
			ipotenusa    = math.sqrt(2 * (curve_radius ** 2) )
			steer        = 90 - math.acos(curve_radius/ipotenusa)
		elif(direction == 'left'):
			sleep(3.75)
			curve_radius = 103.6 
			ipotenusa    = math.sqrt(2 * (curve_radius ** 2) )
			steer        = - ( 90 - math.acos(curve_radius/ipotenusa) )
		else:
			steer = 0

		# Steer the car accordingly
		#self.car.drive(self.speed, self.steering + steer)
		self.car.drive(0.2, self.steering + steer) # Set a known velocity for the manuevre
		# Wait for the manuevre to end
		sleep(3.2)
		self.state == "Straight"



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
