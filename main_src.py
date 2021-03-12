#!/usr/bin/python 
# DA SPOSTARE SULLA CARTELLA PRECEDENTE
""" 
These are the libraries you need to import in your project in order to
be able to communicate with the Gazebo simulator
"""
from bfmclib.gps_s import Gps
from bfmclib.bno055_s import BNO055
from bfmclib.camera_s import CameraHandler
from bfmclib.controller_p import Controller
from bfmclib.trafficlight_s import TLColor, TLLabel, TrafficLight
import rospy
import cv2
from time import sleep
### LINE TRACKING
import sys
import os
#sys.path.insert(0, '../src/SimulatorCode/')
#import ParticleFilter as pf
#import ParticleFilter
#from image_processing import ImageProcessing
from SimulatorCode.controlUnit import ControlUnit
from SimulatorCode.processer import Processer
try:

	# This line should be the first line in your program
	rospy.init_node('main_node', anonymous=True)

	cam = CameraHandler()
	print("Camera loaded")

	car = Controller()
	print("Controller loaded")

	sem = TrafficLight()
	print("Traffic lights listener")

	gps = Gps()
	print("Gps loaded")

	bno = BNO055()
	print("BNO055 loaded")

	print("Sending move with speed 0, steering 0")
	car.drive(0, 0)
	sleep(0.5)

	run_type="ros"
	if(run_type=="original"):
		steering = 0.0
		speed = 0.0
		while 1:
			cv2.imshow("Frame preview", cam.getImage())
			car.drive(1, 1)
			key = cv2.waitKey(1)

			if key == ord('q'):
				cv2.destroyAllWindows()
				break
		cv2.destroyAllWindows()
	elif(run_type=="lanes"):
		N_particles           = 50  #100 # Particles used in the filter
		Interpolation_points  = 17  #25  # Interpolation points used for the spline
		order                 = 2        # Spline order
		N_c                   = 3        # Number of spline control points

		ParticleFilter.filter_usage_BOSH(N_Particles=N_particles,
							Interpolation_points=Interpolation_points,
							order=1,
							N_points=2,
							get_image_function=cam.getImage)
	elif(run_type=="export"):
		step=0
		while 1:
			step=step+1
			img=cam.getImage()
			cv2.imshow("Frame preview", cam.getImage())
			cv2.imwrite('/tmp/dataset/img_' + str(step) + '.png', img)
			key = cv2.waitKey(40)

			if key == ord('q'):
				cv2.destroyAllWindows()
				break
		cv2.destroyAllWindows()
	elif(run_type=="cu"):
		cu=ControlUnit(cam, car, sem, gps, bno)
		cu.start()
	elif(run_type=="ros"):
		#pro=Processer(cam, car, sem, gps, bno)
		#pro.start()
		os.system("rosrun startup_package processer.py &")
		while not rospy.is_shutdown():
			sleep(0.5)
	#while 1:
	#	#cv2.imshow("Frame preview", cam.getImage())

	#	key = cv2.waitKey(1)

	#	if key == ord('q'):
	#		cv2.destroyAllWindows()
	#		break

	#print("Car stopped. \n END")
	#car.stop(0.0)

except Exception as e:
	print("Error in main.py")
	print(e)
