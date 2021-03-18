#!/usr/bin/env python2

import sys
import cv2 as cv
import numpy as np
import math


import sys
#sys.path.insert(0, '/home/{YOUR_USERNAME}/Documents/BFMC_Simulator/startup_workspace/src/startup_package/src')
sys.path.insert(0, './src/startup_package/src/')
#sys.path.insert(0, '../../src')

from bfmclib.gps_s import Gps
from bfmclib.bno055_s import BNO055
from bfmclib.camera_s import CameraHandler
from bfmclib.controller_p import Controller
from bfmclib.trafficlight_s import TLColor, TLLabel, TrafficLight

from SimulatorCode.templates import Producer

import rospy
from std_msgs.msg import String

import rospy

from time import sleep

import numpy as np
import math

#import json

class HorizontalLine(Producer):

	def __init__(self):
		super(HorizontalLine, self).__init__("HorizontalLineDetector")
		#pub = rospy.Publisher('HorizontalLine', String, queue_size=10)
		#rospy.init_node('HorizontalLineDetector', anonymous=True)
		#self.rate = rospy.Rate(10) # 10hz
		#self.pub = pub
		self.set_publisher("HorizontalLine")
		self.status = "Safe"
		self.count = 0

		#self.pub_lines = rospy.Publisher("HorizontalLinesShow", String, queue_size=10)

	def check_horizontal (self, image):
		#Loading test images
		#image = cv.imread('test_images/solidWhiteRight.jpg')

		#Convert to Grey Image
		grey_image = cv.cvtColor(image, cv.COLOR_BGR2GRAY)

		# Define a kernel size and apply Gaussian smoothing
		kernel_size = 5
		blur_gray = cv.GaussianBlur(grey_image,(kernel_size, kernel_size),0)

		# Define our parameters for Canny and apply
		low_threshold = 50
		high_threshold = 150
		edges = cv.Canny(blur_gray, low_threshold, high_threshold)

		# Next we'll create a masked edges image using cv.fillPoly()
		mask = np.zeros_like(edges)
		ignore_mask_color = 255

		# Defining Region of Interest
		imshape = image.shape
		vertices = np.array([[(310,240),(imshape[1]-310, 240), (imshape[1]-200, imshape[0]-66), (200,imshape[0]-66)]], dtype=np.int32)
		cv.fillPoly(mask, vertices, ignore_mask_color)
		masked_edges = cv.bitwise_and(edges, mask)

		# Define the Hough transform parameters
		# Make a blank the same size as our image to draw on
		rho = 2 # distance resolution in pixels of the Hough grid
		theta = np.pi/180 # angular resolution in radians of the Hough grid
		threshold = 15     # minimum number of votes (intersections in Hough grid cell)
		min_line_length = 30 #minimum number of pixels making up a line
		max_line_gap = 30    # maximum gap in pixels between connectable line segments
		line_image = np.copy(image)*0 # creating a blank to draw lines on

		# Run Hough on edge detected image
		# Output "lines" is an array containing endpoints of detected line segments
		lines = cv.HoughLinesP(masked_edges, rho, theta, threshold, np.array([]),
									min_line_length, max_line_gap)

		horizontal = 0
		horizontal_vec = []
		if str(lines) != "None":
			# Iterate over the output "lines" and draw lines on a blank image
			
			for line in lines:
				for x1,y1,x2,y2 in line:

					delta_x = x2 - x1
					delta_y = y2 - y1
					theta_radians = math.atan2(delta_y, delta_x)
					if((theta_radians/math.pi*180 > 175 and theta_radians/math.pi*180 < 185) or (theta_radians/math.pi*180 < 5 and theta_radians/math.pi*180 > -5)):
						horizontal_vec.append(line) # Store horizontal line for later checks

						horizontal = horizontal + 1
						cv.line(line_image,(x1,y1),(x2,y2),(255,0,0),10)
			
			if(horizontal == 2 and horizontal == len(lines[0])):
				if(self.status == "Stop"):
					self.count += 1
				else:
					self.count = 0
				self.status = "Stop"

				# Draw the horizontal lines on the original image
				image = cv.addWeighted(image, 0.8, line_image, 1, 0)

			elif(horizontal >= 2 and horizontal <= len(lines[0])):

				val_thres = 5
				for i in range(0 , len(horizontal_vec)):
					b = False
					for j in range(i + 1 , len(horizontal_vec)):
						if( ( abs ( horizontal_vec[i][0][1] - horizontal_vec[j][0][1] ) < val_thres ) ):
							b = True # They are on the same level
					if (b): # If two lines are at the same level, then the horizontal line detected are more than the actual horizontal ones
						horizontal -= 1

				if(horizontal == 2): # If there are 2 horizontal lines then it's just a stop
					if(self.status == "Stop"):
						self.count += 1
					else:
						self.count = 0
					self.status = "Stop"
				elif(horizontal > 2): # If there are more than 3 horizontal lines then it's a pedestrian thing
					if(self.status == "Pedestrians"):
						self.count += 1
					else:
						self.count = 0
					self.status = "Pedestrians"

				# Draw the lines on the original image
				image = cv.addWeighted(image, 0.8, line_image, 1, 0)
				
			else: # Otherwise if there are not enough horizontal lines it is safe
				if(self.status == "Safe"):
					self.count += 1
				else:
					self.count = 0
				self.status = "Safe"
			
		else: # Otherwise if there are no horizontal lines it is safe
			if(self.status == "Safe"):
				self.count += 1
			else:
				self.count = 0
			self.status = "Safe"

		# Show the lines in the image
		cv.imshow("HorizontalLine", image)
		cv.waitKey(1)

		#self.pub_lines.publish(json.dumps(horizontal_vec)) # Not working, to set line.tolist() when added to horizontal_vec
		# Publish the status only after 3 consequent equal status
		if(self.count == 3): 
			self.pub.publish(self.status)
			#print(self.status)


if __name__ == '__main__':

	try:
		cam = CameraHandler()
		print("Camera loaded for Horizontal Line Detection")
		horizonLine = HorizontalLine()

		while not rospy.is_shutdown():
			img = cam.getImage()
			horizonLine.check_horizontal(img)
			# Slow down the process of checking images to have better performances
			sleep(0.2) 
			#horizonLine.rate.sleep()
	except Exception as e:
		print(e)
