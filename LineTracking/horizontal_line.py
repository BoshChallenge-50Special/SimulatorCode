
import sys
import cv2 as cv
import numpy as np
import math


import sys
#sys.path.insert(0, '/home/{YOUR_USERNAME}/Documents/BFMC_Simulator/startup_workspace/src/startup_package/src')
sys.path.insert(0, '/home/marco/Documents/BFMC_Simulator/startup_workspace/src/startup_package/src/')


from bfmclib.gps_s import Gps
from bfmclib.bno055_s import BNO055
from bfmclib.camera_s import CameraHandler
from bfmclib.controller_p import Controller
from bfmclib.trafficlight_s import TLColor, TLLabel, TrafficLight

import rospy
from std_msgs.msg import String

import rospy
import cv2

from time import sleep

import numpy as np
import math


class HorizontalLine(object):

	def __init__(self):
		pub = rospy.Publisher('HorizontalLine', String, queue_size=10)
		rospy.init_node('HorizontalLineDetector', anonymous=True)
		self.rate = rospy.Rate(10) # 10hz
		self.pub = pub

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
		vertices = np.array([[(300,240),(imshape[1]-300, 240), (imshape[1]-150, imshape[0]-20), (150,imshape[0]-20)]], dtype=np.int32)
		cv.fillPoly(mask, vertices, ignore_mask_color)
		masked_edges = cv.bitwise_and(edges, mask)
		
		# Define the Hough transform parameters
		# Make a blank the same size as our image to draw on
		rho = 2 # distance resolution in pixels of the Hough grid
		theta = np.pi/180 # angular resolution in radians of the Hough grid
		threshold = 15     # minimum number of votes (intersections in Hough grid cell)
		min_line_length = 40 #minimum number of pixels making up a line
		max_line_gap = 30    # maximum gap in pixels between connectable line segments
		line_image = np.copy(image)*0 # creating a blank to draw lines on
		
		# Run Hough on edge detected image
		# Output "lines" is an array containing endpoints of detected line segments
		lines = cv.HoughLinesP(masked_edges, rho, theta, threshold, np.array([]),
									min_line_length, max_line_gap)
		
		horizontal = 0
		if str(lines) != "None":
			# Iterate over the output "lines" and draw lines on a blank image
			for line in lines:
				for x1,y1,x2,y2 in line:

					delta_x = x2 - x1
					delta_y = y2 - y1
					theta_radians = math.atan2(delta_y, delta_x)
					if((theta_radians/math.pi*180 > 175 and theta_radians/math.pi*180 < 185) or (theta_radians/math.pi*180 < 5 and theta_radians/math.pi*180 > -5)):
						horizontal = horizontal + 1 

						cv.line(line_image,(x1,y1),(x2,y2),(255,0,0),10)
			#print(str(horizontal) +"\t"+str(len(lines[0])))
			if(horizontal == 2 and horizontal == len(lines)):
				message = "Stop"
				#print(message)
				#rospy.loginfo(message)
				self.pub.publish(message)
				#data_queue.put("HorizontalLine: Found a STOP line")              
				
				# Draw the lines on the original image
				lines_edges = cv.addWeighted(image, 0.8, line_image, 1, 0)
				#data_queue.put(lines_edges) 

				cv.imshow("HorizontalLine", lines_edges)
				return 

			elif(horizontal >= 2 and horizontal < len(lines)):
				message = "Stop and be aware of pedestrians"
				#print(message)
				#rospy.loginfo(message)
				self.pub.publish(message)

				#data_queue.put("HorizontalLine: Found a STOP line with Pedestrians") 

				# Draw the lines on the original image
				lines_edges = cv.addWeighted(image, 0.8, line_image, 1, 0)
				#data_queue.put(lines_edges)

				cv.imshow("HorizontalLine", lines_edges)

				return
		
		cv.imshow("HorizontalLine", image)
		message = "Safe"
		self.pub.publish(message)


if __name__ == '__main__':
	try:		
		cam = CameraHandler()
		print("Camera loaded for Horizontal Line Detection")
		horizonLine = HorizontalLine()

		while not rospy.is_shutdown():
			img = cam.getImage()
			horizonLine.check_horizontal(img)

			horizonLine.rate.sleep()	
	except Exception as e:
		print(e)