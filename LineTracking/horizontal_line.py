
import sys
import cv2 as cv
import numpy as np
import math

class HorizontalLine(object):

	def check_horizontal (self, image, data_queue):
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
			if(horizontal == 2 and horizontal == len(lines[0])):
				print("Stop")
				#data_queue.put("HorizontalLine: Found a STOP line")              
				
				# Draw the lines on the original image
				lines_edges = cv.addWeighted(image, 0.8, line_image, 1, 0)
				#data_queue.put(lines_edges) 

				cv.imshow("HorizontalLine", lines_edges)
				return 

			elif(horizontal >= 2 and horizontal < len(lines[0])):
				print("Stop and be aware of pedestrians")
				#data_queue.put("HorizontalLine: Found a STOP line with Pedestrians") 

				# Draw the lines on the original image
				lines_edges = cv.addWeighted(image, 0.8, line_image, 1, 0)
				#data_queue.put(lines_edges)

				cv.imshow("HorizontalLine", lines_edges)

				return
		
		cv.imshow("HorizontalLine", image)