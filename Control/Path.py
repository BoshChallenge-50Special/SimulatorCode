#!/usr/bin/env python2

#import cv2 as cv
#import numpy as np
#import math


import rospy
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseWithCovariance, PoseWithCovarianceStamped, TwistWithCovariance
#from std_msgs.msg import String
from time import sleep
import sys
sys.settrace
import traceback
#from scipy import interpolate
#import json
import math
#import Control.CarModel as car_model
sys.path.insert(0, '../BFMC_Simulator/startup_workspace/src/startup_package/src')
sys.path.insert(0, './src/startup_package/src/')
sys.path.insert(0, './src/startup_package/src/SimulatorCode')
#sys.path.insert(0, '../../src')

from bfmclib.controller_p import Controller

#from SimulatorCode.templates import Producer, Consumer
#from Control import PID

from Localization import GraphMap

class PathFollow():

    def __init__(self, vicinity_threshold ,verbose=False):
        #@super(PathFollow, self).init("PathFollowler")

        # Velocity and Angular Velocity
        self.Command = [0.0, 0.0]

        # Define when to switch point to follow
        self.vicinity_threshold = vicinity_threshold

        self.path_followed = list()

        self.verbose = verbose


    def get_distance(self, point1, point2):

        return math.sqrt((point1[0]-point2[0])**2+(point1[1]-point2[1])**2)

    def get_angle(self, car_position, point):
        # Get the angle for the next point
        angle = math.atan2(car_position[1] - point[1], car_position[0] - point[0])
        # Remove the orientation angle
        angle = angle + car_position[2]
        # UnWrap the angle after orientation
        angle = ((angle ) % (2*math.pi) ) - math.pi
        # Scale the angle
        angle = angle / math.pi * 180

        return angle

    def run(self, car_position, next_points):
        # Check for closest points inside the vicinity threshold
        while(self.get_distance(car_position, next_points[0]) < self.vicinity_threshold):
            # Store close points(s) and go for the next ones
            self.path_followed.append({'i': len(self.path_followed), 'coor': next_points[0]})
            next_points.pop(0)

        angle = self.get_angle(car_position, next_points[0])
        self.Command[1] = angle

        if(self.verbose):
            steer_dir=""
            if(self.Command[1] > 0):
                steer_dir="right"
            elif(self.Command[1] < 0):
                steer_dir="left"
            else:
                steer_dir = "straight"
            print("Steer = "+steer_dir+ " --> " +str(angle) + " Distance : "+str(self.get_distance(car_position, next_points[0])) + " Points : "+str(len(self.path_followed)))

        return angle


# Current position of the car
car_position = [ 0.0, 0.0, 0.0]

def Kalman(kalman):
    global car_position
    car_position = [kalman.x, kalman.y, kalman.z]

    #print("Kalman " + str(car_position))


if __name__ == '__main__':
    try:
        # Import Map
        fileMap = './src/startup_package/src/SimulatorCode/Localization/Competition_track.graphml'
        Graph = GraphMap.GraphMap(fileMap)

        # Import Car Controller
        rospy.init_node("PathFollow", anonymous=True)
        car=Controller()


        # Get Kalman
        # Create subscriber node
        rospy.Subscriber("Kalman", Vector3, Kalman)
        # Wait for first measurement
        sleep(1)

        # Get point of position of car
        car_points = Graph.get_location_points(car_position[0], car_position[1], 1)
        # Calculate path
        path, length = Graph.get_path_coor(car_points[0][0], "540")
        print(path)
        print(length)

        print("Starting PathFollow")
        vicinity_threshold = 0.5
        follow = PathFollow(vicinity_threshold = vicinity_threshold, verbose = True)

        """ TEST
        # X Y Yaw
        car_position = [0.79, 14.91, math.pi/2]
        # Path (now and forward) (561-540)
        next_points = [[0.79, 14.91], [0.8, 14.59], [0.81, 14.27], [0.63, 13.53], [1.37, 13.71], [1.69, 13.72], [2.01, 13.73], [2.14, 13.71], [2.88, 13.53], [3.61, 13.71], [3.93, 13.72], [4.34, 13.72], [5.08, 13.54], [5.82, 13.72], [6.13, 13.75], [6.41, 13.9], [6.69, 14.06], [7.0, 14.1], [7.32, 14.13], [7.64, 14.12], [7.95, 14.08], [8.27, 14.04], [8.58, 13.98], [8.88, 14.06], [9.2, 14.09], [9.52, 14.1], [9.84, 14.1], [10.16, 14.08], [10.47, 14.02], [10.77, 13.89], [11.05, 13.75], [11.33, 13.59], [11.59, 13.41], [11.82, 13.18], [12.03, 12.95], [12.29, 12.75], [12.59, 12.66], [12.91, 12.65], [13.23, 12.66], [13.53, 12.55], [13.77, 12.34], [13.96, 12.08], [14.05, 11.77], [14.03, 11.45], [13.93, 11.15], [13.7, 10.91], [13.44, 10.74], [13.12, 10.67], [12.8, 10.66], [12.49, 10.57], [12.22, 10.4], [11.99, 10.16], [11.86, 9.87], [11.83, 9.55], [11.89, 9.23], [12.03, 8.95], [12.25, 8.72], [12.35, 8.41], [12.43, 8.1], [12.57, 7.82], [12.78, 7.58], [13.04, 7.4], [13.26, 7.16], [13.38, 6.87], [13.42, 6.55], [13.37, 6.23], [13.2, 5.96], [12.93, 5.78], [12.64, 5.63], [12.36, 5.47], [12.12, 5.25], [11.96, 4.97], [11.89, 4.66], [11.87, 4.34]]
        follow.run(car_position, next_points)
        """

        angle = follow.run(car_position, path)


        while (len(path) >= 0 ):
            angle = follow.run(car_position, path)
            car.drive(0.17, angle)
            sleep(0.1)

        car.drive(0,0)

    except Exception as e:
        print("Error in Path.py")
        print(e)
        print(traceback.print_exc())
