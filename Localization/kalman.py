#!/usr/bin/env python3

from time import sleep

import rospy
from car_plugin.msg import Command

import sys
#sys.path.insert(0, '/home/{YOUR_USERNAME}/Documents/BFMC_Simulator/startup_workspace/src/startup_package/src')
sys.path.insert(0, './src/startup_package/src/')
#sys.path.insert(0, '../../src')

from bfmclib.gps_s import Gps
from bfmclib.bno055_s import BNO055
#from bfmclib.controller_p import Controller



import math

#import tf
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseWithCovariance, PoseWithCovarianceStamped, TwistWithCovariance
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry

import math

import numpy as np
from scipy.spatial.transform import Rotation as Rot


# import the random module
import random

class Kalman:
    
    def __init__(self, gps, bno):
        print("Initialising Kalman")
        # Define name of the Node
        rospy.init_node("Kalman", anonymous=True)

        now = rospy.get_time()
        # Define set of topics to subscribe to

        # Create subscriber node
        rospy.Subscriber("/rcCar/Command", Command, self.Control)
        self.control_state = [0.0,0.0]

        self.bno = bno
        self.bno_measure = False
        self.bno_state = 0

        self.gps = gps
        self.gps_measure = False
        self.gps_state = 0

        self.kalman_pub = rospy.Publisher('Kalman', Vector3, queue_size=20)


        # Kalman states
        self.x_t = 0.0
        self.y_t = 0.0
        self.yaw_t = 0.0
        self.v_t = 0.0
        self.omega_t = 0.0

        # State-Vector
        self.X_t = np.array([self.x_t,      self.y_t,      self.yaw_t,
                        self.v_t,  self.omega_t])

        # Filter Covariance Matrix
        self.P_t = np.eye(5)

        # Initialise Measurements Vector
        self.Z = np.array([])
        # Initialise Measurements Covariance Matrix
        self.R = np.array([])
        # Initialise Measurements Matrix
        self.H = np.zeros((5,0))
        # Initialise Measurements Jacobian Matrix
        self.J_H = np.zeros((5,0))



    # Prediction step with only the kinematic model
    def Predict(self, dt):

        # State-Transition Matrix
        A = np.array([  [1.0, 0.0, 0.0, dt*math.cos(self.X_t[2]), 0.0],
                        [0.0, 1.0, 0.0, -dt*math.sin(self.X_t[2]), 0.0],
                        [0.0, 0.0, 1.0, 0.0, dt],
                        [0.0, 0.0, 0.0, 1.0, 0.0],
                        [0.0, 0.0, 0.0, 0.0, 1.0]])

        # Noise Variance
        sigma_noise = 0.001

        # Noise Matrix
        W = np.array([  random.gauss(mu = 0, sigma = sigma_noise),
                        random.gauss(mu = 0, sigma = sigma_noise),
                        random.gauss(mu = 0, sigma = sigma_noise)/10,
                        random.gauss(mu = 0, sigma = sigma_noise)/10,
                        random.gauss(mu = 0, sigma = sigma_noise)/100])

        # Jacobian of Transition Matrix
        J_A = np.array([[1.0, 0.0, 0.0, dt*(-math.sin(self.X_t[2])), 0.0],
                        [0.0, 1.0, 0.0, -dt*math.cos(self.X_t[2]), 0.0],
                        [0.0, 0.0, 1.0, 0.0, dt],
                        [0.0, 0.0, 0.0, 1.0, 0.0],
                        [0.0, 0.0, 0.0, 0.0, 1.0]])

        # Prediction Covariance
        Q = np.array([  [sigma_noise, 0.0, 0.0, 0.0, 0.0],
                        [0.0, sigma_noise, 0.0, 0.0, 0.0],
                        [0.0, 0.0, sigma_noise/10, 0.0, 0.0],
                        [0.0, 0.0, 0.0, sigma_noise/10, 0.0],
                        [0.0, 0.0, 0.0, 0.0, sigma_noise/100]])

        # Check control difference
        u = np.array([  0.0,
                        0.0,
                        0.0,
                        self.control_state[0] - self.X_t[3],
                        self.control_state[1] - self.X_t[4]])

        steps = 1
        B = np.diag(np.array([0,0,0,1/steps,1/steps]))

        # Prediction State
        self.X_Pred = A @ self.X_t + B @ u+ W

        #print("Predict" + str(self.X_Pred) )


        # Prediction Covariance Matrix
        self.P_Pred = J_A @ self.P_t @ J_A.T + Q # ??? + A@Q@A.T ???
        self.P_Pred = (self.P_Pred + self.P_Pred.T) / 2 # Ensure that it is symmetric

    # Prediction step with only the kinematic model
    def UpTest(self):

        #print("No Measurement")

        self.X_t = self.X_Pred
        self.P_t = self.P_Pred


    # Update step with the measurements
    def Update(self):

        #print("Update")
        # Check if there are more updates
        if(self.bno_measure or self.gps_measure):
            # Reset Measurements check
            self.bno_measure = self.gps_measure = False

            
            # Transpose matrices after their creation
            self.H = self.H.T
            self.J_H = self.J_H.T
            self.R = np.diag(self.R)
            # Predicted using measurements matrix
            Z_pred = self.H @ self.X_Pred
            # Innovation
            Y = self.Z - Z_pred
            # Innovation Covariance
            S = self.J_H @ self.P_Pred @ self.J_H.T + self.R
            # Kalman Gain
            K = self.P_Pred @ self.J_H.T @ np.linalg.pinv(S) # Using the pseudo-inverse to avoid singularity
            # State Update
            self.X_t = self.X_Pred + K @ Y
            # Covariance Update
            #self.P_t = (np.eye(5) - K @ self.J_H) @ self.P_Pred
            # Joseph form Covariance Update equation -> Ensure Positive Semi-Definite
            self.P_t = (np.eye(5) - K @ self.J_H) @ self.P_Pred @ (np.eye(5) - K @ self.J_H).T + K @ self.R @ K.T
            # Ensure P is symmetric
            #self.P_t = (self.P_t + self.P_t.T) / 2



            # Initialise Measurements Vector
            self.Z = np.array([])
            # Initialise Measurements Covariance Matrix
            self.R = np.array([])
            # Initialise Measurements Matrix
            self.H = np.zeros((5,0))
            # Initialise Measurements Jacobian Matrix
            self.J_H = np.zeros((5,0))

            #print("Update " + str(self.X_t))
        else:
            # Keep just the prediction if no new measurements have been received
            self.UpTest()


        # next, we'll publish the pose message over ROS
        position = Vector3(self.X_t[0], self.X_t[1], self.X_t[2])
        
        # publish the message
        self.kalman_pub.publish(position)

    def Control(self, command):

        msg = command.msg_val

        if(len(msg) == 0 ):
            speed = 0
            angle = 0
        elif(len(msg) == 1 ):
            speed = 0
            angle = 0#self.car.msg_command.msg_val[0]
        elif(len(msg) > 1 ):
            speed = msg[0]
            angle = msg[1]

        v = speed 
        omega = angle / 180 * math.pi
    
        self.control_state = [float(v), float(omega)]
        
        #print(str(self.control_state))


    def BNO(self):

        yaw = self.bno.getYaw()

        if(yaw != None and self.bno_state != yaw):

            #print("BNO - " + str(yaw))

            self.bno_state =  yaw
            self.bno_measure = True

            self.Z = np.append(self.Z, np.array([yaw]))
            self.R = np.append(self.R, np.array([0.0001]))

            self.H = np.column_stack([self.H, np.array([0,0,1,0,0])])
            self.J_H = np.column_stack([self.J_H, np.array([0,0,1,0,0])])


    def GPS(self):

        gps_data = self.gps.getGpsData()

        if(gps_data["coor"] != None and self.gps_state != gps_data["timestamp"]):

            self.gps_state = gps_data["timestamp"]
            self.gps_measure = True

            x = gps_data["coor"][0].real
            y = gps_data["coor"][0].imag
            cos_yaw = gps_data["coor"][1].real
            sin_yaw = gps_data["coor"][1].imag
            yaw = math.atan2(sin_yaw, cos_yaw)

            #print("GPS - " + str(x) + " - " + str(y) + " - " + str(yaw) )

            self.Z = np.append(self.Z, np.array([x,y,yaw]))
            self.R = np.append(self.R, np.array([0.15,0.15,0.01]))

            self.H = np.column_stack([self.H, np.array([1,0,0,0,0]), np.array([0,1,0,0,0]), np.array([0,0,1,0,0])])
            self.J_H = np.column_stack([self.J_H, np.array([1,0,0,0,0]), np.array([0,1,0,0,0]), np.array([0,0,1,0,0])])
            

if __name__ == '__main__':

    try:
        gps = Gps()
        print("Gps loaded")

        bno = BNO055()
        print("BNO055 loaded")
        
        kalman = Kalman(gps, bno)
        
        rate = rospy.Rate(50) # Hz
        start = rospy.get_time()
        end = start
        while not rospy.is_shutdown():

            # Update dt at each iteration
            start = rospy.get_time()
            dt = start - end
            #print(str(start) + "  "  +  str(dt))

             # Prediction step of the Kalman Filter
            kalman.Predict(dt)

            # Get measurements
            kalman.GPS()
            kalman.BNO()
            #kalman.Control()

            # Execute Update step
            kalman.Update()

            #print("Kalman : " + str(kalman.X_t))

            end = start = rospy.get_time()

            # Sleep before next iteration
            rate.sleep()


    except Exception as e:
        print(e)
