#!/usr/bin/python
"""
DA METTERE SU src/startup_package/src
These are the libraries you need to import in your project in order to
be able to communicate with the Gazebo simulator
"""

import sys
#sys.path.insert(0, '/home/{YOUR_USERNAME}/Documents/SimulatorCode/')
sys.path.insert(0, '/home/marco/Documents/BoschChallenge/SimulatorCode')

from processer import Processer


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

pro=Processer(cam, car, sem, gps, bno)
pro.start()
