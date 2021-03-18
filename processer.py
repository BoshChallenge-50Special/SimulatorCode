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

        self.directions=["straight", "left", "right", "left", "straight"]


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

        ########  TO REMOVE ##################
        pub = rospy.Publisher("REMAIN_LEFT", String, queue_size=10)
        pub.publish("Normal")

        ########  TO REMOVE ##################

        state_machine=True

        stateMachineSteer = StateMachineSteer()
        stateMachineVelocity = StateMachineVelocity()

        old_state_steer    = "None"
        old_state_velocity = "None"
        old_state_sign = "None"

        self.speed = 0.22
        self.steering = 0
        current_speed = 0.22   # For incrementing or decrementing to reach a target

        print_msg = ""
        # Wait 20 seconds for the processes to actually start
        sleep(10)

        data_state_machine={}
        while not rospy.is_shutdown():
            if(state_machine):
                state_steer="Steady"
                data_state_machine["moving"] = True
                data_state_machine["horizontal_line"] = "" # Fix it as safe, otherwise there might be a problem if it's missing
                if ("horizontal_line" in self.data):
                    data_state_machine["horizontal_line"] = self.data["horizontal_line"]

                data_state_machine["turning"] = False

                if("sign" in self.data):
                    if(old_state_sign!=self.data["sign"]):
                        old_state_sign=self.data["sign"]
                        signs = json.loads(self.data["sign"])
                        data_state_machine["stop_signal"] = "STOP" in signs
                        data_state_machine["parking_signal"] = "PARKING" in signs
                        data_state_machine["pedestrian_signal"] = "CROSSWALK SIGN" in signs
                        print(signs)
                        if "STOP" in self.data["sign"]:
                            print(signs[0] + ' SIGN detected, checking horizontal line presence.')
                        elif "CROSSWALK SIGN" in self.data["sign"] and  state_steer != 'OnCrosswalk' :
                            print( signs[0] + ' detected, checking zebra crossing presence.')
                        #elif self.data["sign"] == "PARKING":
                        #    print( self.data["sign"] + 'SIGN detected, slowing down.')
                        elif "PRIORITY" in self.data["sign"] :
                            print( signs[0] + 'SIGN detected, going fast.')
                else:
                    data_state_machine["stop_signal"] = False
                    data_state_machine["parking_signal"] = False
                    data_state_machine["pedestrian_signal"] = False

                state_steer=stateMachineSteer.runOneStep(data_state_machine)
                if(state_steer=="Steady"):
                    self.steering = 0
                elif(state_steer=="OnLane" or state_steer=="OnCrosswalk"):
                    if("velocity_steer" in self.data):
                        velocity_steer = json.loads(self.data["velocity_steer"])
                        self.steering = -velocity_steer["steer"]
                elif(state_steer=="NearCrossroad"):
                    if("velocity_steer" in self.data):
                        velocity_steer = json.loads(self.data["velocity_steer"])
                        self.steering = -velocity_steer["steer"]
                elif(state_steer=="OnCrossroad"):
                    pub.publish("LEFT")
                    #if state_velocity == 'OnSteady':
                    #    print_msg = 'The car is still.'
                    #    print(print_msg)
                    #elif state_velocity=="Parking":
                    #    print("PARKING SIGN detected, going slow")
                    #else:
                    #if state_steer == 'OnLane':
                    #    print_msg =  'Car is KEEPING THE LANE'
                    #if state_steer == 'OnCrossroad':
                    #    print_msg = 'Car is NAVIGATING AN INTERSECTION'
                    #elif state_steer == 'NearCrossroad':
                    #    print_msg = 'Car is approaching an intersection'
                    #elif state_steer == 'OnCrosswalk':
                    #    print_msg = 'Car is over a CROSSWALK'
                    #print(print_msg + " and it is going " + state_velocity + ".")  # Repeated here becayuse turn take the full control


                    #self.steering = self.turn('right')
                    if(len(self.directions)):
                        self.turn()
                    else:
                        self.speed = 0 
                        data_state_machine["moving"] = "False"
                        print("End of Simulation Round")
                #pritn(state_steer +"     " +state_velocity)

                data_state_machine["state_steer"] = state_steer
                state_velocity=stateMachineVelocity.runOneStep(data_state_machine)
                if(state_velocity=="OnSteady"):
                    self.speed = 0
                elif(state_velocity=="Slow" or state_velocity=="Parking"):
                    if(current_speed > 0.2):
                        self.speed = min(0.1, current_speed-0.1)
                    else:
                        self.speed = max(0.1, current_speed+0.1)
                elif(state_velocity=="Fast"):
                    if(current_speed > 0.8):
                        self.speed = min(0.3, current_speed-0.05)
                    else:
                        self.speed = max(0.3, current_speed+0.05)
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
                    #if("sign" in self.data):
                    #    # lines = json.loads(self.data["sign"])
                    #    #print("FROM PROCESSER" + self.data["sign"])
                    if("velocity_steer" in self.data):
                        velocity_steer = json.loads(self.data["velocity_steer"])
                        self.steering = -velocity_steer["steer"]

                if ("horizontal_line" in self.data):
                    if(self.data["horizontal_line"] == "Stop"):
                        direction = "straight"
                        self.turn(direction)

            self.car.drive(self.speed, self.steering)
            if(old_state_steer != state_steer or old_state_velocity != state_velocity):
                if state_velocity == 'OnSteady':
                    print('The car is still.')
                elif state_velocity=="Parking":
                    print("PARKING SIGN detected, going slow")
                else:
                    if state_steer == 'OnLane':
                        print_msg =  'Car is KEEPING THE LANE'
                    #elif state_steer == 'OnCrossroad':
                    #    print_msg = 'Car is NAVIGATING AN INTERSECTION'
                    #elif state_steer == 'NearCrossroad':
                    #    print_msg = 'Car is approaching an intersection'
                    elif state_steer == 'OnCrosswalk':
                        print_msg = 'Car is near a CROSSWALK'
                    print(print_msg + " and it is going " + state_velocity + ".")

                old_state_steer = state_steer
                old_state_velocity = state_velocity
            sleep(0.1)

        #rospy.spin()

    # TODO Fix the time for the sleep in the manuevres
    def turn(self):

        # Define that you want to turn
        self.state = "Turn"
        print("Going " + self.directions[0])

        # Make sure to STOP at STOP
        self.car.drive(0, 0)
        sleep(5)
        self.car.drive(0.2, 0) # Slow down to be more sure about the time to be waited

        # Act based on the decision you want
        if(self.directions[0] == 'right'):
            sleep(9)
            #curve_radius = 66.6 # curve_radius = 66.5 # most common radius in the circuit
            #ipotenusa    = math.sqrt(2 * (curve_radius ** 2) )
            #steer        = 90 - math.acos(curve_radius/ipotenusa)
            steer = 22.5
            manuevre = 11.5
        elif(self.directions[0] == 'left'):
            sleep(13)
            #curve_radius = 103.6 
            #ipotenusa    = math.sqrt(2 * (curve_radius ** 2) )
            #steer        = - ( 90 - math.acos(curve_radius/ipotenusa) )
            steer = -17.5
            manuevre = 16
        else:
            steer = 0
            manuevre = 20

        # Steer the car accordingly
        self.car.drive(0.2, steer ) # Set a known velocity for the manuevre
        # Wait for the manuevre to end
        sleep(manuevre)
        # Update direction
        dir = self.directions.pop(0)
        #self.directions.append(dir) # Removed to end the simulation when the directions are finished
        self.state = "Straight"


if __name__ == '__main__':
    try:
        print("Loaded processer")

        processer=Processer()
        processer.start()

        #while not rospy.is_shutdown():
        #    img = cam.getImage()
        #    horizonLine.check_horizontal(img)

        #    horizonLine.rate.sleep()
    except Exception as e:
        print("Error in Processer")
        print(e)
        print(traceback.print_exc())
