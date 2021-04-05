#!/usr/bin/env python3

"""
These are the libraries you need to import in your project in order to
be able to communicate with the Gazebo simulator
"""

# Necessari per ROS
import rospy
from std_msgs.msg import String # Ne esistono di molti altri tipi di messaggi che si possono usare
from geometry_msgs.msg import Vector3
import json
import sys
sys.settrace
import py_trees
import operator

import os
import traceback

#sys.path.insert(0, '/home/{YOUR_USERNAME}/Documents/BFMC_Simulator/startup_workspace/src/startup_package/src')
sys.path.insert(0, '../../BFMC_Simulator/startup_workspace/src/startup_package/src/')
sys.path.insert(0, './src/startup_package/src/')

#from bfmclib.gps_s import Gps
#from bfmclib.bno055_s import BNO055
#from bfmclib.camera_s import CameraHandler
from bfmclib.controller_p import Controller
#from bfmclib.trafficlight_s import TLColor, TLLabel, TrafficLight

#import cv2 as cv
from time import sleep
import rospy

import math

from SimulatorCode.templates import Consumer
#from SimulatorCode.StateMachine.stateMachineSteer import StateMachineSteer
#from SimulatorCode.StateMachine.stateMachineVelocity import StateMachineVelocity

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
        #self.index_turning = 0

        #self.directions=["straight", "left", "right", "left", "straight"]



    def create_root(self):
        def drive(self):
            #Evaluating trajectory with
            if(len(self.blackboard.lane)>0):
                actual_trajectory= get_current_trajectory_point(self.blackboard.lane)

            self.car.drive(self.blackboard.speed, self.blackboard.steering)
            return py_trees.common.Status.SUCCESS

        def drive_trough_crosswalk(self):
            self.car.drive(0.05, self.blackboard.steering)
            return py_trees.common.Status.RUNNING

        def gather_data(self):

            if("velocity_steer" in self.data):
                velocity_steer = json.loads(self.data["velocity_steer"])
                self.blackboard.steering = -velocity_steer["steer"]
                self.blackboard.speed = 0.1##velocity_steer["velocity"]
            else:
                self.blackboard.steering = 0
                self.blackboard.speed = 0

            if("sign" in self.data):
                 signs = json.loads(self.data["sign"])

                 self.blackboard.signal.stop = "STOP" in signs
                 self.blackboard.signal.parking = "PARKING" in signs
                 self.blackboard.signal.priority = "PRIORITY" in signs
                 self.blackboard.signal.crosswalk = "CROSSWALK SIGN" in signs
                 self.blackboard.signal.highway_entrance = "HIGHWAY ENTRANCE" in signs
                 self.blackboard.signal.one_way = "ONE WAY" in signs
                 self.blackboard.signal.highway_exit = "HIGHWAY EXIT" in signs
                 self.blackboard.signal.roundabout = "ROUNDABOUT" in signs
                 self.blackboard.signal.no_entry = "NO-ENTRY" in signs

            if ("horizontal_line" in self.data):
                 self.blackboard.signal.horizontal.type = self.data["horizontal_line"] #Safe,Stop,Pedestrians

            if("position" in self.data):
                self.blackboard.position.x = self.data["position"].x
                self.blackboard.position.y = self.data["position"].y
                self.blackboard.position.z = self.data["position"].z

            if("street_lines" in self.data):
                self.blackboard.lane = json.loads(self.data["street_lines"])
            else:
                self.blackboard.lane = []
            #print(self.blackboard)

            return py_trees.common.Status.SUCCESS


        #self.root = py_trees.composites.Sequence("Sequence - Root")
        #THIS CHOICE IS TO DISCUSS. Parallel is made so it always gather data also if a task is still running --->Think if is correct
        self.root = py_trees.composites.Parallel(name="Parallel - Root", policy=py_trees.common.ParallelPolicy.SuccessOnOne())

        gather_data_behaviour = py_trees.meta.create_behaviour_from_function(gather_data)()
        gather_data_behaviour.data=self.data
        gather_data_behaviour.blackboard = py_trees.blackboard.Client(name="Client gather_data")
        gather_data_behaviour.blackboard.register_key(key="steering", access=py_trees.common.Access.WRITE)
        gather_data_behaviour.blackboard.register_key(key="speed", access=py_trees.common.Access.WRITE)
        gather_data_behaviour.blackboard.register_key(key="/signal/stop", access=py_trees.common.Access.WRITE)
        gather_data_behaviour.blackboard.register_key(key="/signal/parking", access=py_trees.common.Access.WRITE)
        gather_data_behaviour.blackboard.register_key(key="/signal/priority", access=py_trees.common.Access.WRITE)
        gather_data_behaviour.blackboard.register_key(key="/signal/crosswalk", access=py_trees.common.Access.WRITE)
        gather_data_behaviour.blackboard.register_key(key="/signal/highway_entrance", access=py_trees.common.Access.WRITE)
        gather_data_behaviour.blackboard.register_key(key="/signal/one_way", access=py_trees.common.Access.WRITE)
        gather_data_behaviour.blackboard.register_key(key="/signal/highway_exit", access=py_trees.common.Access.WRITE)
        gather_data_behaviour.blackboard.register_key(key="/signal/roundabout", access=py_trees.common.Access.WRITE)
        gather_data_behaviour.blackboard.register_key(key="/signal/no_entry", access=py_trees.common.Access.WRITE)
        gather_data_behaviour.blackboard.register_key(key="/signal/horizontal/type", access=py_trees.common.Access.WRITE)
        gather_data_behaviour.blackboard.register_key(key="/position/x", access=py_trees.common.Access.WRITE)
        gather_data_behaviour.blackboard.register_key(key="/position/y", access=py_trees.common.Access.WRITE)
        gather_data_behaviour.blackboard.register_key(key="/position/z", access=py_trees.common.Access.WRITE)
        gather_data_behaviour.blackboard.register_key(key="lane", access=py_trees.common.Access.WRITE)

        crosswalk_sequence = py_trees.composites.Sequence("Sequence - Crosswalk")
        check_crosswalk_sign = py_trees.behaviours.CheckBlackboardVariableValue(
            name="Check Crosswalk sign",
            check=py_trees.common.ComparisonExpression(
                variable="/signal/crosswalk",
                value=True,
                operator=operator.eq
            )
        )
        crosswalk_behaviour = py_trees.meta.create_behaviour_from_function(drive_trough_crosswalk)()
        crosswalk_behaviour.blackboard = py_trees.blackboard.Client(name="Client drive trough crosswalk")
        crosswalk_behaviour.blackboard.register_key(key="steering", access=py_trees.common.Access.READ)
        crosswalk_behaviour.car=self.car

        def checkIfCrosswalkFinished(blackboard):
            print(blackboard.signal.crosswalk)
            return blackboard.signal.crosswalk

        crosswalk_eternal_guard = py_trees.decorators.EternalGuard(
            name="Eternal Guard - Is Crosswalk finished?",
            condition=checkIfCrosswalkFinished,
            blackboard_keys = {"/signal/crosswalk"},
            child=crosswalk_behaviour
        )
        crosswalk_sequence.add_children([check_crosswalk_sign, crosswalk_eternal_guard])

        drive_behaviour = py_trees.meta.create_behaviour_from_function(drive)()
        drive_behaviour.blackboard = py_trees.blackboard.Client(name="Client drive")
        drive_behaviour.blackboard.register_key(key="steering", access=py_trees.common.Access.READ)
        drive_behaviour.blackboard.register_key(key="speed", access=py_trees.common.Access.READ)
        drive_behaviour.blackboard.register_key(key="lane", access=py_trees.common.Access.READ)
        drive_behaviour.car=self.car
        #self.root.add_child(drive_behaviour)
        low_level_selector = py_trees.composites.Selector("Selector - LowLevelStreet")
        low_level_selector.add_children([crosswalk_sequence, drive_behaviour])
        self.root.add_children([gather_data_behaviour, low_level_selector])

        #NOTE CHECK EITHER_Or to select task directly from selector. Good idiom to write less code

    def start(self):
        #CRITICAL
        #ERROR
        #WARNING
        #INFO
        #DEBUG
        #NOTSET
        py_trees.logging.level = py_trees.logging.Level.DEBUG
        self.create_root()
        py_trees.display.render_dot_tree(self.root)
        # Far eseguire il file come se fosse eseguito da terminale
        os.system("rosrun startup_package horizontal_line.py &")
        os.system("rosrun startup_package ParticleFilter.py &")
        os.system("rosrun startup_package PidControl.py &")
        os.system("rosrun startup_package traffic.py &")
        os.system("rosrun startup_package kalman.py &")

        self.subscribe("HorizontalLine", "horizontal_line")
        self.subscribe("StreetLane", "street_lines")
        self.subscribe("PidControlValues", "velocity_steer")
        self.subscribe("Sign", "sign")
        self.subscribe("Kalman", "position", Vector3)

        ########  TO REMOVE ##################
        #pub = rospy.Publisher("REMAIN_LEFT", String, queue_size=10)
        #pub.publish("Normal")

        ########  TO REMOVE ##################

        #state_machine=True

        #stateMachineSteer = StateMachineSteer()
        #stateMachineVelocity = StateMachineVelocity()

        #old_state_steer    = "None"
        #old_state_velocity = "None"
        #old_state_sign = "None"

        #self.speed = 0.22
        #self.steering = 0
        #current_speed = 0.22   # For incrementing or decrementing to reach a target

        #print_msg = ""
        # Wait 20 seconds for the processes to actually start
        #sleep(10)

        #data_state_machine={}
        while not rospy.is_shutdown():
            i=0
            try:
                #print("\n--------- Tick {0} ---------\n".format(i))
                self.root.tick_once()
                #print("\n")
                #print(py_trees.display.unicode_tree(root=self.root, show_status=True))
                sleep(1.0)
            except KeyboardInterrupt:
                break
            #print("tick")
            #if(state_machine):
            #    state_steer="Steady"
            #    data_state_machine["moving"] = True
            #    data_state_machine["horizontal_line"] = "" # Fix it as safe, otherwise there might be a problem if it's missing
                # if ("horizontal_line" in self.data):
                #     data_state_machine["horizontal_line"] = self.data["horizontal_line"]
                #
                # data_state_machine["turning"] = False
                #

                #
                # state_steer=stateMachineSteer.runOneStep(data_state_machine)
                # if(state_steer=="Steady"):
                #     self.steering = 0
                # elif(state_steer=="OnLane" or state_steer=="OnCrosswalk"):
                #     if("velocity_steer" in self.data):
                #         velocity_steer = json.loads(self.data["velocity_steer"])
                #         self.steering = -velocity_steer["steer"]
                # elif(state_steer=="NearCrossroad"):
                #     if("velocity_steer" in self.data):
                #         velocity_steer = json.loads(self.data["velocity_steer"])
                #         self.steering = -velocity_steer["steer"]
                # elif(state_steer=="OnCrossroad"):

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

            #
            #         #self.steering = self.turn('right')
            #         if(len(self.directions)):
            #             self.turn()
            #         else:
            #             self.speed = 0
            #             data_state_machine["moving"] = "False"
            #             print("End of Simulation Round")
            #     #pritn(state_steer +"     " +state_velocity)
            #
            #     data_state_machine["state_steer"] = state_steer
            #     state_velocity=stateMachineVelocity.runOneStep(data_state_machine)
            #     if(state_velocity=="OnSteady"):
            #         self.speed = 0
            #     elif(state_velocity=="Slow" or state_velocity=="Parking"):
            #         if(current_speed > 0.2):
            #             self.speed = min(0.1, current_speed-0.1)
            #         else:
            #             self.speed = max(0.1, current_speed+0.1)
            #     elif(state_velocity=="Fast"):
            #         if(current_speed > 0.8):
            #             self.speed = min(0.3, current_speed-0.05)
            #         else:
            #             self.speed = max(0.3, current_speed+0.05)
            #     current_speed = self.speed
            # else:
            #     #cv.imshow("Processer", cam.getImage())
            #     #cv.waitKey(1)
            #     # COMMENTED TO TEST THE TURNING PROCEDURE
            #     #print("Sending move with speed "+ str(speed) + " , self.steering " + str(self.steering))
            #     if(self.state == "Straight"):
            #         if("street_lines" in self.data):
            #             lines = json.loads(self.data["street_lines"])
            #             #print(lines[0][0])
            #         #if("sign" in self.data):
            #         #    # lines = json.loads(self.data["sign"])
            #         #    #print("FROM PROCESSER" + self.data["sign"])
            #         if("velocity_steer" in self.data):
            #             velocity_steer = json.loads(self.data["velocity_steer"])
            #             self.steering = -velocity_steer["steer"]
            #
            #     if ("horizontal_line" in self.data):
            #         if(self.data["horizontal_line"] == "Stop"):
            #             direction = "straight"
            #             self.turn(direction)
            #
            # self.car.drive(self.speed, self.steering)
            # if(old_state_steer != state_steer or old_state_velocity != state_velocity):
            #     if state_velocity == 'OnSteady':
            #         print('The car is still.')
            #     elif state_velocity=="Parking":
            #         print("PARKING SIGN detected, going slow")
            #     else:
            #         if state_steer == 'OnLane':
            #             print_msg =  'Car is KEEPING THE LANE'
            #         #elif state_steer == 'OnCrossroad':
            #         #    print_msg = 'Car is NAVIGATING AN INTERSECTION'
            #         #elif state_steer == 'NearCrossroad':
            #         #    print_msg = 'Car is approaching an intersection'
            #         elif state_steer == 'OnCrosswalk':
            #             print_msg = 'Car is near a CROSSWALK'
            #         print(print_msg + " and it is going " + state_velocity + ".")
            #
            #     old_state_steer = state_steer
            #     old_state_velocity = state_velocity
            sleep(1)

        #rospy.spin()

    # TODO Fix the time for the sleep in the manuevres
    # def turn(self):
    #
    #     # Define that you want to turn
    #     self.state = "Turn"
    #     print("Going " + self.directions[0])
    #
    #     # Make sure to STOP at STOP
    #     self.car.drive(0, 0)
    #     sleep(5)
    #     self.car.drive(0.2, 0) # Slow down to be more sure about the time to be waited
    #
    #     # Act based on the decision you want
    #     if(self.directions[0] == 'right'):
    #         sleep(9)
    #         #curve_radius = 66.6 # curve_radius = 66.5 # most common radius in the circuit
    #         #ipotenusa    = math.sqrt(2 * (curve_radius ** 2) )
    #         #steer        = 90 - math.acos(curve_radius/ipotenusa)
    #         steer = 22.5
    #         manuevre = 11.5
    #     elif(self.directions[0] == 'left'):
    #         sleep(13)
    #         #curve_radius = 103.6
    #         #ipotenusa    = math.sqrt(2 * (curve_radius ** 2) )
    #         #steer        = - ( 90 - math.acos(curve_radius/ipotenusa) )
    #         steer = -17.5
    #         manuevre = 16
    #     else:
    #         steer = 0
    #         manuevre = 20
    #
    #     self.index_turning=self.index_turning+1
    #     if(self.index_turning>5):
    #         self.index_turning=-100000000000
    #         pub.publish("LEFT")
    #
    #     # Steer the car accordingly
    #     self.car.drive(0.2, steer ) # Set a known velocity for the manuevre
    #     # Wait for the manuevre to end
    #     sleep(manuevre)
    #     # Update direction
    #     dir = self.directions.pop(0)
    #     #self.directions.append(dir) # Removed to end the simulation when the directions are finished
    #     self.state = "Straight"

def get_current_trajectory_point(lines):
	distance_from_base = 0.6       #it's a percentage
	size = 640 ###### TODO --> PUT IT IN A CONFIG FILE
	dist = []

	if(lines[0] != None):
		dist.append([lines[0][i][0] for i in range(0, len(lines[0]))])
	if(lines[1] != None):
		dist.append([lines[1][i][0]- size/2 for i in range(0, len(lines[1]))])

	if(len(dist)==0):
		return None
	elif(len(dist)==1):
		index = int(distance_from_base * len(dist[0]))
		return dist[0][index]
	else:
		too_near=False
		dist_avg = [(dist[0][i] + dist[1][i])/2 for i in range(0, len(dist[0])) ]
		index = int(distance_from_base * len(dist[0]))
		return dist_avg[index]

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
