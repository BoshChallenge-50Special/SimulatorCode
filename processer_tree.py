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

from bfmclib.controller_p import Controller
#from bfmclib.trafficlight_s import TLColor, TLLabel, TrafficLight

#import cv2 as cv
from time import sleep
import rospy

import math

from SimulatorCode.templates import Consumer
from Control.Path import PathFollow
from Localization.GraphMap import GraphMap

import Control.CarModel as car_model

class Processer(Consumer):

    def __init__(self):
        super(Processer, self).__init__()
        rospy.init_node("PROCESSOR", anonymous=True)
        self.car=Controller()

        # Salva i dati per la macchina nel Processer
        self.speed = 0
        self.steering = 0
        self.state = "Straight"

        #self.pid_control=PidControl(verbose=True)
        #self.index_turning = 0
        self.path_follow=PathFollow(vicinity_threshold=0.5, verbose=True)

        #self.directions=["straight", "left", "right", "left", "straight"]



    def create_root(self):
        def drive(self):
            #Evaluating trajectory with
            #if(len(self.blackboard.lane)>0 and self.blackboard.exists("position.x")):
            obj_point_correction=[]
            if(self.blackboard.exists("/lane")):
                # It return the center point in the street based on the lines of the lane.
                x, y, self.street_size_px = get_current_trajectory_point(self.blackboard.lane, self.street_size_px)
                if(x != None):
                    # Point objective is the center of the street based on the lane
                    POV_obj_x, POV_obj_y = car_model.camera_to_car_ref(x,y)
                    # The real point where the car is going based on the center of the camera picture
                    POV_real_x, POV_real_y = car_model.camera_to_car_ref(320,y)
                    obj_point_correction=[POV_obj_x, POV_obj_y]
                    error = POV_real_x-POV_obj_x #Error in reference frame of the car
                    print("DRIVEEEEE")
                    print(error)
            """"actual_trajectory= get_current_trajectory_point(self.blackboard.lane)
                dist_camera_view_point_center = 0.2/math.sin(0.2617) #ipotenusa, 0.2 is the high of the camera on the street
                dist_car_view_point_center =  math.cos(0.2617)*dist_camera_view_point

                vertical_fov= 1.085594795*480/640
                dist_camera_view_point_center = 0.2/math.sin(0.2617) #ipotenusa, 0.2 is the high of the camera on the street
                dist_car_view_point_center =  math.cos(0.2617)*dist_camera_view_point


                print(self.blackboard)
                print("Error from Lane:" + str(actual_trajectory))
                trajectory_point_in_world = car_model.car_to_world(self.blackboard.position.x,
                                                         self.blackboard.position.y,
                                                         self.blackboard.position.z,
                                                         [distance_point_of_interest, actual_trajectory])
                #Fixed
                objective_point_world = car_model.car_to_world(self.blackboard.position.x,
                                                         self.blackboard.position.y,
                                                         self.blackboard.position.z,
                                                         [distance_point_of_interest, 0])


                #(x_car+cos(angolo_car)*distanza_visiva, y_car+sin(angolo_car)*distanza_visiva)
                #trajectory.append()
                car_position_array=[self.blackboard.position.x,self.blackboard.position.y,self.blackboard.position.z]
            """


            if(self.blackboard.exists("/position/x")):
                car_position_array=[self.blackboard.position.x,self.blackboard.position.y,self.blackboard.position.z]
                angle_trajectory = self.path_follow.run(car_position_array, self.blackboard.path)

                if(len(obj_point_correction)>0):
                    angle_correction = self.path_follow.get_angle(car_position_array, obj_point_correction)

                    # Final angle is the average between the trajectory angle from the Map
                    #           and the correction angle from the lane
                    print("angle_trajectory : " + str(angle_trajectory) + "  angle_correction : " + str(angle_correction))
                    angle =  (angle_trajectory + angle_correction)/2
                    angle = angle_trajectory
                else:
                    angle = angle_trajectory
                #TO CHECK IF PATH IS REDUCED OR IT SHOULD BE RETURNED
                self.car.drive(self.blackboard.speed, angle)
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.FAILURE

        def drive_trough_crosswalk(self):
            if(self.blackboard.exists("/position/x")):
                car_position_array=[self.blackboard.position.x,self.blackboard.position.y,self.blackboard.position.z]
                angle = self.path_follow.run(car_position_array, self.blackboard.path)

                #TO CHECK IF PATH IS REDUCED OR IT SHOULD BE RETURNED
                self.car.drive(0.05, angle)
            return py_trees.common.Status.RUNNING

        def gather_data(self):

            self.blackboard.steering = 0
            self.blackboard.speed = 0.2

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

            #print(self.blackboard)

            return py_trees.common.Status.SUCCESS


        fileMap = './src/startup_package/src/SimulatorCode/Localization/Competition_track.graphml'
        Graph = GraphMap(fileMap)
        path, length = Graph.get_path_coor("66", "88")

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
        gather_data_behaviour.blackboard.register_key(key="path", access=py_trees.common.Access.WRITE)
        gather_data_behaviour.blackboard.path = path

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
        drive_behaviour.blackboard.register_key(key="/position/x", access=py_trees.common.Access.READ)
        drive_behaviour.blackboard.register_key(key="/position/y", access=py_trees.common.Access.READ)
        drive_behaviour.blackboard.register_key(key="/position/z", access=py_trees.common.Access.READ)
        drive_behaviour.blackboard.register_key(key="path", access=py_trees.common.Access.READ)
        drive_behaviour.car=self.car
        drive_behaviour.path_follow=self.path_follow
        drive_behaviour.street_size_px=None
        #drive_behaviour.pid_control=self.pid_control
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
        os.system("rosrun startup_package traffic.py &")
        os.system("rosrun startup_package kalman.py &")

        self.subscribe("HorizontalLine", "horizontal_line")
        self.subscribe("StreetLane", "street_lines")
        self.subscribe("Sign", "sign")
        self.subscribe("Kalman", "position", Vector3)

        #self.speed = 0.22
        #current_speed = 0.22   # For incrementing or decrementing to reach a target

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

            sleep(1)

#
# lines --> from Lane tracking
# street_size --> Firs time is none, later on it will be returned by this method based on the lane
#                           and on the distance from the bottom of the image choosed
#
def get_current_trajectory_point(lines, street_size=None):
    distance_from_base = 0.6       #it's a percentage
    size = 640  ###### TODO --> PUT IT IN A CONFIG FILE
    dist = []
    y=0

    if(lines[0] != None):
        # distance from left border to left_line
        dist.append([lines[0][i][0] for i in range(0, len(lines[0]))])
    if(lines[1] != None):
        # distance(negative) from right border to right_line
        #dist.append([lines[1][i][0]- size/2 for i in range(0, len(lines[1]))])

        # distance from left border of the right_line
        dist.append([lines[1][i][0]+ size/2 for i in range(0, len(lines[1]))])

    if(len(dist)==0):
        return None, None, None
    index = int(distance_from_base * len(dist[0]))

    if(lines[0] != None):
        y=lines[0][index][1]
    elif(lines[1] != None):
        y=lines[1][index][1]

    if(len(dist)==1):
        if(street_size!=None):
            if(dist[0][index]>0):
                return dist[0][index]+street_size/2, y, street_size
            else:
                return dist[0][index]-street_size/2, y, street_size
        else:
            return None, y, street_size
    else:
        too_near=False
        #dist_avg = [(dist[0][i] + dist[1][i])/2 for i in range(0, len(dist[0])) ]
        cental_point = [(dist[0][i] + dist[1][i])/2 for i in range(0, len(dist[0])) ]
        street_size = dist[1][index] - dist[0][index]
        return cental_point[index], y, street_size


if __name__ == '__main__':
    try:
        print("Loaded processer")

        processer=Processer()
        processer.start()

    except Exception as e:
        print("Error in Processer")
        print(e)
        print(traceback.print_exc())
