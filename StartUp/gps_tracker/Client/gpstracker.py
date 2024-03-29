#!/usr/bin/env python2

# RUN THROUGH after ROS startup_package configuration
# rosrun startup_package gpstracker.py


# Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE

import threading
import server_data
import server_listener
import server_subscriber
import position_listener

import time

import rospy
from geometry_msgs.msg import Twist

import math

class GpsTracker(threading.Thread):
    
    def __init__(self, ID):
        """ GpsTracker targets to connect on the server and to receive the messages, which incorporates 
        the coordinate of the robot on the race track. It has two main state, the setup state and the listening state. 
        In the setup state, it creates the connection with server. It's receiving  the messages from the server in the listening
        state. 

        It's a thread, so can be running parallel with other threads. You can access to the received parameters via 'coor' function.

        Examples
        --------
        Here you can find a simple example, where the GpsTracker are running 10 second:
            | gpstracker = GpsTracker()
            | gpstracker.start()
            | time.sleep(10)
            | gpstracker.stop()
            | gpstracker.join()

        """
        super(GpsTracker, self).__init__()
        #: serverData object with server parameters
        self.__server_data = server_data.ServerData()
        #: discover the parameters of server
        self.__server_listener = server_listener.ServerListener(self.__server_data)
        #: connect to the server
        self.__subscriber = server_subscriber.ServerSubscriber(self.__server_data,ID)
        #: receive and decode the messages from the server
        self.__position_listener = position_listener.PositionListener(self.__server_data)
        
        self.__running = True

    def setup(self):
        """Actualize the server's data and create a new socket with it.
        """
        # Running while it has a valid connection with the server
        while(self.__server_data.socket == None and self.__running):
            # discover the parameters of server
            self.__server_listener.find()
            if self.__server_data.is_new_server and self.__running:
                # connect to the server 
                self.__subscriber.subscribe()
        
    
    def listen(self):
        """ Listening the coordination of robot
        """
        self.__position_listener.listen()

    def run(self):
        while(self.__running):
            self.setup()
            self.listen()
    
    def coor(self):
        """Access to the last receive coordinate
        
        Returns
        -------
        dictionary
            coordinate and timestamp
        """
        return self.__position_listener.coor

    def ID(self):
        return self.__subscriber.ID()
    
    def stop(self):
        """Terminate the thread running.
        """
        self.__running = False
        self.__server_listener.stop()
        self.__position_listener.stop()

if __name__ == '__main__':
    gpstracker = GpsTracker(4)
    gpstracker.start()

    # Define Node
    rospy.init_node("gps_node", anonymous=True)

    #About queue_size http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers
    pub = rospy.Publisher("/rcCar/GPS", Twist, queue_size=10)
    
    time.sleep(5)
    while True:
        try:
            coora = gpstracker.coor()

            print(gpstracker.ID(), coora['timestamp'], coora['coor'][0], coora['coor'][1])

            twist = Twist()

            twist.linear.x = coora['coor'][0]["real"]
            twist.linear.y = coora['coor'][0]["imag"]

            cos_yaw = coora["coor"][1]["real"]
            sin_yaw = coora["coor"][1]["imag"]

            yaw = math.atan2(sin_yaw, cos_yaw)

            twist.angular.z = yaw
            pub.publish(twist)
            time.sleep(1)
        except KeyboardInterrupt:
            break

    gpstracker.stop()

    gpstracker.join()
