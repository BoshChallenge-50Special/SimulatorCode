#!/usr/bin/env python2

import rospy
from std_msgs.msg import String

class Consumer(object):
    def __init__(self):
        super(Consumer, self).__init__()
        #print('Init Vehicle')
        #self.__vehicle = 'this is vehicle'
        self.data={}

    def subscribe(self, publisher_name, variable_name, variable_type=None):
        if(variable_type==None):
            variable_type=String

        rospy.Subscriber(publisher_name, variable_type, self.callback, [variable_name, variable_type])

    def callback(self, data, args):
        #print("callback "+str(data)+"  "+ str(args))
        if(args[1]==String):
            self.data[args[0]] = data.data
        else:
            self.data[args[0]] = data

class Producer(object):
    def __init__(self, node_name):
        super(Producer, self).__init__()
        rospy.init_node(node_name, anonymous=True)
        #self.rate = rospy.Rate(10) # 10hz

    def set_publisher(self, pub_name):
        ### Possibilities to implement multiple publishers

        #About queue_size http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers
        self.pub = rospy.Publisher(pub_name, String, queue_size=10)
		#self.pub = pub

#class Producer_and_Consumer(Producer, Consumer):
#    def __init__(self, node_name):
#        super(Producer_and_Consumer, self).__init__(node_name)

#pc= Producer_and_Consumer("aa")
