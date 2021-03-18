#!/usr/bin/env python2

import rospy
from std_msgs.msg import String

class Consumer(object):
    def __init__(self):
        super(Consumer, self).__init__()
        #print('Init Vehicle')
        #self.__vehicle = 'this is vehicle'
        self.data={}

    def subscribe(self, publisher_name, variable_name):
        #rospy.Subscriber("HorizontalLine", String, self.callback, variable_name)
        rospy.Subscriber(publisher_name, String, self.callback, variable_name)
        #print("subscribe")

    #def HorizontalCheck(self, data):
    #if(data.data != self.HorizontalState):
    #		self.HorizontalState = data.data
    #		rospy.loginfo('Horizontal: %s', data.data)

    def callback(self, data, args):
        #print("callback "+str(data)+"  "+ str(args))
        self.data[args] = data.data

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
