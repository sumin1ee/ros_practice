#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from datetime import datetime

class CurrentTimePublisher:
    def __init__(self):
        rospy.init_node('current_time_publisher', anonymous=True)
        self.pub = rospy.Publisher('current_time', String, queue_size=10)
        self.rate = rospy.Rate(1)
        
        while not rospy.is_shutdown():
            self.publish()
            self.rate.sleep()
            
    def publish(self):
        '''
        TODO: Define str_message as a string that contains the current time
        and publish it to the current_time topic
        '''
        str_message = f'current time: {...}'
        ... # Implement the publishing of the message