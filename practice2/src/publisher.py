#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from datetime import datetime

class Publisher:
    def __init__(self):
        rospy.init_node('publisher', anonymous=True)
        self.pub = rospy.Publisher('publisher', String, queue_size=10)
        self.rate = rospy.Rate(1)
        
        while not rospy.is_shutdown():
            self.publish()
            self.rate.sleep()
            
    def publish(self):
        '''
        TODO: Publish a message to the topic
        the message should be the current time. It can be obtained by calling "datetime.now()" function
        Note that the message should be a string, and it dosen't matter  whether it is a ROS message or not in this case
        '''
        str_message = f'current time: {...}'
        rospy.loginfo(str_message)
        self.pub.publish(str_message)

if __name__ == '__main__':
    try:
        Publisher()
    except rospy.ROSInterruptException:
        pass