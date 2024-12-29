#!/usr/bin/env python3
import rospy

class Subscriber:
    def __init__(self):
        rospy.init_node('subscriber', anonymous=True)
        '''
        TODO: Choose the appropriate message type and topic name
        '''
        rospy.Subscriber(..., ..., self.callback)
        rospy.spin()
    
    def callback(self, data):
        '''
        TODO: Print the data received from the publisher
        '''
        pass

if __name__ == '__main__':
    try:
        Subscriber()
    except rospy.ROSInterruptException:
        pass