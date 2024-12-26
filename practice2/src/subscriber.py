#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

class Subscriber:
    def __init__(self):
        rospy.init_node('subscriber', anonymous=True)
        rospy.Subscriber('publisher', String, self.callback)
        rospy.spin()
    
    def callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

if __name__ == '__main__':
    try:
        Subscriber()
    except rospy.ROSInterruptException:
        pass