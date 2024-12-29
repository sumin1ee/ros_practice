#!/usr/bin/env python3

import rospy
from std_msgs.msg import ...
from practice3.msg import MyBoundingBox

class BBoxParser:
    def __init__(self):
        rospy.init_node('bbox_parser', anonymous=True)
        rospy.Publisher('bbox', MyBoundingBox, queue_size=10)
        
        '''
        TODO: Check the topic name and data type from "rostopic list" or "rostopic info /topic_name" and define the appropriate subscriber
        '''
        rospy.Subscriber("...", ..., self.center_x_callback)
        rospy.Subscriber("...", ..., self.center_y_callback)
        rospy.Subscriber("...", ..., self.width_callback)
        rospy.Subscriber("...", ..., self.height_callback)
        
        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown():
            self.publish_bbox()
            rate.sleep()
            
    def publish_bbox(self):
        '''
        TODO: Publish the bounding box information to the bbox topic
        '''
        bbox = MyBoundingBox()
        ... = self.center_x
        ... = self.center_y
        ... = self.width
        ... = self.height
        self.pub.publish(bbox)
    
    def center_x_callback(self, data):
        self.center_x = data.data
    
    def center_y_callback(self, data):
        self.center_y = data.data
    
    def width_callback(self, data):
        self.width = data.data
        
    def height_callback(self, data):
        self.height = data.data
        


if __name__ == '__main__':
    try:
        BBoxParser()
    except rospy.ROSInterruptException:
        pass