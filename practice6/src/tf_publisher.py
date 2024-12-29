#!/usr/bin/env python3
 
import rospy
import tf
from sensor_msgs.msg import PointCloud2

class tf_publisher:
    def __init__(self):
        rospy.init_node('tf_publisher', anonymous=True)
        
        self.sub = rospy.Subscriber('velodyne_points', PointCloud2, self.time_stamp_callback)
        self.pc_time = 0.0
        
        br = tf.TransformBroadcaster()

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            '''
            TODO: Define the translation and rotation between ground and LiDAR
            LiDAR is above 1.85m from the ground and far forward 1.03 from the rear wheel with no rotation
            '''
            translation = (..., ..., ...) # (x, y, z)
            rotation = tf.transformations.quaternion_from_euler(..., ..., ...) # (roll, pitch, yaw)

            '''
            TODO: Broadcast the transformation
            '''
            br.sendTransform(
                translation,
                rotation,
                self.pc_time,
                "...",  # Child frame (target frame), Please check the frame name from rostopic info /velodyne_points
                "..."   # Parent frame (source frame)
            )

            rate.sleep()
        
    def time_stamp_callback(self, msg):
        self.pc_time = msg.header.stamp


if __name__ == '__main__':
    try:
        tf_publisher()
    except rospy.ROSInterruptException:
        pass