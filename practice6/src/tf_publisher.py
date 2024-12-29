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
            # Define the translation and rotation between odom and velodyne
            translation = (1.03, 0.0, 1.85)  # x, y, z offset (example values)
            rotation = tf.transformations.quaternion_from_euler(0, 0, 0)  # Roll, Pitch, Yaw (example values)

            # Broadcast the transformation
            br.sendTransform(
                translation,
                rotation,
                self.pc_time,
                "velodyne",  # Child frame (target frame)
                "ground"       # Parent frame (source frame)
            )

            rate.sleep()
        
    def time_stamp_callback(self, msg):
        self.pc_time = msg.header.stamp


if __name__ == '__main__':
    try:
        tf_publisher()
    except rospy.ROSInterruptException:
        pass