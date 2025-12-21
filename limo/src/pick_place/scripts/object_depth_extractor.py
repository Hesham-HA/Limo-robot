#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import Float32MultiArray, Float32


class ObjectDepthExtractor:
    def __init__(self):
        rospy.init_node("object_depth_extractor")

        # Subscriber to find_object_2d output
        rospy.Subscriber("/objects", Float32MultiArray, self.objects_callback)

        # Optional publisher (useful later for navigation)
        self.dist_pub = rospy.Publisher("/object_distance", Float32, queue_size=10)

        rospy.loginfo("Object depth extractor node started.")
        rospy.spin()

    def objects_callback(self, msg):
        # Safety check
        if len(msg.data) < 6:
            rospy.logwarn("Received /objects message with insufficient data.")
            return

        # Extract 3D position (camera frame)
        x = msg.data[3]  # forward distance (meters)
        y = msg.data[4]
        z = msg.data[5]

        # Euclidean distance
        distance = math.sqrt(x**2 + y**2 + z**2)

        rospy.loginfo(
            "Object position (camera frame): X=%.2f m, Y=%.2f m, Z=%.2f m | Distance=%.2f m",
            x, y, z, distance
        )

        # Publish distance
        self.dist_pub.publish(distance)


if __name__ == "__main__":
    try:
        ObjectDepthExtractor()
    except rospy.ROSInterruptException:
        pass
