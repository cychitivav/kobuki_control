#!/usr/bin/python
import rospy
import tf2_ros 
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf_conversions import transformations as tf


class generator:
    def __init__(self, path=[[-3.5, 0.0]]):
        pub = rospy.Publisher('/plan', Path, queue_size=10)

        rospy.sleep(3.0)
        
        msg = Path()
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()

        for wp in path:
            pose = PoseStamped()

            pose.pose.position.x = wp[0]
            pose.pose.position.y = wp[1]

            msg.poses.append(pose)

        pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('path_gen', anonymous=True)

    path = [#[0.0, 0.0],
            [-3.5, 0.0],
            [-3.5, 3.5],
            [1.5, 3.5],
            [1.5, -1.5],
            [3.5, -1.5],
            [3.5, -8.0],
            [-2.5, -8.0],
            [-2.5, -5.5],
            [1.5, -5.5],
            [1.5, -3.5],
            [-1.0, -3.5]]

    path2 =[[1.0, 1.0],
            [3.0, 1.0],
            [-1.0, -0.5],
            [2.0, 3.0],
            [5.0, -1.0],
            [0.0, 0.5]]

    path3 =[[-2.0, -2.5],
            [-1.0, -3.5],
            [0.0, 3.0],
            [5.0, -1.0],
            [0.0, 0.0],
            [1.0, -0.5],
            [2.0, -1.0]]

    rospy.loginfo("Node init")

    generator(path=path)

    rospy.spin()
