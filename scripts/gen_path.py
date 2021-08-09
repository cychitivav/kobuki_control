#!/usr/bin/python
import rospy
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class generator:
    def __init__(self, path=[[-5.0, -4.0]]):
        route = np.array(path)

        pub = rospy.Publisher('/path', Path, queue_size=10)

        while not rospy.is_shutdown():
            msg = Path()
            msg.header.frame_id = "map"
            msg.header.stamp = rospy.Time.now()

            for wp in route:
                pose = PoseStamped()

                pose.pose.position.x = wp[0]
                pose.pose.position.y = wp[1]

                msg.poses.append(pose)

            pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('path_gen', anonymous=True)

    path = [[0.0, 0.0],
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
            #inicio de primera trayectoria extra

    path2 =[[0.0, 0.0],
            [1.0, 1.0],
            [3.0, 1.0],
            [-1.0, -0.5],
            [2.0, 3.0],
            [5.0, -1.0],
            [0.0, 0.5]]
            #inicio de segunda trayectoria extra

    path3 =[[0.0, 0.0],
            [-2.0, -2.5],
            [-1.0, -3.5],
            [0.0, 3.0],
            [5.0, -1.0],
            [1.0, -0.5],
            [2.0, -1.0]]

    rospy.loginfo("Node init")

    generator(path=path)

    rospy.spin()
