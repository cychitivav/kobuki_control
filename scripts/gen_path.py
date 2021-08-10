#!/usr/bin/python
import rospy
from tf.transformations import quaternion_from_euler
import tf2_ros 
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from tf_conversions import transformations as tf


class generator:
    def __init__(self, path=[[-5.0, -4.0]]):
        route = np.array(path)

        pub = rospy.Publisher('/path', Path, queue_size=10)

        self.pub_trans()

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


    def pub_trans(self):
        trans=TransformStamped()
        trans.header.stamp=rospy.Time.now()
        trans.header.frame_id= "map"
        trans.child_frame_id="odom"

        yaw_0 = rospy.get_param("yaw")
        x_0 = rospy.get_param("x")
        y_0 = rospy.get_param("y")

        trans.transform.translation.x = x_0
        trans.transform.translation.y = y_0

        quat=tf.quaternion_from_euler(0.0,0.0,yaw_0)
        trans.transform.rotation.x = quat[0]
        trans.transform.rotation.y = quat[1]
        trans.transform.rotation.z = quat[2]
        trans.transform.rotation.w = quat[3]

        broadcaster=tf2_ros.StaticTransformBroadcaster()
        broadcaster.sendTransform(trans)

if __name__ == '__main__':
    rospy.init_node('path_gen', anonymous=True)

    path = [[-3.5, 0.0],
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

    path2 =[[1.0, 1.0],
            [3.0, 1.0],
            [-1.0, -0.5],
            [2.0, 3.0],
            [5.0, -1.0],
            [0.0, 0.5]]
            #inicio de segunda trayectoria extra

    path3 =[[-2.0, -2.5],
            [-1.0, -3.5],
            [0.0, 3.0],
            [5.0, -1.0],
            [1.0, -0.5],
            [2.0, -1.0]]

    rospy.loginfo("Node init")

    generator(path=path)

    rospy.spin()
