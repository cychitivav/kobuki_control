#!/usr/bin/python

import rospy

import numpy as np
from rospy.numpy_msg import numpy_msg
import tf2_ros
from tf_conversions import transformations as tf
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float64

class sensor:
    def __init__(self):
        self.r = 0.0352
        self.l = 0.115
        self.Dphi = np.array([0,0])

         # Subscribers
        self.sub_wheel_right = rospy.Subscriber('/right_wheel_ctrl/command', Float64, self.update_phi_right)
        self.sub_wheel_left = rospy.Subscriber('/left_wheel_ctrl/command', Float64, self.update_phi_left)

        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.transform = TransformStamped()
        self.transform.header.frame_id = "odom" 
        self.transform.child_frame_id = "base_footprint"
        self.yaw = 0

        f = 120
        r = rospy.Rate(f) 
        while not rospy.is_shutdown():
            self.update_odom(1/f)
            r.sleep()

    def update_phi_left(self, phi_L):
        self.Dphi[0] = phi_L.data
        
    def update_phi_right(self, phi_R):
        self.Dphi[1] = phi_R.data

    def update_odom(self, dt):
        self.transform.header.stamp = rospy.Time.now()

        v = self.r*sum(self.Dphi)
        w = self.r*np.subtract(self.Dphi[0], self.Dphi[1])/(2*self.l)

        self.transform.transform.translation.x = self.transform.transform.translation.x + dt*v*np.cos(self.yaw)
        self.transform.transform.translation.y = self.transform.transform.translation.y + dt*v*np.sin(self.yaw)
        
        self.yaw = self.yaw + dt * w
        quaternion = tf.quaternion_from_euler(0, 0, self.yaw)

        self.transform.transform.rotation.x = quaternion[0]
        self.transform.transform.rotation.y = quaternion[1]
        self.transform.transform.rotation.z = quaternion[2]
        self.transform.transform.rotation.w = quaternion[3]

        self.broadcaster.sendTransform(self.transform)

if __name__ == '__main__':
    rospy.init_node('odom_bradcaster', anonymous=True)

    sensor()

    rospy.spin()