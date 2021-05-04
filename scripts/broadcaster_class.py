#!/usr/bin/python

import rospy

import numpy as np
from rospy.numpy_msg import numpy_msg
from tf.transformations import rotation_matrix

import tf2_ros
import tf_conversions

from geometry_msgs.msg import TransformStamped
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker


class Broadcaster:

    def __init__(self):

        # Atributos
        # Para realizar un broadcast
        self.broadcts = tf2_ros.TransformBroadcaster()
        self.transform = TransformStamped()

        # Para realizar la escucha "listener"
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        # Subscribers
        rospy.Subscriber("/ar_pose_marker", numpy_msg(AlvarMarkers),
                         self.Marker_Callback, queue_size=10)

    #--------------------------------------------------------------------------------------#
    # Callback o interrupcion

    def Marker_Callback(self, marker_info):

        #--------------------------------------------------------------------------------#
        #--------------------------------------------------------------------------------#
        #--------------------------------------------------------------------------------#
        # MTH from base to camera

        try:
            trans_mobbase_cam = self.tfBuffer.lookup_transform(
                "base_footprint", "camera_depth_frame", rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Error trying to look for transform")
            return

        # Creater quaternion vector
        quat_odom_cam = np.array([trans_mobbase_cam.transform.rotation.x,
                                  trans_mobbase_cam.transform.rotation.y,
                                  trans_mobbase_cam.transform.rotation.z,
                                  trans_mobbase_cam.transform.rotation.w])

        # MTH with position vector equal to zero
        rt_mat_odom_cam = tf_conversions.transformations.quaternion_matrix(
            quat_odom_cam)

        # Add position vector to MTH
        MTH_odom_cam = rt_mat_odom_cam.copy()
        MTH_odom_cam[0, 3] = trans_mobbase_cam.transform.translation.x
        MTH_odom_cam[1, 3] = trans_mobbase_cam.transform.translation.y
        MTH_odom_cam[2, 3] = trans_mobbase_cam.transform.translation.z

        print("MTH_odom_cam")
        print(MTH_odom_cam)
        print("")
        #--------------------------------------------------------------------------------#
        #--------------------------------------------------------------------------------#
        #--------------------------------------------------------------------------------#
        # MTH from camera to Alvar marker

        singleMarker = marker_info.markers[0]

        # Creater quaternion vector
        quat_cam_marker = np.array([singleMarker.pose.pose.orientation.x,
                                    singleMarker.pose.pose.orientation.y,
                                    singleMarker.pose.pose.orientation.z,
                                    singleMarker.pose.pose.orientation.w])

        # MTH with position vector equal to zero
        rt_mat_cam_marker = tf_conversions.transformations.quaternion_matrix(
            quat_cam_marker)

        # Add position vector to MTH
        MTH_cam_marker = rt_mat_cam_marker.copy()
        MTH_cam_marker[0, 3] = singleMarker.pose.pose.position.x
        MTH_cam_marker[1, 3] = singleMarker.pose.pose.position.y
        MTH_cam_marker[2, 3] = singleMarker.pose.pose.position.z

        print("MTH_cam_marker")
        print(MTH_cam_marker)
        print("")

        #--------------------------------------------------------------------------------#
        #--------------------------------------------------------------------------------#
        #--------------------------------------------------------------------------------#
        # Comparison between MTH obtained directly from ROS a the one we created using
        # the two previos MTHs

        # Transform obtained from ROS
        try:
            trans_base_marker = self.tfBuffer.lookup_transform(
                "base_footprint", "ar_marker_5", rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Error trying to look for transform")
            return
        quat_from_ROS = np.array([trans_base_marker.transform.rotation.x,
                                  trans_base_marker.transform.rotation.y,
                                  trans_base_marker.transform.rotation.z,
                                  trans_base_marker.transform.rotation.w])
        rt_mat_from_ROS = tf_conversions.transformations.quaternion_matrix(
            quat_from_ROS)
        MTH_from_ROS = rt_mat_from_ROS.copy()
        MTH_from_ROS[0, 3] = trans_base_marker.transform.translation.x
        MTH_from_ROS[1, 3] = trans_base_marker.transform.translation.y
        MTH_from_ROS[2, 3] = trans_base_marker.transform.translation.z

        print("MTH obtained directly from ROS")
        print(MTH_from_ROS)
        print("")

        MTH_from_twoSteps = np.matmul(MTH_odom_cam, MTH_cam_marker)

        print("MTH obtained by the multiplication of MTHs")
        print(MTH_from_twoSteps)
        print("")

        # Error

        print("Error between the two")
        print(MTH_from_ROS - MTH_from_twoSteps)
        print("")

        #--------------------------------------------------------------------------------#
        #--------------------------------------------------------------------------------#
        #--------------------------------------------------------------------------------#
        # Broadcasting of a MTH from the marker to a task frame 0.5m to the right and
        # 0.2m up of it, and with a rotation of 112.5 degrees around the Z-axis

        # It is needed to stamp (associate a time) to the MTH
        self.transform.header.stamp = rospy.Time.now()

        # It is needed to determine which task frame is the parent (base)
        # and the name of the new TF
        self.transform.header.frame_id = "ar_marker_5"  # Parent or base frame
        # Name of new TF, created by this MTH
        self.transform.child_frame_id = "custom_tf"

        # Translation part of MTH
        self.transform.transform.translation.x = 0.5
        self.transform.transform.translation.y = 0.2
        self.transform.transform.translation.z = 0.0

        # Rotation part of MTH
        # Even though we only need the rotation matrix
        des_rotation = np.identity(4)
        # to use the library of transformations it is
        # needed to define a MTH so we create one with
        # translational part equal to zero
        angle2rotate = 5*np.math.pi/8
        des_rotation[0, 0] = np.math.cos(angle2rotate)
        des_rotation[0, 1] = -np.math.sin(angle2rotate)
        des_rotation[1, 0] = np.math.sin(angle2rotate)
        des_rotation[1, 1] = np.math.cos(angle2rotate)
        quat_desired = tf_conversions.transformations.quaternion_from_matrix(
            des_rotation)

        self.transform.transform.rotation.x = quat_desired[0]
        self.transform.transform.rotation.y = quat_desired[1]
        self.transform.transform.rotation.z = quat_desired[2]
        self.transform.transform.rotation.w = quat_desired[3]

        self.broadcts.sendTransform(self.transform)


# Init of program
if __name__ == '__main__':

    rospy.init_node('tf2_broadcaster', anonymous=True)

    rospy.loginfo("Node init")

    Broadcaster()

    rospy.spin()