#!/usr/bin/python
import rospy
import numpy as np
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Path
import tf2_ros
from tf_conversions import transformations as tf


class control:
    def __init__(self):
        self.v_cruising = 1#0.3  # 30 cm/s
        self.w_cruising = np.pi#/3  # 60 deg/s

        # Listener odometry
        self.tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tfBuffer)

        # Velocity publisher
        self.pub = rospy.Publisher(
            '/mobile_base/commands/velocity', Twist, queue_size=10)
        self.sub = rospy.Subscriber("/path", Path, self.follow)

    def follow(self, path):
        waypoints = []
        for pose in path.poses:
            waypoints.append([pose.pose.position.x, pose.pose.position.y])
            
        # Listener loop
        i = 0
        # Errors
        last_lin_error, accu_lin_error = [0, 0]
        last_ang_error, accu_ang_error = [0, 0]

        # rospy.sleep(3.0)
        while not rospy.is_shutdown() and i < len(waypoints):
            # Listener block
            try:
                trans = self.tfBuffer.lookup_transform(
                    'odom', 'base_footprint', rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue

            current_theta = tf.euler_from_quaternion([trans.transform.rotation.x,
                                                      trans.transform.rotation.y,
                                                      trans.transform.rotation.z,
                                                      trans.transform.rotation.w])[2]

            x, y = waypoints[i]

            x_error = x - trans.transform.translation.x
            y_error = y - trans.transform.translation.y

            linear_error = np.sqrt(x_error**2 + y_error**2)
            angular_error = self.normalizeAngle(
                np.arctan2(y_error, x_error) - current_theta)

            kobuki_vel = Twist()

            if abs(linear_error) + abs(angular_error) <= 0.1:
                self.pub.publish(Twist())
                rospy.loginfo('Waypoint achieved: '+str(waypoints[i]))
                i = i + 1
                continue
            else:
                param_names = rospy.get_param_names()
                # Linear
                kpv = rospy.get_param("/PID/Kpv") if "/PID/Kpv" in param_names else 0.004 #0.4
                kiv = rospy.get_param("/PID/Kiv") if "/PID/Kiv" in param_names else 0.004 #0.4
                kdv = rospy.get_param("/PID/Kdv") if "/PID/Kdv" in param_names else 0.010 #10.0
                accu_lin_error += linear_error
                rate_error = linear_error - last_lin_error
                last_lin_error = linear_error
                v = kpv*linear_error + kiv*accu_lin_error + kdv*rate_error

                # Angular
                kpw = rospy.get_param("/PID/Kpw") if "/PID/Kpw" in param_names else 50.7 #3.7
                kiw = rospy.get_param("/PID/Kiw") if "/PID/Kiw" in param_names else 0.001135 #1.2
                kdw = rospy.get_param("/PID/Kdw") if "/PID/Kdw" in param_names else 10.5 #4.0
                accu_ang_error += angular_error
                rate_error = angular_error - last_ang_error
                last_ang_error = angular_error
                w = kpw*angular_error + kiw*accu_ang_error + kdw*rate_error

                kobuki_vel.linear.x = v
                if abs(kobuki_vel.linear.x) > self.v_cruising:
                    kobuki_vel.linear.x = self.v_cruising

                kobuki_vel.angular.z = w
                if abs(kobuki_vel.angular.z) > self.w_cruising:
                    kobuki_vel.angular.z = self.w_cruising * \
                        np.sign(kobuki_vel.angular.z)

            # # print self.path[i]
            # print [trans.transform.translation.x, trans.transform.translation.y]
            # print angular_error
            # print kobuki_vel
            # print
            self.pub.publish(kobuki_vel)

    def normalizeAngle(self, angle):
        while angle < -np.pi:
            angle += 2.0*np.pi
        while angle > np.pi:
            angle -= 2.0*np.pi
        return angle


if __name__ == '__main__':
    rospy.init_node('control', anonymous=True)
    rospy.loginfo("Node init")

    control()

    rospy.spin()
