#!/usr/bin/python
import rospy
import numpy as np
from tf.transformations import rotation_matrix
from geometry_msgs.msg import Twist
import tf2_ros
from tf_conversions import transformations as tf
from geometry_msgs.msg import TransformStamped


class control:
    def __init__(self, path):
        self.path = self.calc_angle(np.array(path), [0, 0, 0])
        self.v_cruising = 0.3  # 30 cm/s
        self.w_cruising = np.pi/3  # 60 deg/s

        # Listener odometry
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)

        # Velocity publisher
        #self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pub = rospy.Publisher(
            '/mobile_base/commands/velocity', Twist, queue_size=10)

        # Frecuency
        f = 1.0  # Hz
        rate = rospy.Rate(f)
        # Listener loop
        i = 0
        # Errors
        last_x_error, cum_x_error = [0, 0]
        last_y_error, cum_y_error = [0, 0]
        last_theta_error, cum_theta_error = [0, 0]
        # Time while start gazebo
        rospy.sleep(2.0)
        while not rospy.is_shutdown() and i < len(self.path):
            # Listener block
            try:
                trans = tfBuffer.lookup_transform(
                    'odom', 'base_footprint', rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rate.sleep()
                continue

            current_theta = tf.euler_from_quaternion([trans.transform.rotation.x,
                                                      trans.transform.rotation.y,
                                                      trans.transform.rotation.z,
                                                      trans.transform.rotation.w])[2]

            x_error = trans.transform.translation.x - self.path[i, 0]
            y_error = trans.transform.translation.y - self.path[i, 1]
            theta_error = current_theta - self.path[i, 2]
            if theta_error >= np.pi:
                theta_error -= 2.0*np.pi
            elif theta_error < -np.pi:
                theta_error += 2.0*np.pi

            kobuki_vel = Twist()

            if abs(x_error) + abs(y_error) + abs(theta_error) <= 0.01:
                self.pub.publish(Twist())
                print('Point:', path[i])
                i = i + 1
                rospy.sleep(0.5)
                continue
            else:
                # X
                kp, ki, kd = [0.4, 0.0, 0.0]
                cum_x_error += (x_error / f)
                rate_error = (x_error - last_x_error) * f
                last_x_error = x_error
                vel_x = kp*x_error + ki*cum_x_error + kd*rate_error

                # Y
                kp, ki, kd = [0.4, 0.0, 0.0]
                cum_y_error += (y_error / f)
                rate_error = (y_error - last_y_error) * f
                last_y_error = y_error
                vel_y = kp*y_error + ki*cum_y_error + kd*rate_error

                # THETA
                kp, ki, kd = [10.0, 1.0, 0.0]
                cum_theta_error += (theta_error / f)
                rate_error = (theta_error - last_theta_error) * f
                last_theta_error = theta_error
                omega = kp*theta_error + ki*cum_theta_error + kd*rate_error

                kobuki_vel.linear.x = np.sqrt(vel_x**2 + vel_y**2)
                if abs(kobuki_vel.linear.x) > self.v_cruising:
                    kobuki_vel.linear.x = self.v_cruising

                kobuki_vel.angular.z = omega - np.arctan2(vel_y, vel_x)
                if abs(kobuki_vel.angular.z) > self.w_cruising:
                    kobuki_vel.angular.z = self.w_cruising * \
                        kobuki_vel.angular.z/abs(kobuki_vel.angular.z)

            kobuki_vel.linear.x = 0
            kobuki_vel.angular.z = 1

            print(self.path[i])
            print([trans.transform.translation.x, trans.transform.translation.y, current_theta])
            print(kobuki_vel)
            print()
            self.pub.publish(kobuki_vel)
            rate.sleep()

    def calc_angle(self, path, current_pose):
        angle = []
        n = len(path)

        for i in range(n):
            if i == 0:
                x_distance = path[i+1, 0] - current_pose[0]
                y_distance = path[i+1, 1] - current_pose[1]
                angle.append([np.arctan2(x_distance, y_distance)])
            elif i == n-1:
                x_distance = path[i, 0] - path[i-1, 0]
                y_distance = path[i, 1] - path[i-1, 1]
                angle.append([np.arctan2(x_distance, y_distance)])
            else:
                x_distance = path[i+1, 0] - path[i-1, 0]
                y_distance = path[i+1, 1] - path[i-1, 1]
                angle.append([np.arctan2(x_distance, y_distance)])

        return np.append(path, angle, axis=1)


if __name__ == '__main__':
    rospy.init_node('control', anonymous=True)
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

    rospy.loginfo("Node init")

    control(path=path)

    rospy.spin()
