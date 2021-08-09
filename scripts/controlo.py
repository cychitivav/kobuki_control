#!/usr/bin/python
import rospy
import numpy as np
from geometry_msgs.msg import Twist
import tf2_ros
from tf_conversions import transformations as tf


class control:
    def __init__(self, path=[[5.0, -4.0]]):
        self.path = np.array(path)
        self.v_cruising = 0.3  # 30 cm/s
        self.w_cruising = np.pi/3  # 60 deg/s

        # Listener odometry
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)

        # Velocity publisher
        # self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)

        # Listener loop
        i = 0
        # Errors
        last_lin_error, accu_lin_error = [0, 0]
        last_ang_error, accu_ang_error = [0, 0]

        rospy.sleep(10.0)
        while not rospy.is_shutdown() and i < len(self.path):
            # Listener block
            try:
                trans = tfBuffer.lookup_transform(
                    'odom', 'base_footprint', rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue

            current_theta = tf.euler_from_quaternion([trans.transform.rotation.x,
                                                      trans.transform.rotation.y,
                                                      trans.transform.rotation.z,
                                                      trans.transform.rotation.w])[2]

            yaw_0 = rospy.get_param("yaw")
            x_0 = rospy.get_param("x")
            y_0 = rospy.get_param("y")

            mth = np.matmul(tf.euler_matrix(0.0,0.0,-yaw_0,'sxyz'),tf.translation_matrix((x_0,y_0,0.0)))

            target = np.array([self.path[i, 0], #x
                               self.path[i, 1], #y
                               0.0,
                               1.0])

            x,y,_,_ = np.dot(mth,target)

            x_error = x - trans.transform.translation.x
            y_error = y - trans.transform.translation.y

            linear_error = np.sqrt(x_error**2 + y_error**2)
            angular_error = self.normalizeAngle(np.arctan2(y_error, x_error) - current_theta)

            kobuki_vel = Twist()

            if abs(linear_error) + abs(angular_error) <= 0.1:
                self.pub.publish(Twist())
                rospy.loginfo('Point achieved: '+str(path[i]))
                i = i + 1
                continue
            else:
                # Linear
                kp = 0.4 # rospy.get_param("/PID/Kpv")
                ki = 0.4 # rospy.get_param("/PID/Kiv")
                kd = 10.0 # rospy.get_param("/PID/Kdv")
                accu_lin_error += linear_error
                rate_error = linear_error - last_lin_error
                last_lin_error = linear_error
                v = kp*linear_error + ki*accu_lin_error + kd*rate_error

                # Angular
                kp = 3.7 # rospy.get_param("/PID/Kpw")
                ki = 0.0 # rospy.get_param("/PID/Kiw")
                kd = 0.0 # rospy.get_param("/PID/Kdw")
                accu_ang_error += angular_error
                rate_error = angular_error - last_ang_error
                last_ang_error = angular_error
                w = kp*angular_error + ki*accu_ang_error + kd*rate_error

                kobuki_vel.linear.x = v
                if abs(kobuki_vel.linear.x) > self.v_cruising:
                    kobuki_vel.linear.x = self.v_cruising

                kobuki_vel.angular.z = w
                if abs(kobuki_vel.angular.z) > self.w_cruising:
                    kobuki_vel.angular.z = self.w_cruising * np.sign(kobuki_vel.angular.z)

            # print self.path[i] 
            # print [trans.transform.translation.x, trans.transform.translation.y]
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

    rospy.loginfo("Node control init")

    control(path=path)

    rospy.spin()
