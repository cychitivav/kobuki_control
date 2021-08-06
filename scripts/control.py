#!/usr/bin/python
from math import sin
import rospy
import numpy as np
from tf.transformations import rotation_matrix, translation_matrix
from geometry_msgs.msg import Twist
import tf2_ros
from tf_conversions import transformations as tf
from geometry_msgs.msg import TransformStamped


class control:
    def __init__(self, path=[[-5.0, -4.0]]):
        # Variable initialization
        self.path = np.array(path)

        # Listener odometry
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        # Velocity publisher
        # self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)

        # Frecuency
        f = 30.0  # Hz
        rate = rospy.Rate(f)
        # Error variables
        last_error = 0.0
        cum_error = 0.0
        # Listener loop
        i = 0
        # Time while start gazebo
        rospy.sleep(5)
        while not rospy.is_shutdown() and i < len(self.path):
            # Listener block
            try:
                trans = self.tfBuffer.lookup_transform(
                    'odom', 'base_footprint', rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rate.sleep()
                continue
            

            current_theta = tf.euler_from_quaternion([trans.transform.rotation.x,
                                                      trans.transform.rotation.y,
                                                      trans.transform.rotation.z,
                                                      trans.transform.rotation.w])[2]

            yaw_inicial = rospy.get_param("yaw")
            x_inicial = rospy.get_param("x")
            y_inicial = rospy.get_param("y")

            mth=np.matmul(tf.euler_matrix(0.0,0.0,-yaw_inicial,'sxyz'),tf.translation_matrix((x_inicial,y_inicial,0.0)))
            
            # mth=np.array([[np.cos(-yaw_inicial), -np.sin(-yaw_inicial),   0.0,    x_inicial*np.cos(yaw_inicial)-y_inicial*np.sin(yaw_inicial)],
            #             [np.sin(-yaw_inicial),   np.cos(-yaw_inicial),    0.0,    x_inicial*np.sin(yaw_inicial)+y_inicial*np.cos(yaw_inicial)],
            #             [0.0,                0.0,       1.0,    0.0],
            #             [0.0,                0.0,       0.0,    1.0]]) 

            target = np.array([self.path[i, 0], #x
                               self.path[i, 1], #y
                               0.0,
                               1.0])

            x,y,_,_ = np.dot(mth,target)

            x_distance = x - trans.transform.translation.x 
            y_distance = y - trans.transform.translation.y 

            desired_theta = np.arctan2(y_distance, x_distance) 
            distance = np.sqrt(x_distance**2 + y_distance**2)

            kobuki_vel = Twist()

            error = desired_theta - current_theta 
            while error >= np.pi:
                error -= 2.0*np.pi
            while error < -np.pi:
                error += 2.0*np.pi

            if abs(error) <= 0.1:
                if distance > 0.3:
                    kobuki_vel.linear.x = 0.3
                else:
                    kobuki_vel.linear.x = distance

                if distance <= 0.01:
                    self.pub.publish(Twist())
                    print 'Point:', path[i]
                    print 'Point in odom:',"[",x[0],",",y[0],']' 
                    i = i + 1
                    continue
            else:
                K = rospy.get_param('K')
                cum_error += (error / f)
                rate_error = (error - last_error) * f
                last_error = error

                omega = K['p']*error + K['i']*cum_error + K['d']*rate_error
                if abs(omega) > np.pi/3.0:
                    kobuki_vel.angular.z = np.pi/3.0*omega/abs(omega)
                else:
                    kobuki_vel.angular.z = omega

            self.pub.publish(kobuki_vel)
            # print(self.path[i])
            # print(desired_theta, current_theta, distance)
            # print(kobuki_vel)
            # print()
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('control', anonymous=True)
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

    control(path=path)

    rospy.spin()
