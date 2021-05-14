#!/usr/bin/python
import rospy
import numpy as np
from tf.transformations import rotation_matrix
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
		self.pub = rospy.Publisher(
		    '/mobile_base/commands/velocity', Twist, queue_size=10)

		# Frecuency
		f = 30.0  # Hz
		rate = rospy.Rate(f)
		# Errors
		last_theta = 0.0
		cum_theta = 0.0
		# Listener loop
		i = 0
		while not rospy.is_shutdown() and i < len(self.path):
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

			x_distance = self.path[i, 0]-trans.transform.translation.x
			y_distance = self.path[i, 1]-trans.transform.translation.y

			desired_theta = np.arctan2(y_distance, x_distance)
			distance = np.sqrt(x_distance**2 + y_distance**2)

			kobuki_vel = Twist()

			if abs(current_theta - desired_theta) <= 0.1:
				if distance>0.3:
					kobuki_vel.linear.x = 0.3
				else:
					kobuki_vel.linear.x = distance

				if distance <= 0.1:
					self.pub.publish(Twist())
					print('Point:', path[i])
					i = i + 1
					# rospy.sleep(5.0)
					continue
			else:
				kp, ki, kd = [5.0, 0.0, 0.0]
				omega, cum_theta, last_theta = self.PIDt(
				    desired_theta, current_theta, 1/f, cum_theta, last_theta, kp, ki, kd)
				if abs(omega)>np.pi/3.0:
					kobuki_vel.angular.z=np.pi/3.0*omega/abs(omega)
				else:
					kobuki_vel.angular.z=omega

			self.pub.publish(kobuki_vel)
			print(self.path[i])
			print(desired_theta, current_theta, distance)
			print(kobuki_vel)
			print()
			rate.sleep()

	def PIDt(self, desired_value, current_value, dt, cum_error, previous_error, kp, ki, kd):
		error = desired_value - current_value 
		if error >= np.pi:
			error -= 2.0*np.pi
		if error < -np.pi:
			error += 2.0*np.pi
		cum_error = cum_error + (error * dt)
		rate_error = (error - previous_error) / dt

		previous_error = error		
		return(kp*error + ki*cum_error + kd*rate_error, cum_error, error)

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
			[-1.0, -5.5]]

	rospy.loginfo("Node init")

	control(path=path)

	rospy.spin()
