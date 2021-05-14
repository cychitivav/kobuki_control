#!/usr/bin/python
import rospy
import numpy as np
from tf.transformations import rotation_matrix
from geometry_msgs.msg  import Twist
import tf2_ros
from tf_conversions import transformations as tf
from geometry_msgs.msg import TransformStamped

class control:
	def __init__(self, path=[[1.0, 1.0]], tolerance=0.01):
		# Variable initialization
		self.path = np.array(path)
		#self.x_G, self.y_G = [np.array(self.path[:,0]), np.array(self.path[:,1])]
		self.angle = np.arctan2(self.path[:,1], self.path[:,0])

		# Listener odometry
		self.tfBuffer = tf2_ros.Buffer()
		self.listener = tf2_ros.TransformListener(self.tfBuffer)
		
		# Velocity publisher
		#self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		self.pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)

		# Frecuency
		f = 120.0 # Hz
		rate = rospy.Rate(f)
		# Errors
		last_rho = 0.0
		cum_rho = 0.0
		
		last_theta = 0.0
		cum_theta = 0.0
		# Listener loop
		i = 0
		while not rospy.is_shutdown() and i < len(self.path):
			try:
				trans = self.tfBuffer.lookup_transform('odom', 'base_footprint', rospy.Time())
			except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
				rate.sleep()
				continue

			current_rho = np.sqrt(trans.transform.translation.x**2 + trans.transform.translation.y**2)
			current_theta = np.arctan2(trans.transform.translation.y, trans.transform.translation.x)
			print(current_rho)
			print([trans.transform.translation.x, trans.transform.translation.y], current_theta)
			print()
			desired_rho = np.linalg.norm(path[i])
			desired_theta = self.angle[i]
			print(desired_rho)
			print(path[i], desired_theta)

			if (abs(current_rho - desired_rho) <= 0.001) and (abs(current_theta - desired_theta) <= 0.001):
				self.pub.publish(Twist())
				print('Point:', path[i])
				i = i + 1
				last_rho = 0.0
				cum_rho = 0.0
				
				last_theta = 0.0
				cum_theta = 0.0
				rospy.sleep(5.0)
				continue

			kobuki_vel = Twist()
			kp, ki, kd = np.array([0.5, 0.0, 0.0])
			v, cum_rho, last_rho = self.PID(desired_rho, current_rho, 1/f, cum_rho, last_rho, kp, ki, kd)
						
			kp, ki, kd = np.array([4.0, 0.0, 0.0])
			w, cum_theta, last_theta = self.PIDt(desired_theta, current_theta, 1/f, cum_theta, last_theta, kp, ki, kd)

			kobuki_vel.linear.x = v
			kobuki_vel.angular.z = w

			print(kobuki_vel)
			self.pub.publish(kobuki_vel)
			rate.sleep()

	def PID(self, desired_value, current_value, dt, cum_error, previous_error, kp, ki, kd):
		error = desired_value - current_value
		cum_error = cum_error + (error * dt)
		rate_error = (error - previous_error) / dt

		previous_error = error		
		return(kp*error + ki*cum_error + kd*rate_error, cum_error, error)

	def PIDt(self, desired_value, current_value, dt, cum_error, previous_error, kp, ki, kd):
			error = desired_value - current_value
			if error > np.pi:
				error -= 2*np.pi
			if error < -np.pi:
				error += 2*np.pi
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

	control()#path=path)

	rospy.spin()