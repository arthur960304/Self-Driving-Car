#!/usr/bin/env python

import rospy
import eigen as e #Perform matrix computation
from sensor_msgs.msg import Imu
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Vector3
from math import sqrt, sin, cos

global num, C, g, vel_vec, pos_vec, last_time, marker_pub, marker
num = 0
last_time = 0
C = e.Matrix3d.Identity()
vel_vec = e.Vector3d.Zero()
pos_vec = e.Vector3d.Zero()
g = e.Vector3d.Zero()
marker_pub = rospy.Publisher('route', Marker, queue_size = 100)
marker = Marker()
marker.points = []


def listener(): 
	rospy.init_node("deadReckoning")
	rospy.Subscriber("/imu/data", Imu, callback)
	rospy.spin()


def callback(data_sub):
	global num, C, g, vel_vec, pos_vec, last_time, marker_pub, marker
	B = e.Matrix3d.Zero()
	I = e.Matrix3d.Identity()
	acc_vec = e.Vector3d.Zero()
	angular_vec = e.Vector3d.Zero()
	Header = data_sub.header
	current_time = Header.stamp.to_sec()

	if num == 0:
		last_time = current_time
	dt = current_time - last_time
	
	# Get angular and acceleration data from IMU
	angular_vec[0] = data_sub.angular_velocity.x
	angular_vec[1] = data_sub.angular_velocity.y
	angular_vec[2] = data_sub.angular_velocity.z
	
	acc_vec[0] = data_sub.linear_acceleration.x
	acc_vec[1] = data_sub.linear_acceleration.y
	acc_vec[2] = data_sub.linear_acceleration.z

	#Calculate orientation below
	sigma = sqrt((angular_vec[0] * dt)**2 + (angular_vec[1] * dt)**2 + (angular_vec[2] * dt)**2)

	B[0,0], B[1,1], B[2,2] = 0, 0, 0
	B[0, 1] = -angular_vec[2] * dt
	B[0, 2] = angular_vec[1] * dt
	B[1, 0] = angular_vec[2] * dt
	B[1, 2] = -angular_vec[0] * dt
	B[2, 0] = -angular_vec[1] * dt
	B[2, 1] = angular_vec[0] * dt

	if num == 0 :
		C = e.Matrix3d.Identity() #Rotation matrix C
		g[0] = data_sub.linear_acceleration.x #Assume to initial gravity
		g[1] = data_sub.linear_acceleration.y
		g[2] = data_sub.linear_acceleration.z
		num = num + 1
	else :
		C = C * (I + (sin(sigma)/sigma)*B + ((1 - cos(sigma))/(sigma*sigma))*B*B)

	acc_vec = C * acc_vec #Convert base frame to global frame
	vel_vec = vel_vec + dt * (acc_vec - g)
	pos_vec = pos_vec + dt * vel_vec
	last_time = Header.stamp.to_sec()

	#Show the result in Rviz
	iteration = 0
	while not rospy.is_shutdown() and iteration < 1:
		marker.header.frame_id = '/map'
		marker.type = marker.LINE_STRIP
		marker.action = marker.ADD

		#Marker scale
		marker.scale.x = 0.1

		#Marker color
		marker.color.a = 1.0
		marker.color.r = 1.0
		marker.color.g = 1.0
		marker.color.b = 0.0

		#Marker orientation
		marker.pose.orientation.x = 0.0
		marker.pose.orientation.y = 0.0
		marker.pose.orientation.z = 0.0
		marker.pose.orientation.w = 1.0

		#Marker line points
		p = Point()
		p.x = pos_vec[0]
		p.y = pos_vec[1]
		p.z = pos_vec[2]
		marker.points.append(p)

		#Publish the marker
		marker_pub.publish(marker)
		iteration = iteration + 1
	


if __name__ == '__main__':
	listener()

