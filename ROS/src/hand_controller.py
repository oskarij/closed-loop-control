#!/usr/bin/env python3

import rospy
import time

from geometry_msgs.msg import Point
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from qb_device_srvs.srv import GetMeasurements

class Hand_controller:

	def __init__(self):
		self.controller_type = 0 #using position control as default
		self.hand_publisher = rospy.Publisher('/qbhand1/control/qbhand1_synergy_trajectory_controller/command'
											, JointTrajectory, queue_size=3)
		self.mat_signal_listener = rospy.Subscriber('/matlab_est', Point, self.callback, queue_size=1)
		self.cocontr_listener = rospy.Subscriber('/matlab_cocontr', String, self.change_control, queue_size=1)
		self.get_measurements = rospy.ServiceProxy('/communication_handler/get_measurements', GetMeasurements, persistent=True)
		self.keyboard_listener = rospy.Subscriber('/keyboard', String, self.keyboard_callback, queue_size=1)
		self.hand_msg = JointTrajectory()
		self.mat_limits = [-100,100] 
		self.qb_limits = [0,1] 
		self.service_limits = [0,19000]
		self.velocity_signal_threshold = [-2,2]

		self.simulated_speed = Point()
		self.simulated_speed.y = 0
		self.simulated_speed.z = 0
		self.simulated_speed.x = 0

		
		#parameters
		self.rate = rospy.Rate(10) #publishing rate to robot
		self.max_speed = 4 #maximum duration from fully closed/open to fully closed/open
		self.matlab_rate_nsec = 80000000 #nanoseconds 
		self.matlab_rate_sec = 0.08 #seconds

		self.message_init()

	def __del__(self):
		self.get_measurements.close()

	def callback(self, msg):
		#send position control signals
		if self.controller_type == 0:
			self.position_control(msg)

		#send velocity control signals
		elif self.controller_type == 1:
			self.velocity_control(msg)

	def keyboard_callback(self, msg):
		var = msg.data[0]

		#print(self.control_framework)

		if var == '1':
			self.simulated_speed.y += 2
			print(self.simulated_speed.y)
			self.velocity_control(self.simulated_speed)
		elif var == '0':
			self.simulated_speed.y -= 2
			print(self.simulated_speed.y)
			self.velocity_control(self.simulated_speed)
		else:
			return


	def position_control(self, msg):
		#set hand speed to fastest = 1 nsec
		duration = 1

		robot_position = self.transform_position(self.mat_limits, self.qb_limits, msg.x)

		#additional joint limits
		if robot_position > 0.95:
			robot_position = 0.95
		elif robot_position < 0.05:
			robot_position = 0.05

		self.set_new_position(robot_position, 1, 0)

	def velocity_control(self, msg):
		#0 = open, 1 = closed
		#parameters: id, max_repeats, get_positions, get_currents, get_distinct_packages, get_commands
		srv_message = self.get_measurements(1, 0, True, True, False, False)
		#print(srv_message.positions[0])
		#print(srv_message.currents[0])
		current_position = srv_message.positions[0]

		#change to position used in trajectory messages [0,1]
		current_position = self.transform_position(self.service_limits, self.qb_limits, current_position)
		velocity = msg.y

		#stop movement
		if velocity > self.velocity_signal_threshold[0] and velocity < self.velocity_signal_threshold[1]:
			self.set_new_position(current_position, 0, 1)
		
		#from 1 to 0 (closed to open)
		if velocity > self.velocity_signal_threshold[1]:
			#1st parameter is positive matlab cursor limit and 2nd param. is lowest and highest duration for full open
			#transforming velocity signal to the amount of duration it takes to open fully
			full_duration = self.transform_position([0,20],[20,4], velocity)
			#scaling duration depending on the current position
			duration = full_duration * current_position
			#duration in full seconds (no decimals)
			duration_secs = int(duration)
			#residual duration in nanoseconds
			duration_nsecs = int(duration * 10**9)%10**9

			self.set_new_position(0, duration_secs, duration_nsecs)

		#from 0 to 1 (open to closed)
		if velocity < self.velocity_signal_threshold[0]:
			full_duration = self.transform_position([-20,0],[4,20], velocity)
			duration = full_duration - current_position*full_duration
			duration_secs = int(duration)
			duration_nsecs = int(duration * 10**9)%10**9

			self.set_new_position(1, duration_secs, duration_nsecs)
		

	def change_control(self, msg):
		if self.controller_type == 0:
			self.controller_type = 1
		else:
			self.controller_type = 0

		rospy.loginfo("Control mode changed to %i", self.controller_type)

	def transform_position(self, old_limits, new_limits, old_position):
		#if old position is over limits, change the value to be in the valid range
		if old_position < old_limits[0]:
			old_position = old_limits[0]
		elif old_position > old_limits[1]:
			old_position = old_limits[1]

		old_range = old_limits[1] - old_limits[0]
		new_range = new_limits[1] - new_limits[0]

		new_position = (((old_position - old_limits[0])*new_range)/old_range+new_limits[0])

		return new_position

	def set_new_position(self, position, duration_secs, duration_nsecs):
		self.hand_msg.points[0].time_from_start.secs = duration_secs
		self.hand_msg.points[0].time_from_start.nsecs = duration_nsecs
		self.hand_msg.points[0].positions[0] = position

		self.hand_publisher.publish(self.hand_msg)


	def message_init(self):
		self.hand_msg.joint_names = ['qbhand1_synergy_joint']
		point = JointTrajectoryPoint()
		point.positions.append(0)
		point.velocities.append(0)
		point.accelerations.append(0)
		point.effort.append(0)
		point.time_from_start.secs = 10
		point.time_from_start.nsecs = 0

		self.hand_msg.points.append(point)


if __name__ == '__main__':
	rospy.init_node('hand_controller', anonymous=True)
	Hand_controller()
	rospy.spin()