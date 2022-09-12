#!/usr/bin/env python3

import rospy
import logging
import argparse
import numpy as np
import scripts.bimatrix_controller_framework as framework

from std_msgs.msg import String, Float64
from geometry_msgs.msg import Point

##electrode array pin numbers

#24 17 16  9
#23 20 13 10
#22	19 14 11
#21 18 15 12

class Bimatrix_controller:
	def __init__(self, args):
		self.cocontr_listener = rospy.Subscriber('/matlab_cocontr', String, self.cocontraction_callback)
		#self.est_listener = rospy.Subscriber('/matlab_est', Point, self.estimation_callback)
		self.keyboard_listener = rospy.Subscriber('/keyboard', String, self.keyboard)
		#self.force_listener = rospy.Subscriber('/force_reading', Float64, self.force_callback)
		self.control_framework = framework.Controller(args.device, logging_level=self.log_level(args.logging_level), log_file=args.log_file)

		self.control_framework.read_battery()
		
		self.a = 14
		self.p = 100
		self.r = 6 #how many pulses per second
		
		self.burst_duration = 0.2

		self.velocity_feedback_mode = 0
		self.force_feedback_mode = 0
		self.triggered = False
		self.num_pairs = 4

		#set device initial parameters
		self.control_framework.set_pulse_generator(True)
		self.control_framework.set_current_range('high')
		self.control_framework.set_voltage(120)
		self.control_framework.set_num_nplets(0)
		self.control_framework.set_time_between(1)
		self.control_framework.set_delay(0)
		self.control_framework.set_repetition_rate(self.r)
		self.control_framework.set_pulse_width([self.p]*4)
		self.control_framework.set_amplitude([self.a]*4)
		#self.control_framework.set_mode('unipolar')
		#self.control_framework.set_common_electrode('cathode')
		#self.control_framework.set_pulses_unipolar([[24]])
		self.control_framework.set_mode('bipolar')
		self.control_framework.set_pulses_bipolar([([24],[17]), ([13],[10]), ([15],[12]), ([22],[19])])

		self.force_listener = rospy.Subscriber('/force_reading', Float64, self.force_callback)

		#print parameters
		print(self.control_framework)

	def __del__(self):
		self.control_framework.set_pulse_generator(False)

	def cocontraction_callback(self, msg):
		#self.burst_trigger()
		pass

	def estimation_callback(self, msg):
		#self.velocity_feedback(msg.y)
		pass

	def force_callback(self, msg):
		#self.force_feedback(msg.data)
		pass

	def burst_trigger(self):
		print("Burst triggered")
		self.control_framework.trigger_pulse_generator()
		rospy.sleep(self.burst_duration)
		self.control_framework.trigger_pulse_generator()
		print("Burst stopped")

	def force_feedback(self, f):
		#print(f)
		#print(self.force_feedback_mode)
		if f < 1:
			if self.triggered:
				self.control_framework.trigger_pulse_generator()
				self.force_feedback_mode = 0
				print(0)
				self.triggered = False

		elif 1 <= f < 7.5:
			if self.force_feedback_mode != 1:
				self.force_feedback_mode = 1
				print(1)
				self.control_framework.set_pulses_bipolar( [([15],[12]) ])
				self.control_framework.set_amplitude([self.a+6])
				self.control_framework.set_pulse_width([self.p+30])
				self.control_framework.set_repetition_rate(5)
				if not self.triggered:
					self.control_framework.trigger_pulse_generator()
					self.triggered = True

		elif 7.5 <= f < 15:
			if self.force_feedback_mode != 2:
				self.force_feedback_mode = 2
				print(2)
				self.control_framework.set_pulses_bipolar( [([22],[19]), ([15],[12])] )
				self.control_framework.set_amplitude([self.a+5]*2)
				self.control_framework.set_pulse_width([self.p+20]*2)
				self.control_framework.set_repetition_rate(7)
				if not self.triggered:
					self.control_framework.trigger_pulse_generator()
					self.triggered = True

		elif 15 <= f < 22.5:
			if self.force_feedback_mode != 3:
				self.force_feedback_mode = 3
				print(3)
				self.control_framework.set_pulses_bipolar( [([13],[10]), ([15],[12]), ([22],[19])] )
				self.control_framework.set_amplitude([self.a+2]*3)
				self.control_framework.set_pulse_width([self.p+10]*3)
				self.control_framework.set_repetition_rate(9)
				if not self.triggered:
					self.control_framework.trigger_pulse_generator()
					self.triggered = True

		elif 22.5 <= f < 100:
			if self.force_feedback_mode != 4:
				self.force_feedback_mode = 4
				print(4)
				self.control_framework.set_pulses_bipolar( [([24],[17]), ([13],[10]), ([15],[12]), ([22],[19])] )
				self.control_framework.set_amplitude([self.a]*4)
				self.control_framework.set_pulse_width([self.p]*4)
				self.control_framework.set_repetition_rate(11)
				if not self.triggered:
					self.control_framework.trigger_pulse_generator()
					self.triggered = True

	def velocity_feedback(self, y):
		#limits = [-20,20]
		#ranges = np.linspace(limits[0],limits[1], 5)

		print(y)

		if -2.5 <= y < 2.5:
			if self.triggered:
				self.control_framework.trigger_pulse_generator()
				self.triggered = False

		#if ranges[0] <= y < ranges[1]:
		elif 2.5 <= y < 7:
			if self.velocity_feedback_mode != 1:
				self.velocity_feedback_mode = 1
				self.control_framework.set_pulses_bipolar( [([19],[16]) ])
				self.control_framework.set_amplitude([self.a])
				self.control_framework.set_pulse_width([self.p])
				if not self.triggered:
					self.control_framework.trigger_pulse_generator()
					self.triggered = True

		#elif ranges[1] <= y < ranges[2]:
		elif 7 <= y < 11:
			if self.velocity_feedback_mode != 2:
				self.velocity_feedback_mode = 2
				self.control_framework.set_pulses_bipolar( [([18],[15]), ([19],[16])] )
				self.control_framework.set_amplitude([self.a+4]*3)
				self.control_framework.set_pulse_width([self.p+20]*2)
				self.control_framework.set_repetition_rate(7)
				if not self.triggered:
					self.control_framework.trigger_pulse_generator()
					self.triggered = True

		#elif ranges[2] <= y < ranges[3]:
		elif 11 <= y < 15:
			if self.velocity_feedback_mode != 3:
				self.velocity_feedback_mode = 3
				self.control_framework.set_pulses_bipolar( [([18],[15]), ([19],[16]), ([17],[14])] )
				self.control_framework.set_amplitude([self.a]*3)
				self.control_framework.set_pulse_width([self.p]*3)
				if not self.triggered:
					self.control_framework.trigger_pulse_generator()
					self.triggered = True

		#elif ranges[3] <= y <= ranges[4]:
		elif 15 <= y <= 20:
			if self.velocity_feedback_mode != 4:
				self.velocity_feedback_mode = 4
				self.control_framework.set_pulses_bipolar( [([18],[15]), ([19],[16]), ([17],[14]), ([20],[13])] )
				self.control_framework.set_amplitude([self.a]*4)
				self.control_framework.set_pulse_width([self.p]*4)
				if not self.triggered:
					self.control_framework.trigger_pulse_generator()
					self.triggered = True

		elif -7 <= y < -2.5:
			if self.velocity_feedback_mode != 5:
				self.velocity_feedback_mode = 5
				self.control_framework.set_pulses_bipolar( [([20],[13]) ])
				self.control_framework.set_amplitude([self.a])
				self.control_framework.set_pulse_width([self.p])
				if not self.triggered:
					self.control_framework.trigger_pulse_generator()
					self.triggered = True

		#elif ranges[1] <= y < ranges[2]:
		elif -11 <= y < -7:
			if self.velocity_feedback_mode != 6:
				self.velocity_feedback_mode = 6
				self.control_framework.set_pulses_bipolar( [([20],[13]), ([17],[14])] )
				self.control_framework.set_amplitude([self.a]*2)
				self.control_framework.set_pulse_width([self.p]*2)
				if not self.triggered:
					self.control_framework.trigger_pulse_generator()
					self.triggered = True

		#elif ranges[2] <= y < ranges[3]:
		elif -15 <= y < -11:
			if self.velocity_feedback_mode != 7:
				self.velocity_feedback_mode = 7
				self.control_framework.set_pulses_bipolar( [([20],[13]), ([17],[14]), ([18],[15])] )
				self.control_framework.set_amplitude([self.a]*3)
				self.control_framework.set_pulse_width([self.p]*3)
				if not self.triggered:
					self.control_framework.trigger_pulse_generator()
					self.triggered = True

		#elif ranges[3] <= y <= ranges[4]:
		elif -20 <= y <= -15:
			if self.velocity_feedback_mode != 8:
				self.velocity_feedback_mode = 8
				self.control_framework.set_pulses_bipolar( [([18],[15]), ([19],[16]), ([17],[14]), ([20],[13])] )
				self.control_framework.set_amplitude([self.a]*4)
				self.control_framework.set_pulse_width([self.p]*4)
				if not self.triggered:
					self.control_framework.trigger_pulse_generator()
					self.triggered = True

	def keyboard(self, msg):
		var = msg.data[0]

		#print(self.control_framework)

		if var == 'a':
			self.change_amplitude(msg.data[1])
		elif var == 'w':
			self.change_width(msg.data[1])
		elif var == 'r':
			self.change_rate(msg.data[1])
		elif var == 'd':
			self.change_burst_duration(msg.data[1])
		elif var == 'p':
			self.change_num_pairs(msg.data[1])
		elif var == 't':
			self.control_framework.trigger_pulse_generator()
			self.triggered = not self.triggered
			#print(self.triggered)
		else:
			return


	def log_level(self, x: str):
		return {
			'debug': logging.DEBUG,
			'info': logging.INFO,
			'warning': logging.WARNING,
			'error': logging.ERROR,
			'critical': logging.CRITICAL
		}.get(x.lower(), logging.error)

	def change_num_pairs(self, x):
		cur_p = self.num_pairs

		if x == '1' and cur_p != 4:
			new_p = cur_p + 1
		elif x == '0' and cur_p != 1:
			new_p = cur_p - 1

		if new_p == 1:
			self.control_framework.set_pulses_bipolar([([15],[12])])
			self.control_framework.set_amplitude([self.a+6]*new_p)
			self.control_framework.set_pulse_width([self.p+30]*new_p)
			self.control_framework.set_repetition_rate(5)
			print("mode 1")
		elif new_p == 2:
			self.control_framework.set_pulses_bipolar([([15],[12]), ([22],[19])])
			self.control_framework.set_amplitude([self.a+4]*new_p)
			self.control_framework.set_pulse_width([self.p+20]*new_p)
			self.control_framework.set_repetition_rate(7)
			print("mode 2")
		elif new_p == 3:
			self.control_framework.set_pulses_bipolar([([13],[10]), ([15],[12]), ([22],[19])])
			self.control_framework.set_amplitude([self.a+2]*new_p)
			self.control_framework.set_pulse_width([self.p+10]*new_p)
			self.control_framework.set_repetition_rate(9)
			print("mode 3")
		elif new_p == 4:
			self.control_framework.set_pulses_bipolar([([24],[17]), ([13],[10]), ([15],[12]), ([22],[19])])
			self.control_framework.set_amplitude([self.a]*new_p)
			self.control_framework.set_pulse_width([self.p]*new_p)
			self.control_framework.set_repetition_rate(11)
			print("mode 4")

		self.num_pairs = new_p


	def change_amplitude(self, x):
		cur_a = self.a

		if x == '1':
			new_a = cur_a + 1
		elif x == '0':
			new_a = cur_a - 1

		res = self.control_framework.set_amplitude([new_a]*self.num_pairs)
		print(res)
		if res:
			self.a = new_a
			print("Amplitude:", new_a)
		else:
			print("Amplitude: {}, amplitude not changed due to error".format(cur_a))


	def change_width(self, x):
		cur_w = self.p

		if x == '1':
			new_w = cur_w + 25
		elif x == '0':
			new_w = cur_w - 25

		res = self.control_framework.set_pulse_width([new_w]*self.num_pairs)
		print(res)
		if res:
			self.p = new_w
			print("Pulse width:", new_w)
		else:
			print("Pulse width: {}, width not changed due to error".format(cur_w))

	def change_rate(self, x):
		cur_r = self.r

		if x == '1':
			new_r = cur_r + 1
		elif x == '0':
			new_r = cur_r - 1

		res = self.control_framework.set_repetition_rate(new_r)
		print(res)
		if res:
			self.r = new_r
			print("Rate per second:", new_r)
		else:
			print("Rate per second: {}, rate not changed due to error".format(cur_w))

	def change_burst_duration(self, x):
		cur_d = self.burst_duration

		if x == '1':
			new_d = cur_d + 0.1
		elif x == '0':
			new_d = cur_d - 0.1

		self.burst_duration = new_d
		print("Burst duration:", new_d)

if __name__ == '__main__':
	rospy.init_node('bimatrix_controller', anonymous=True)
	
	parser = argparse.ArgumentParser(description='Controller options')
	parser.add_argument('-d', '--device', default="/dev/ttyUSB0", help='Device serial port')
	parser.add_argument('-l', '--logging_level', default="warning", help='Logging level')
	parser.add_argument('-f', '--log_file', default="", help='Log file')
	parser.add_argument('-c', '--commands', default="", help='Commands to be executed on the controller')
	arguments = parser.parse_args()

	Bimatrix_controller(arguments)
	
	rospy.spin()