#!/usr/bin/env python3

import rospy
import numpy as np
import collections
import matplotlib.pyplot as plt
import time
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button
from std_msgs.msg import Float64

#needs to be run to get readings
#rosrun rosserial_python serial_node.py _baud:=9600 /dev/ttyACM0


class Force_sensor:
	def __init__(self):
		self.listener = rospy.Subscriber('/force_reading', Float64, self.callback)
		self.force_plt = collections.deque(np.zeros(10))
		self.force = 0
		self.fig = plt.figure(figsize=(12,6), facecolor='#DEDEDE')
		self.ax = plt.subplot()
		self.ax.set_facecolor('#DEDEDE')

		self.start_target = False
		self.target_type = ['trapezoid', 'sinusoid']
		self.target_idx = 0

		self.ticker = 0
		self.speed_scale = 40 #shouldnt be zero

		self.diff_list = []
		self.recorded_force = []
		self.recorded_target = []
		self.t_x = []

		self.blind = False

		axstart = plt.axes([0.01, 0.4, 0.08, 0.075])
		bstart = Button(axstart, 'Start')
		bstart.on_clicked(self.start_target_event)

		axnext = plt.axes([0.01, 0.5, 0.08, 0.075])
		bnext = Button(axnext, 'Next')
		bnext.on_clicked(self.select_next)

		axstop = plt.axes([0.01, 0.3, 0.08, 0.075])
		bstop = Button(axstop, 'Stop')
		bstop.on_clicked(self.stop_target_event)

		axblind = plt.axes([0.01, 0.1, 0.08, 0.075])
		bblind = Button(axblind, 'Blind')
		bblind.on_clicked(self.blind_test_event)


		ani = FuncAnimation(self.fig, self.update_plot, interval=20)
		plt.show()

	def callback(self, msg):
		'''y = msg.data
		y_lst = self.force_plt
		y_lst.popleft()
		y_lst.append(y)
		'''
		self.force = msg.data

	def start_target_event(self,event):
		self.start_target = True
		self.ticker = 0
		self.diff_list = []

	def select_next(self, event):
		cur_idx = self.target_idx
		size_tests = len(self.target_type)

		if cur_idx+1 == size_tests:
			self.target_idx = 0
		else:
			self.target_idx += 1

	def stop_target(self):
		self.start_target = False

	def stop_target_event(self,event):
		self.start_target = False

	def blind_test_event(self, event):
		self.blind = not self.blind

	def update_plot(self,i):
		self.ticker += 1
		d = self.ticker/self.speed_scale

		y_lst = self.force
		
		self.ax.cla()
		self.ax.set_ylim(0,40)
		self.ax.set_xlim(0,12)
		self.ax.set_xticks([])
		self.ax.text(.01, 0.6, self.target_type[self.target_idx], fontsize=14, transform=plt.gcf().transFigure)

		#plot force level
		#self.ax.plot(y_lst)
		#self.ax.scatter(len(y_lst)-1, y_lst[-1])
		if self.blind:
			self.ax.scatter(6,y_lst,marker="P", color='#DEDEDE')
		else:
			self.ax.scatter(6,y_lst,marker="P", color='k')
		#self.ax.text(len(y_lst)-1, y_lst[-1]+2, "{}kg".format(y_lst[-1]))
		
		#plot target
		if self.start_target:
			if self.blind:
				self.ax.axvline(x=6, linestyle='--', linewidth=1, color='r')
			if self.target_type[self.target_idx] == 'trapezoid':
				x = [8-d, 10-d, 13-d, 15-d]
				y = [0, 15, 15, 0]
				self.ax.plot(x, y)
				j = np.interp(6, x,y)

			if self.target_type[self.target_idx] == 'sinusoid':
				x = np.arange(8,15,0.1)
				y = 2*np.sin(2.5*x)+15
				x = np.arange(8-d,15-d,0.1)
				self.ax.plot(x, y)
				j = np.interp(6, x,y)
				
			if j != 0:				
				self.store_data(y_lst, j)

			#target has passed
			if x[-1] < 5.5:
				rmse = np.sqrt(np.mean(self.diff_list))
				self.diff_list = []
				#result_fig = plt.figure(figsize=(12,6), facecolor='#DEDEDE')
				#plt.plot(self.t_x, self.recorded_force)
				#plt.plot(self.t_x, self.recorded_target)
				#plt.ylim(0,1)
				#plt.show()
				self.save_to_file(rmse)
				self.stop_target()


	def store_data(self, i, j):
		#i = force
		#mx = i
		#mn = 0

		diffmx = (i-j)**2
		diffmn = (0-j)**2

		if diffmx > diffmn:
			diff = diffmx
		else:
			diff = diffmn
		self.diff_list.append(diff)
		self.recorded_force.append(i)
		self.recorded_target.append(j)
		self.t_x.append(self.ticker)

	def save_to_file(self, rmse):
		with open(self.target_type[self.target_idx] + str(time.time()) + ".txt", "w") as f:	
			str_recorded_force = [str(i) for i in self.recorded_force]
			str_recorded_target = [str(i) for i in self.recorded_target]
			f.write(",".join(str_recorded_force)+'\n')
			f.write(",".join(str_recorded_target)+'\n')
			f.write(str(rmse))
			self.recorded_target = []
			self.recorded_force = []


if __name__ == '__main__':
	rospy.init_node('force_sensor', anonymous=True)
	Force_sensor()
	
	rospy.spin()