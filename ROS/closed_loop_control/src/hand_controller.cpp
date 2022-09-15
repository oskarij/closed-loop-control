#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "geometry_msgs/Point.h"
#include "qb_device_srvs/GetMeasurements.h"
#include <iostream>
ros::Publisher hand_publisher;
trajectory_msgs::JointTrajectory hand_msg;

int controller_type = 0;
int mat_limits[] {-100,100};
int qb_limits[] {0,1};
int service_limits[] {0,19000};
int velocity_signal_threshold[] {-2,2};


void mat_callback(const geometry_msgs::Point& msg) {
	if (controller_type == 0) {
	}

	else {
	}
};

void change_control_mode(const std_msgs::String& msg) {

};

void keyboard_callback(const std_msgs::String& msg){

};

void message_init(){
	hand_msg.joint_names.push_back("qbhand1_synergy_joint");
	trajectory_msgs::JointTrajectoryPoint point;
	point.positions.push_back(0);
	point.velocities.push_back(0);
	point.accelerations.push_back(0);
	point.effort.push_back(0);
	point.time_from_start = ros::Duration(10,0);
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "hand_controller");
	ros::NodeHandle n;

	message_init(); // constructing parameters JointTrajectory-message 


	hand_publisher = n.advertise<trajectory_msgs::JointTrajectory>("/qbhand1/control/qbhand1_synergy_trajectory_controller/command", 3);
	ros::Subscriber mat_listener = n.subscribe("/matlab_est",1000, mat_callback);
	ros::Subscriber cocontr_listener = n.subscribe("/matlab_cocontr", 1000, change_control_mode);
	ros::Subscriber keyboard_listener = n.subscribe("/keyboard", 1000, keyboard_callback);
	// add service for position measurement

	return 0;
};