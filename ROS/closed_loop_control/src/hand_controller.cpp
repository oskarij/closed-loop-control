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

float transform_position(const int old_limits[], const int new_limits[], int old_position){
	// if old position is over limits, change the value to be in the valid range
	if (old_position < old_limits[0]){
		old_position = old_limits[0];
	}
	else if (old_position > old_limits[1]){
		old_position = old_limits[1];
	}

	int old_range = old_limits[1] - old_limits[0];
	int new_range = new_limits[1] - old_limits[0];

	float new_position = (((old_position - old_limits[0])*new_range)/old_range+new_limits[0]);

	return new_position;
};

void set_new_position(const float position, const int duration_sec, const int duration_nsec) {
	hand_msg.points[0].time_from_start.sec = duration_sec;
	hand_msg.points[0].time_from_start.nsec = duration_nsec;
	hand_msg.points[0].positions[0] = position;

	hand_publisher.publish(hand_msg);
};

void position_control(const geometry_msgs::Point& msg) {
	//set hand speed to fastest = 1 nsec
	ros::Duration(0,1);

	float robot_position = transform_position(mat_limits, qb_limits, msg.x);

	// additional joint limits
	if (robot_position > 0.95) {
		robot_position = 0.95;
	}
	else if (robot_position < 0.05) {
		robot_position = 0.05;
	}


	// what has happened here
	set_new_position(robot_position, 1, 0);
};

void mat_callback(const geometry_msgs::Point& msg) {
	// send position control messages
	if (controller_type == 0) {
		position_control(msg);
	}

	// send velocity control messages
	else {
		velocity_control(msg);
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