#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "geometry_msgs/Point.h"
#include "qb_device_srvs/GetMeasurements.h"

int controller_type = 0;

void mat_callback(const std_msgs::Point& msg) {
	if (controller_type == 0) {
		position_control(msg)
	}

	else {
		velocity_control(msg)
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "hand_controller");
	ros::NodeHandle n;

	hand_publisher = n.advertise<trajectory_msgs::JointTrajectory>(
					'/qbhand1/control/qbhand1_synergy_trajectory_controller/command', 3);
	ros::Subscriber mat_listener = n.subscribe("/matlab_est",1000, mat_callback);


	return 0;
};