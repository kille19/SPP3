#pragma once
#include <Eigen/Dense>

struct RobotState {
	bool valid = false;
	double t = 0.0;
	double q[7] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }; // joint angles (1-6) and (0) unused
};

// where the imu's are located on the robot
enum class RobotLink : int {
	Link3 = 3,
	Tool = 6
};

Eigen::Quaterniond expectedRotation(const RobotState& state, RobotLink link);

