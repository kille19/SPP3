#pragma once
#include <Eigen/Dense>
#include "robot_state.h"

struct Pose {
	Eigen::Vector3d position = Eigen::Vector3d::Zero();
	Eigen::Matrix3d orientation = Eigen::Matrix3d::Identity();
};

