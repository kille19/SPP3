#include "robot_state.h"
#include <Eigen/Dense>

Eigen::Quaterniond expectedRotation(const RobotState& state, int joint)
{
	if (!state.valid)
		return Eigen::Quaterniond::Identity();

	Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();
	double angle = state.q[joint];

	Eigen::AngleAxisd aa(angle, axis);
	return Eigen::Quaterniond(aa);
}