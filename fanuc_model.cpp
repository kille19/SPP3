#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include "fanuc_model.h"

bool FanucModel::loadFromURDF(const std::string& urdf_path, const std::string& base_frame, const std::string& link3_frame, const std::string& tool_frame) {
	model_ = pinocchio::Model();
	pinocchio::urdf::buildModel(urdf_path, model_);
	data_ = pinocchio::Data(model_);

	base_fid_ = model_.getFrameId(base_frame);
	link3_fid_ = model_.getFrameId(link3_frame);
	tool_fid_ = model_.getFrameId(tool_frame);

	q_ = Eigen::VectorXd::Zero(model_.nq);
	loaded_ = true;
	updateFK_();
	return true;
}

void FanucModel::setJointAngles(const Eigen::Matrix<double, 6, 1>& q_rad) {
	if (!loaded_) {
		return;
	}
	
	if (model_.nq == 6) {
		q_.head<6>() = q_rad;
	}
	updateFK_();
}

void FanucModel::updateFK_() {
	pinocchio::forwardKinematics(model_, data_, q_);
	pinocchio::updateFramePlacements(model_, data_);
}

Eigen::Quaterniond FanucModel::baseToLink3Rotation() const {
	const auto& M = data_.oMf[link3_fid_];
	return Eigen::Quaterniond(M.rotation());
}

Eigen::Quaterniond FanucModel::baseToToolRotation() const {
	const auto& M = data_.oMf[tool_fid_];
	return Eigen::Quaterniond(M.rotation());
}

Eigen::Vector3d FanucModel::baseToLink3Position() const {
	const auto& M = data_.oMf[link3_fid_];
	return M.translation();
}

Eigen::Vector3d FanucModel::baseToToolPosition() const {
	const auto& M = data_.oMf[tool_fid_];
	return M.translation();
}