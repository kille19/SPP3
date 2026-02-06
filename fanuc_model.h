#pragma once
#include <Eigen/Dense>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>


class FanucModel {
public:
	bool loadFromURDF(const std::string& urdf_path, const std::string& base_frame, const std::string& link3_frame, const std::string& tool_frame);

	void setJointAngles(const Eigen::Matrix<double, 6, 1>& q_rad);

	Eigen::Quaterniond baseToLink3Rotation() const;
	Eigen::Quaterniond baseToToolRotation() const;

	Eigen::Vector3d baseToLink3Position() const;
	Eigen::Vector3d baseToToolPosition() const;

private:
	void updateFK_();

	pinocchio::Model model_;
	pinocchio::Data data_;

	Eigen::VectorXd q_;

	pinocchio::FrameIndex base_fid_{ 0 };
	pinocchio::FrameIndex link3_fid_{ 0 };
	pinocchio::FrameIndex tool_fid_{ 0 };

	bool loaded_{ false };
};