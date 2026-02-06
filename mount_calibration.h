#pragma once
#include <Eigen/Dense>
#include <vector>
#include <cassert>
#include <algorithm>
#include <cmath>

static inline Eigen::Matrix<double, 9, 9> computeCalibration(const Eigen::Matrix3d& A, const Eigen::Matrix3d& B)	{
	Eigen::Matrix<double, 9, 9> K;
	K.setZero();
	for (int i = 0; i < 3; ++i)	{
		for (int j = 0; j < 3; ++j)	{
			K.block<3, 3>(i * 3, j * 3) = A(i, j) * B;
		}
	}
	return K;
}

static inline double rotationAngle(const Eigen::Matrix3d& R) {
	double c = (R.trace() - 1.0) * 0.5;
	c = std::clamp(c, -1.0, 1.0);
	return acos(c);
}

static inline bool handEye(const std::vector<Eigen::Matrix3d>& A_list, const std::vector<Eigen::Matrix3d>& B_list, Eigen::Matrix3d& X_out)	{
	assert(A_list.size() == B_list.size());
	
	const int K = static_cast<int>(A_list.size());
	Eigen::MatrixXd M(9 * K, 9);
	M.setZero();

	const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();	

	for (int k = 0; k < K; ++k) {
		const Eigen::Matrix3d& A = A_list[k];
		const Eigen::Matrix3d& B = B_list[k];

		Eigen::Matrix<double, 9, 9> Mk = computeCalibration(I, A) - computeCalibration(B.transpose(), I);
		M.block(9 * k, 0, 9, 9) = Mk;
	}

	Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeFullV);
	Eigen::VectorXd v = svd.matrixV().col(8);

	Eigen::Matrix3d X_temp;
	X_temp << v(0), v(1), v(2),
		 v(3), v(4), v(5),
		v(6), v(7), v(8);

	Eigen::JacobiSVD<Eigen::Matrix3d> svd_X(X_temp, Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::Matrix3d U = svd_X.matrixU();
	Eigen::Matrix3d V = svd_X.matrixV();

	Eigen::Matrix3d R = U * V.transpose();
	if (R.determinant() < 0.0) {
		Eigen::Matrix3d U2 = U;
		U2.col(2) *= -1.0;
		R = U2 * V.transpose();
	}

	X_out = R;
	return true;
}