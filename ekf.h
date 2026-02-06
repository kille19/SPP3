#pragma once
#include <Eigen/Dense>
#include <functional>

template<int N, int M> // N = # values in state vector, M = # values in measurement vector

class EKF {
public:
	using StateVector = Eigen::Matrix<double, N, 1>;
	using StateMatrix = Eigen::Matrix<double, N, N>;

	using MeasurementVector = Eigen::Matrix<double, M, 1>;
	using MeasurementMatrix = Eigen::Matrix<double, M, M>;

	using MeasurementModelMatrix = Eigen::Matrix<double, M, N>;
	using MeasurementModelMatrixT = Eigen::Matrix<double, N, M>;

	StateVector x = StateVector::Zero(); // state estimate
	StateMatrix P = StateMatrix::Identity(); // covariance estimate
	StateMatrix Q = StateMatrix::Identity(); // process noise covariance
	MeasurementMatrix R = MeasurementMatrix::Identity(); // measurement noise covariance

	std::function<StateVector(const StateVector&, double)> f; // state transition function
	std::function<StateMatrix(const StateVector&, double)> F; // state transition Jacobian

	std::function<MeasurementVector(const StateVector&)> h; // measurement function
	std::function<MeasurementModelMatrix(const StateVector&)> H; // measurement Jacobian

	bool joseph_form = true;

	// prediction step
	void predict(double dt) {
		assert(f && F);
		StateVector x_prev = x;
		StateMatrix Fk = F(x_prev, dt);
		x = f(x_prev, dt);
		P = Fk * P * Fk.transpose() + Q * dt;
	}

	// update step
	void update(const MeasurementVector& z) {
		assert(h && H);
		MeasurementVector y = z - h(x);
		MeasurementModelMatrix Hk = H(x);
		MeasurementMatrix S = Hk * P * Hk.transpose() + R;
		MeasurementModelMatrixT K = P * Hk.transpose() * S.ldlt().solve(MeasurementMatrix::Identity());

		x = x + K * y;

		if (joseph_form) {
			StateMatrix I = StateMatrix::Identity();
			StateMatrix IKH = I - K * Hk;
			P = IKH * P * IKH.transpose() + K * R * K.transpose();
		}
		else {
			P = (StateMatrix::Identity() - K * Hk) * P;
		}
	}
};