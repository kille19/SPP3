#pragma once
#include <Eigen/Dense>
#include "ekf.h"

struct Measurement {
	double ax = 0.0, ay = 0.0, az = 0.0; // Accelerometer readings
	double gx = 0.0, gy = 0.0, gz = 0.0; // Gyroscope readings
	double timestamp = 0.0;
};

class Fusion {
public:
	Fusion();
	
	void update(const Measurement& measurement, double dt);

	Eigen::Vector4d getQuaternion() const;

	void setInitialGyroBias(const Eigen::Vector3d& bias);

	void setProcessNoiseCovariance(const EKF<7, 3>::StateMatrix& Q);
	void setMeasurementNoiseCovariance(const EKF<7, 3>::MeasurementMatrix& R);

private:
	EKF<7, 3> filter_; // 4 orientation (quaternion) + 3 gyro biases = 7 state variables, 3 measurement variables (acceleration)

	static Eigen::Vector4d normalizeQuaternion(const Eigen::Vector4d& q);
	static Eigen::Vector4d multiplyQuaternions(const Eigen::Vector4d& q, const Eigen::Vector4d& r);

	static EKF<7, 3>::StateVector stateTransitionFunction(const EKF<7, 3>::StateVector& x, double dt, const Eigen::Vector3d& gyro_rad_s);
	static EKF<7, 3>::StateMatrix stateTransitionJacobian(const EKF<7, 3>::StateVector& x, double dt, const Eigen::Vector3d& gyro_rad_s);
};