#include "fusion.h"
#include <cmath>
#include <Eigen/Geometry>

constexpr double PI = 3.14159265358979323846;

Fusion::Fusion()
{
	filter_.x.setZero(); // initialize state vector to zero
	filter_.x(0) = 1.0; // qw

	filter_.P = EKF<7, 3>::StateMatrix::Identity() * 0.1; // Initial covariance
	filter_.Q = EKF<7, 3>::StateMatrix::Identity() * 1e-6; // Process noise covariance
	filter_.R = EKF<7, 3>::MeasurementMatrix::Identity() * 1e-3; // Measurement noise covariance

	filter_.h = [&](const EKF<7, 3>::StateVector& x) {
		Eigen::Vector4d q(x(0), x(1), x(2), x(3));
		q = normalizeQuaternion(q);

		Eigen::Quaterniond quat(q(0), q(1), q(2), q(3));
		Eigen::Matrix3d R = quat.toRotationMatrix();

		Eigen::Vector3d g_world(0.0, 0.0, -1.0);
		Eigen::Vector3d g_body = R.transpose() * g_world;

		EKF<7, 3>::MeasurementVector h;
		h << g_body(0), g_body(1), g_body(2);
		return h;
		};

	filter_.H = [&](const EKF<7, 3>::StateVector& x) {
		EKF<7, 3>::MeasurementModelMatrix H = EKF<7, 3>::MeasurementModelMatrix::Zero();
		auto h_of_x = [&](const EKF<7, 3>::StateVector& x_inner) -> EKF<7, 3>::MeasurementVector {
			return filter_.h(x_inner);
			};

		for (int i = 0; i < 7; ++i) {
			EKF<7, 3>::StateVector x_plus = x;
			EKF<7, 3>::StateVector x_minus = x;

			x_plus(i) += 1e-6;
			x_minus(i) -= 1e-6;

			EKF<7, 3>::MeasurementVector h_plus = h_of_x(x_plus);
			EKF<7, 3>::MeasurementVector h_minus = h_of_x(x_minus);

			H.col(i) = (h_plus - h_minus) / (2e-6);
		}
		return H;
	};
}

void Fusion::setProcessNoiseCovariance(const EKF<7, 3>::StateMatrix& Q) {
	filter_.Q = Q;
}

void Fusion::setMeasurementNoiseCovariance(const EKF<7, 3>::MeasurementMatrix& R) {
	filter_.R = R;
}

void Fusion::setInitialGyroBias(const Eigen::Vector3d& bias) {
	filter_.x(4) = bias(0);
	filter_.x(5) = bias(1);
	filter_.x(6) = bias(2);
}

Eigen::Vector4d Fusion::normalizeQuaternion(const Eigen::Vector4d& q) {
    double n = q.norm();
    if (n <= 0.0) {
        return Eigen::Vector4d(1, 0, 0, 0);
    }
	return q / n;
}

Eigen::Vector4d Fusion::multiplyQuaternions(const Eigen::Vector4d& q, const Eigen::Vector4d& r) {
	double w1 = q(0), x1 = q(1), y1 = q(2), z1 = q(3);
	double w2 = r(0), x2 = r(1), y2 = r(2), z2 = r(3);

	return Eigen::Vector4d(
		w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
		w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
		w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
		w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
	);
}

static Eigen::Vector4d deltaQuat(const Eigen::Vector3d& w_rad_s, double dt) {
	double theta = w_rad_s.norm() * dt;
	if (theta < 1e-12) {
		return Eigen::Vector4d(1, 0, 0, 0);
	}

	Eigen::Vector3d axis = w_rad_s.normalized();
	double half = theta * 0.5;
	double sin = std::sin(half);

	return Eigen::Vector4d(std::cos(half), axis.x() * sin, axis.y() * sin, axis.z() * sin);
}

EKF<7, 3>::StateVector Fusion::stateTransitionFunction(const EKF<7, 3>::StateVector& x, double dt, const Eigen::Vector3d& gyro_rad_s) {
	EKF<7, 3>::StateVector x_pred = x;

	// Extract quaternion and biases
	Eigen::Vector4d q(x(0), x(1), x(2), x(3));
	q = normalizeQuaternion(q);

	Eigen::Vector3d b(x(4), x(5), x(6));

	Eigen::Vector3d w = gyro_rad_s - b;

	// Quaternion derivative
	Eigen::Vector4d dq = deltaQuat(w, dt);
	dq = normalizeQuaternion(dq);

	Eigen::Vector4d q_next = multiplyQuaternions(q, dq);
	q_next = normalizeQuaternion(q_next);

	x_pred(0) = q_next(0);
	x_pred(1) = q_next(1);
	x_pred(2) = q_next(2);
	x_pred(3) = q_next(3);

	return x_pred;
}

EKF<7, 3>::StateMatrix Fusion::stateTransitionJacobian(const EKF<7, 3>::StateVector& x, double dt, const Eigen::Vector3d& gyro_rad_s) {
	EKF<7, 3>::StateMatrix J = EKF<7, 3>::StateMatrix::Zero();

	for (int i = 0; i < 7; ++i) {
		EKF<7, 3>::StateVector x_plus = x;
		EKF<7, 3>::StateVector x_minus = x;

		x_plus(i) += 1e-6;
		x_minus(i) -= 1e-6;

		EKF<7, 3>::StateVector f_plus = stateTransitionFunction(x_plus, dt, gyro_rad_s);
		EKF<7, 3>::StateVector f_minus = stateTransitionFunction(x_minus, dt, gyro_rad_s);

		J.col(i) = (f_plus - f_minus) / (2e-6);
	}
	return J;
}

void Fusion::update(const Measurement& measurement, double dt) {
	Eigen::Vector3d gyro_rad_s(measurement.gx, measurement.gy, measurement.gz);

	filter_.f = [&](const EKF<7, 3>::StateVector& x, double dt_in) {
		return stateTransitionFunction(x, dt_in, gyro_rad_s);
	};

	filter_.F = [&](const EKF<7, 3>::StateVector& x, double dt_in) {
		return stateTransitionJacobian(x, dt_in, gyro_rad_s);
	};

	filter_.predict(dt);

	Eigen::Vector3d a(measurement.ax, measurement.ay, measurement.az);
	double a_norm = a.norm();

	if (a_norm > 1e-6) {
		double g = 9.81;
		if (std::abs(a_norm - g) < 2.0) {
			Eigen::Vector3d a_unit = a / a_norm;
			EKF<7, 3>::MeasurementVector z;
			z << a_unit(0), a_unit(1), a_unit(2);

			filter_.update(z);

			Eigen::Vector4d q_new(filter_.x(0), filter_.x(1), filter_.x(2), filter_.x(3));
			q_new = normalizeQuaternion(q_new);
			filter_.x(0) = q_new(0);
			filter_.x(1) = q_new(1);
			filter_.x(2) = q_new(2);
			filter_.x(3) = q_new(3);
		}
	}
}

Eigen::Vector4d Fusion::getQuaternion() const {
	return Eigen::Vector4d(filter_.x(0), filter_.x(1), filter_.x(2), filter_.x(3));
}
