//  Copyright (c) 2003-2025 Movella Technologies B.V. or subsidiaries worldwide.
//  All rights reserved.
//  
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//  
//  1.	Redistributions of source code must retain the above copyright notice,
//  	this list of conditions, and the following disclaimer.
//  
//  2.	Redistributions in binary form must reproduce the above copyright notice,
//  	this list of conditions, and the following disclaimer in the documentation
//  	and/or other materials provided with the distribution.
//  
//  3.	Neither the names of the copyright holders nor the names of their contributors
//  	may be used to endorse or promote products derived from this software without
//  	specific prior written permission.
//  
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
//  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
//  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
//  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS 
//  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES 
//  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE 
//  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.
//  
//--------------------------------------------------------------------------------
// Public Xsens device API C++ example MTi receive data.
//--------------------------------------------------------------------------------
#include <xscontroller/xscontrol_def.h>
#include <xscontroller/xsdevice_def.h>
#include <xscontroller/xsscanner.h>
#include <xstypes/xsoutputconfigurationarray.h>
#include <xstypes/xsdatapacket.h>
#include <xstypes/xstime.h>
#include <xstypes/xsfilterprofile.h>
#include <xstypes/xsfilterprofilearray.h>
#include <xscommon/xsens_mutex.h>

// added includes
#include <iostream>
#include <list>
#include <string>
#include <atomic>
#include <csignal>
#include <Eigen/Dense>
#include <fstream>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <vector>
#include <cassert>
#include <optional>

#include "fusion.h"
#include "robot_state.h"
#include "fanuc_udp_receiver.h"
#include "time_buffer.h"
#include "fanuc_model.h"
#include "mount_calibration.h"

constexpr double PI = 3.14159265358979323846;

static uint64_t updates0 = 0, updates1 = 0;
static uint64_t calibSkips0 = 0, calibSkips1 = 0;

Journaller* gJournal = 0;

using namespace std;

// global variables
static atomic<bool> exitRequested{ false };

static void signalHandler(int)
{
	exitRequested.store(true);
	cout << "\nCTRL+C detected... Stopping the measurement..." << endl;
}

class CallbackHandler : public XsCallback
{
public:
	CallbackHandler(size_t maxBufferSize = 5)
		: m_maxNumberOfPacketsInBuffer(maxBufferSize)
		, m_numberOfPacketsInBuffer(0)
	{
	}

	virtual ~CallbackHandler() throw()
	{
	}

	bool packetAvailable() const
	{
		xsens::Lock locky(&m_mutex);
		return m_numberOfPacketsInBuffer > 0;
	}

	XsDataPacket getNextPacket()
	{
		assert(packetAvailable());
		xsens::Lock locky(&m_mutex);
		XsDataPacket oldestPacket(m_packetBuffer.front());
		m_packetBuffer.pop_front();
		--m_numberOfPacketsInBuffer;
		return oldestPacket;
	}

protected:
	void onLiveDataAvailable(XsDevice*, const XsDataPacket* packet) override
	{
		xsens::Lock locky(&m_mutex);
		assert(packet != 0);
		while (m_numberOfPacketsInBuffer >= m_maxNumberOfPacketsInBuffer)
			(void)getNextPacket();

		m_packetBuffer.push_back(*packet);
		++m_numberOfPacketsInBuffer;
		assert(m_numberOfPacketsInBuffer <= m_maxNumberOfPacketsInBuffer);
	}
private:
	mutable xsens::Mutex m_mutex;

	size_t m_maxNumberOfPacketsInBuffer;
	size_t m_numberOfPacketsInBuffer;
	list<XsDataPacket> m_packetBuffer;
};

/* ---- Structs ---- */
struct GyroCalibration
{
	bool done = false;
	Eigen::Vector3d sum = Eigen::Vector3d::Zero();
	int count = 0;
};

static void handleCalibration(GyroCalibration& calibration, Fusion& fusion, const Measurement& m)
{
	if (calibration.done)
		return;

	calibration.sum += Eigen::Vector3d(m.gx, m.gy, m.gz);
	calibration.count++;

	constexpr int N = 16000; // 1600 Hz * 10 s

	if (calibration.count >= N)
	{
		Eigen::Vector3d bias = calibration.sum / calibration.count;
		fusion.setInitialGyroBias(bias);
		calibration.done = true;

		cout << "Gyro bias calibrated (" << calibration.count << " samples): " << bias.transpose() << endl;
	}
}


void printLive(const Eigen::Vector4d& q0, const Eigen::Vector4d& q1, const Eigen::Quaterniond& qrel, double err_angle, bool calib_done)
{
	cout << "\033[6A";
	cout << "\033[2K\rIMU0 : q = ["
		<< q0(0) << ", " << q0(1) << ", " << q0(2) << ", " << q0(3) << "]\n";
	cout << "\033[2K\rIMU1 : q = ["
		<< q1(0) << ", " << q1(1) << ", " << q1(2) << ", " << q1(3) << "]\n";
	cout << "\033[2K\rRel : q = ["
		<< qrel.w() << ", " << qrel.x() << ", " << qrel.y() << ", " << qrel.z() << "]\n";
	cout << "\033[2K\rErr angle : "
		<< err_angle << "\n";
	cout << "\033[2K\rMount calib : " << (calib_done ? "done" : "in progress") << "\n";
	cout << "\033[2K\r"
		<< "updates0=" << updates0
		<< " updates1=" << updates1
		<< " calibSkips0=" << calibSkips0
		<< " calibSkips1=" << calibSkips1
		<< endl;

	cout << flush;
}

struct MountCalibration {

	vector<Eigen::Matrix3d> A;
	vector<Eigen::Matrix3d> B;

	bool done = false;
	Eigen::Matrix3d link_to_imu = Eigen::Matrix3d::Identity();

	bool have_prev = false;
	Eigen::Matrix3d prev_link = Eigen::Matrix3d::Identity();
	Eigen::Matrix3d prev_imu = Eigen::Matrix3d::Identity();

	void reset() {
		A.clear();
		B.clear();
		done = false;
		link_to_imu.setIdentity();
		have_prev = false;
		prev_link.setIdentity();
		prev_imu.setIdentity();
	}

	void pushAbs(const Eigen::Matrix3d& linkRotationAbs, const Eigen::Matrix3d& imuRotationAbs) {
		if (done) {
			return;
		}

		if (!have_prev) {
			prev_link = linkRotationAbs;
			prev_imu = imuRotationAbs;
			have_prev = true;
			return;
		}

		const Eigen::Matrix3d A_k = prev_link.transpose() * linkRotationAbs;
		const Eigen::Matrix3d B_k = prev_imu.transpose() * imuRotationAbs;

		prev_link = linkRotationAbs;
		prev_imu = imuRotationAbs;

		const double angleA = rotationAngle(A_k);
		const double angleB = rotationAngle(B_k);

		const double min_angle = 2.0 * PI / 180.0; // 2 degrees
		const double max_angle = 60.0 * PI / 180.0; // 60 degrees

		if (angleA > min_angle && angleA < max_angle && angleB > min_angle && angleB < max_angle) {
			A.push_back(A_k);
			B.push_back(B_k);
		}
	}

	bool solveIfReady(int min_pairs = 80) {
		if (done) {
			return true;
		}
		if ((int)A.size() < min_pairs) {
			return false;
		}
		Eigen::Matrix3d X;
		if (!handEye(A, B, X)) {
			return false;
		}

		link_to_imu = X;
		done = true;
		
		return true;
	}
};
//--------------------------------------------------------------------------------
int main(void)
{
	// register handler for CTRL+C signal
	signal(SIGINT, signalHandler);

	cout << "Creating XsControl object..." << endl;
	XsControl* control = XsControl::construct();
	assert(control != 0);

	// Lambda function for error handling
	auto handleError = [=](string errorString)
	{
		control->destruct();
		cout << errorString << endl;
		cout << "Press [ENTER] to continue." << endl;
		cin.get();
		return -1;
	};

	cout << "Scanning for devices..." << endl;
	XsPortInfoArray portInfoArray = XsScanner::scanPorts(XBR_2000k, 100, false);

	// Find an MTi devices
	vector<XsPortInfo> mtiPorts;
	for (auto const& portInfo : portInfoArray)
	{
		if (portInfo.deviceId().isMti() || portInfo.deviceId().isMtig() || portInfo.deviceId().isMti6X0())
		{
			mtiPorts.push_back(portInfo);
		}
	}

	if (mtiPorts.size() < 2)
		return handleError("Connect both IMU's and try again. Aborting...");

	XsPortInfo mtiPort0 = mtiPorts[0];
	XsPortInfo mtiPort1 = mtiPorts[1];

	cout << "Found IMU0 with ID: " << mtiPort0.deviceId().toString().toStdString() << " @ port: " << mtiPort0.portName().toStdString() << ", baudrate: " << mtiPort0.baudrate() << endl;
	cout << "Found IMU1 with ID: " << mtiPort1.deviceId().toString().toStdString() << " @ port: " << mtiPort1.portName().toStdString() << ", baudrate: " << mtiPort1.baudrate() << endl;

	cout << "Opening ports..." << endl;
	if (!control->openPort(mtiPort0.portName().toStdString(), mtiPort0.baudrate()))
		return handleError("Could not open port for IMU0. Aborting...");
	if (!control->openPort(mtiPort1.portName().toStdString(), mtiPort1.baudrate()))
		return handleError("Could not open port for IMU1. Aborting...");

	// Get the device objects
	XsDevice* device0 = control->device(mtiPort0.deviceId());
	XsDevice* device1 = control->device(mtiPort1.deviceId());
	assert(device0 != 0);
	assert(device1 != 0);

	cout << "IMU0: " << device0->productCode().toStdString() << ", with ID: " << device0->deviceId().toString() << " opened." << endl;
	cout << "IMU1: " << device1->productCode().toStdString() << ", with ID: " << device1->deviceId().toString() << " opened." << endl;

	// Create and attach callback handlers to each device
	CallbackHandler callback0;
	CallbackHandler callback1;
	device0->addCallbackHandler(&callback0);
	device1->addCallbackHandler(&callback1);

	//--- Put the devices into configuration mode before configuring the device ---
	// IM0:
	cout << "Putting IMU0 into configuration mode..." << endl;
	if (!device0->gotoConfig())
		return handleError("Could not put IMU0 into configuration mode. Aborting...");
	
	cout << "Selecting filter profile 'responsive' and heading 'VRUAHS'..." << endl;
	
	if (!device0->setOnboardFilterProfile("Responsive/VRUAHS"))
		return handleError("Failed to set filter profile and heading. Aborting...");

	cout << "Configuring IMU0..." << endl;

	// Important for Public XDA!
	// Call this function if you want to record a mtb file:
	device0->readEmtsAndDeviceConfiguration();

	XsOutputConfigurationArray configArray0;
	configArray0.push_back(XsOutputConfiguration(XDI_PacketCounter, 0));
	configArray0.push_back(XsOutputConfiguration(XDI_SampleTimeFine, 0));
	configArray0.push_back(XsOutputConfiguration(XDI_AccelerationHR, 2000));
	configArray0.push_back(XsOutputConfiguration(XDI_RateOfTurnHR, 1600));

	if (!device0->setOutputConfiguration(configArray0))
		return handleError("Could not configure IMU0. Aborting...");

	// IMU1:
	cout << "Putting IMU1 into configuration mode..." << endl;
	if (!device1->gotoConfig())
		return handleError("Could not put IMU1 into configuration mode. Aborting...");

	cout << "Selecting filter profile 'responsive' and heading 'VRUAHS'..." << endl;

	if (!device1->setOnboardFilterProfile("Responsive/VRUAHS"))
		return handleError("Failed to set filter profile and heading. Aborting...");

	cout << "Configuring IMU1..." << endl;

	// Important for Public XDA!
	// Call this function if you want to record a mtb file:
	device1->readEmtsAndDeviceConfiguration();

	XsOutputConfigurationArray configArray1;
	configArray1.push_back(XsOutputConfiguration(XDI_PacketCounter, 0));
	configArray1.push_back(XsOutputConfiguration(XDI_SampleTimeFine, 0));
	configArray1.push_back(XsOutputConfiguration(XDI_AccelerationHR, 2000));
	configArray1.push_back(XsOutputConfiguration(XDI_RateOfTurnHR, 1600));

	if (!device1->setOutputConfiguration(configArray1))
		return handleError("Could not configure IMU1. Aborting...");
	// --- Configuration done ---

	cout << "Creating log files..." << endl;
	string logFileName0 = "logfile_imu0.mtb";
	string logFileName1 = "logfile_imu1.mtb";
	if (device0->createLogFile(logFileName0) != XRV_OK)
		return handleError("Failed to create a log file for IMU0. Aborting...");
	if (device1->createLogFile(logFileName1) != XRV_OK)
		return handleError("Failed to create a log file for IMU1. Aborting...");
	cout << "Log files created: " << logFileName0 << " and " << logFileName1 << endl;

	cout << "Putting devices into measurement mode..." << endl;
	if (!device0->gotoMeasurement())
		return handleError("Could not put IMU0 into measurement mode. Aborting...");
	if (!device1->gotoMeasurement())
		return handleError("Could not put IMU1 into measurement mode. Aborting...");

	cout << "Starting recording..." << endl;
	if (!device0->startRecording())
		return handleError("Failed to start recording on IMU0. Aborting...");
	if (!device1->startRecording())
		return handleError("Failed to start recording on IMU1. Aborting...");

	cout << "\nMain loop. Recording data. Press CTRL+C to stop." << endl;
	cout << string(79, '-') << endl;

	TimeBuffer<Measurement> imu0Buffer;
	TimeBuffer<Measurement> imu1Buffer;
	TimeBuffer<FanucSample> fanucBuffer;

	// --- Fanuc UDP receiver ---
	FanucUdpReceiver fanucRx(5555);
	if (!fanucRx.start()) {
		cout << "Failed to start Fanuc UDP receiver. Aborting..." << endl;
		return -1;
	}

	// --- Pinocchio Model ---
	FanucModel fanucModel;
	if (!fanucModel.loadFromURDF("models/m710ic70.urdf", "base_link", "link_3", "tool0")) {
		cout << "Failed to load Fanuc model from URDF. Aborting..." << endl;
		return -1;
	}
	
	Fusion fusion0;
	Fusion fusion1;

	GyroCalibration calibration0;
	GyroCalibration calibration1;

	RobotState state;

	MountCalibration mountCalib0;
	MountCalibration mountCalib1;
	bool mountCalibStarted = false;
	double mountCalibStartTime = 0.0;

	double lastAccNorm0 = 0.0, lastAccNorm1 = 0.0;
	double lastGyrNorm0 = 0.0, lastGyrNorm1 = 0.0;

	ofstream csv("imu_fusion_output.csv");
	csv << "t;"
		<< "q0_w;q0_x;q0_y;q0_z;"
		<< "q1_w;q1_x;q1_y;q1_z;"
		<< "qrel_w;qrel_x;qrel_y;qrel_z;"
		<< "qrel_robot_w;qrel_robot_x;qrel_robot_y;qrel_robot_z;"
		<< "err_raw_deg;"
		<< "qrel_imu_cal_w;qrel_imu_cal_x;qrel_imu_cal_y;qrel_imu_cal_z;"
		<< "err_cal_deg;"
		<< "acc0_norm;acc1_norm;"
		<< "gyro0_norm;gyro1_norm;"
		<< "updates0;updates1;"
		<< "mount_pairs0;mount_pairs1"
		<< "\n";
	csv << flush;

	cout << "IMU0 : q = [0, 0, 0, 0]\n";
	cout << "IMU1 : q = [0, 0, 0, 0]\n";
	cout << "Rel  : q = [0, 0, 0, 0]\n";
	cout << "Err angle : 0\n";
	cout << "Mount calib : collecting...\n";
	cout << "updates0=0 updates1=0 calibSkips0=0 calibSkips1=0\n";
	cout << flush;

	auto nowSeconds = []() -> double {
		using clk = chrono::steady_clock;
		static const auto t0 = clk::now();
		return chrono::duration<double>(clk::now() - t0).count();
	};

	auto extractAndBuffer = [&](const XsDataPacket& packet, TimeBuffer<Measurement>& buffer) {
		Measurement measurement{};
		measurement.timestamp = nowSeconds();

		if (packet.containsAccelerationHR())
		{
			XsVector acc = packet.accelerationHR();
			measurement.ax = acc[0];
			measurement.ay = acc[1];
			measurement.az = acc[2];
		}
		else if (packet.containsCalibratedAcceleration()) // Fallback
		{
			XsVector acc = packet.calibratedAcceleration();
			measurement.ax = acc[0];
			measurement.ay = acc[1];
			measurement.az = acc[2];
		}

		// Gyroscope in rad/s
		if (packet.containsRateOfTurnHR())
		{
			XsVector gyr = packet.rateOfTurnHR();
			measurement.gx = gyr[0];
			measurement.gy = gyr[1];
			measurement.gz = gyr[2];

		}
		else if (packet.containsCalibratedGyroscopeData()) // Fallback
		{
			XsVector gyr = packet.calibratedGyroscopeData();
			measurement.gx = gyr[0];
			measurement.gy = gyr[1];
			measurement.gz = gyr[2];
		}

		buffer.push(measurement.timestamp, measurement);
	};

	static bool fusionStarted = false;
	static double lastFusionTime = 0.0;

	// changed to infinite loop with CTRL+C exit
	while (!exitRequested.load())
	{
		if (callback0.packetAvailable())
		{
			XsDataPacket packet = callback0.getNextPacket();
			extractAndBuffer(packet, imu0Buffer);
		}

		if (callback1.packetAvailable())
		{
			XsDataPacket packet = callback1.getNextPacket();
			extractAndBuffer(packet, imu1Buffer);
		}

		const double t_fuse = nowSeconds();
		double dt_fuse = fusionStarted ? (t_fuse - lastFusionTime) : (1.0 / 200.0);
		if (dt_fuse < 1e-6 || dt_fuse > 0.05) {
			dt_fuse = 1.0 / 200.0;
		}

		FanucSample js{};
		if (fanucRx.getLatestSample(js)) {
			fanucBuffer.push(t_fuse, js);
		}

		optional<Measurement> m0 = imu0Buffer.interpolate(t_fuse);
		if (!m0) {
			m0 = imu0Buffer.hold(t_fuse);
		}

		optional<Measurement> m1 = imu1Buffer.interpolate(t_fuse);
		if (!m1) {
			m1 = imu1Buffer.hold(t_fuse);
		}

		optional<FanucSample> j = fanucBuffer.hold(t_fuse);
		if (!j || !m0 || !m1) {
			XsTime::msleep(0);
			continue;
		}

		Eigen::Vector3d acc0(m0->ax, m0->ay, m0->az);
		Eigen::Vector3d acc1(m1->ax, m1->ay, m1->az);
		Eigen::Vector3d gyr0(m0->gx, m0->gy, m0->gz);
		Eigen::Vector3d gyr1(m1->gx, m1->gy, m1->gz);

		const double acc0_norm = acc0.norm();
		const double acc1_norm = acc1.norm();
		const double gyr0_norm = gyr0.norm();
		const double gyr1_norm = gyr1.norm();

		if (!calibration0.done) {
			handleCalibration(calibration0, fusion0, *m0);
		}
		if (!calibration1.done) {
			handleCalibration(calibration1, fusion1, *m1);
		}

		if (calibration0.done) {
			fusion0.update(*m0, dt_fuse);
			updates0++;
		}
		else {
			calibSkips0++;
		}

		if (calibration1.done) {
			fusion1.update(*m1, dt_fuse);
			updates1++;
		}
		else {
			calibSkips1++;
		}

		fusionStarted = true;
		lastFusionTime = t_fuse;

		state.valid = j.has_value();
		state.t = t_fuse;

		// --- Pinocchio forward kinematics ---
		Eigen::Matrix<double, 6, 1> qJ;
		for (int i = 0; i < 6; ++i) {
			qJ(i) = j->q[i];
		}
		fanucModel.setJointAngles(qJ);

		const Eigen::Quaterniond Q_base_link = fanucModel.baseToLink3Rotation().normalized();
		const Eigen::Quaterniond Q_base_tool = fanucModel.baseToToolRotation().normalized();
		Eigen::Quaterniond Qrel_robot = (Q_base_link.conjugate() * Q_base_tool).normalized();

		const Eigen::Vector4d q0v = fusion0.getQuaternion();
		const Eigen::Vector4d q1v = fusion1.getQuaternion();
		const Eigen::Quaterniond Qw_imu0(q0v(0), q0v(1), q0v(2), q0v(3));
		const Eigen::Quaterniond Qw_imu1(q1v(0), q1v(1), q1v(2), q1v(3));
		const Eigen::Quaterniond Qrel_imu_raw = (Qw_imu0.conjugate() * Qw_imu1).normalized();

		Eigen::Quaterniond Qerr_raw = (Qrel_robot.conjugate() * Qrel_imu_raw).normalized();
		double w_raw = clamp(Qerr_raw.w(), -1.0, 1.0);
		double err_angle_raw = 2.0 * acos(w_raw) * (180.0 / PI);

		if (calibration0.done && calibration1.done && !mountCalibStarted) {
			mountCalibStarted = true;
			mountCalibStartTime = t_fuse;
			mountCalib0.reset();
			mountCalib1.reset();
			cout << "Mount calibration started. Move robot through varied orientations for about 30 seconds.\n";
		}

		if (mountCalibStarted && !(mountCalib0.done && mountCalib1.done)) {
			static int decimation = 0;
			if (++decimation % 10 == 0) {
				const Eigen::Matrix3d R_link = Q_base_link.toRotationMatrix();
				const Eigen::Matrix3d R_tool = Q_base_tool.toRotationMatrix();
				const Eigen::Matrix3d R_imu0 = Qw_imu0.normalized().toRotationMatrix();
				const Eigen::Matrix3d R_imu1 = Qw_imu1.normalized().toRotationMatrix();

				mountCalib0.pushAbs(R_link, R_imu0);
				mountCalib1.pushAbs(R_tool, R_imu1);
			}

			if (!mountCalib0.done && mountCalib0.solveIfReady()) {
				cout << "Mount calibration for IMU0 done. Collected " << mountCalib0.A.size() << " motion pairs.\n";
			}

			if (!mountCalib1.done && mountCalib1.solveIfReady()) {
				cout << "Mount calibration for IMU1 done. Collected " << mountCalib1.A.size() << " motion pairs.\n";
			}

			if ((t_fuse - mountCalibStartTime) > 40.0) {
				if (!mountCalib0.done) {
					cout << "Mount calibration for IMU0 failed (not enough varied motion). pairs 0 = " << mountCalib0.A.size() << "\n";
				}
				if (!mountCalib1.done) {
					cout << "Mount calibration for IMU1 failed (not enough varied motion). pairs 1 = " << mountCalib1.A.size() << "\n";
				}
			}

		}

		bool mountDone = mountCalib0.done && mountCalib1.done;
		Eigen::Quaterniond Qrel_imu_cal = Qrel_imu_raw;
		double err_angle_cal = err_angle_raw;

		if (mountDone) {
			const Eigen::Matrix3d X0 = mountCalib0.link_to_imu;
			const Eigen::Matrix3d X1 = mountCalib1.link_to_imu;

			const Eigen::Matrix3d Rw_imu0 = Qw_imu0.normalized().toRotationMatrix();
			const Eigen::Matrix3d Rw_imu1 = Qw_imu1.normalized().toRotationMatrix();

			const Eigen::Matrix3d R_link_est = Rw_imu0 * X0.transpose();
			const Eigen::Matrix3d R_tool_est = Rw_imu1 * X1.transpose();

			const Eigen::Matrix3d Rrel_imu_est = R_link_est.transpose() * R_tool_est;
			Qrel_imu_cal = Eigen::Quaterniond(Rrel_imu_est).normalized();

			Eigen::Quaterniond Qerr_cal = (Qrel_robot.conjugate() * Qrel_imu_cal).normalized();
			double w_cal = clamp(Qerr_cal.w(), -1.0, 1.0);
			err_angle_cal = 2.0 * acos(w_cal) * (180.0 / PI);
		}

		static int logDecimation = 0;
		if (++logDecimation % 16 == 0) {

			csv << t_fuse << ";"
				<< q0v(0) << ";" << q0v(1) << ";" << q0v(2) << ";" << q0v(3) << ";"
				<< q1v(0) << ";" << q1v(1) << ";" << q1v(2) << ";" << q1v(3) << ";"
				<< Qrel_imu_raw.w() << ";" << Qrel_imu_raw.x() << ";" << Qrel_imu_raw.y() << ";" << Qrel_imu_raw.z() << ";"
				<< Qrel_robot.w() << ";" << Qrel_robot.x() << ";" << Qrel_robot.y() << ";" << Qrel_robot.z() << ";"
				<< err_angle_raw<< ";"
				<< Qrel_imu_cal.w() << ";" << Qrel_imu_cal.x() << ";" << Qrel_imu_cal.y() << ";" << Qrel_imu_cal.z() << ";"
				<< err_angle_cal << ";"
				<< acc0_norm << ";" << acc1_norm << ";"
				<< gyr0_norm << ";" << gyr1_norm << ";"
				<< updates0 << ";" << updates1 << ";"
				<< mountCalib0.A.size() << ";" << mountCalib1.A.size()
				<< "\n";
		}

		static int relativeDecimation = 0;
		if (++relativeDecimation % 2000 == 0) {
			const Eigen::Quaterniond Qrel_used = mountDone ? Qrel_imu_cal : Qrel_imu_raw;
			const double err_used = mountDone ? err_angle_cal : err_angle_raw;
			printLive(q0v, q1v, Qrel_used, err_used, mountDone);
		}

		XsTime::msleep(0);
	}

	cout << "Stopping recording..." << endl;
	if (!device0->stopRecording())
		return handleError("Failed to stop recording on IMU0. Aborting...");
	if (!device1->stopRecording())
		return handleError("Failed to stop recording on IMU1. Aborting...");

	cout << "Closing log files..." << endl;
	if (!device0->closeLogFile())
		return handleError("Failed to close log file for IMU0. Aborting...");
	if (!device1->closeLogFile())
		return handleError("Failed to close log file for IMU1. Aborting...");

	cout << "Closing ports..." << endl;
	control->closePort(mtiPort0.portName().toStdString());
	control->closePort(mtiPort1.portName().toStdString());

	cout << "Freeing XsControl object..." << endl;
	control->destruct();

	cout << "Successful exit." << endl;
	csv.close();
	fanucRx.stop();

	cout << "Press [ENTER] to continue." << endl;
	cin.get();

	return 0;
}
