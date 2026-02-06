#pragma once

#ifdef interface
#undef interface
#endif

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

#ifndef NOMINMAX
#define NOMINMAX
#endif

#include <WinSock2.h>
#include <WS2tcpip.h>

#pragma comment(lib, "ws2_32.lib")

#include <array>
#include <atomic>
#include <thread>
#include <mutex>
#include <cstdint>


struct FanucSample {
	double t = 0.0;
	double latency = 0.0;
	std::array<double, 6> q = {};
};

class FanucUdpReceiver {
public:
	explicit FanucUdpReceiver(uint16_t port);
	~FanucUdpReceiver();

	bool start();
	void stop();

	bool getLatestSample(FanucSample& sample);

private:
	void run();

	uint16_t port_;
	std::thread th_;
	std::atomic<bool> running_{ false };

	SOCKET sock_{ INVALID_SOCKET };

	std::atomic<bool> has_{ false };

	mutable std::mutex mutex_;
	FanucSample latest_sample_{};
};