#include "fanuc_udp_receiver.h"
#include <cstring>
#include <Windows.h>

static constexpr size_t PACKET_SIZE = 64;

FanucUdpReceiver::FanucUdpReceiver(uint16_t port)
	: port_(port)
{
}

FanucUdpReceiver::~FanucUdpReceiver()
{
	stop();
}

bool FanucUdpReceiver::start()
{
	if (running_.load()) {
		return true;
	}

	WSADATA wsaData;
	if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
		return false;
	}

	sock_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (sock_ == INVALID_SOCKET) {
		WSACleanup();
		return false;
	}

	sockaddr_in addr{};
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = htonl(INADDR_ANY);
	addr.sin_port = htons(port_);

	if (bind(sock_, (sockaddr*)&addr, sizeof(addr)) == SOCKET_ERROR) {
		closesocket(sock_);
		sock_ = INVALID_SOCKET;
		WSACleanup();
		return false;
	}

	u_long mode = 1;
	ioctlsocket(sock_, FIONBIO, &mode);

	running_.store(true);
	th_ = std::thread(&FanucUdpReceiver::run, this);
	return true;
}

void FanucUdpReceiver::stop()
{
	if (!running_.exchange(false)) {
		return;
	}
	
	if (th_.joinable()) {
		th_.join();
	}

	if (sock_ != INVALID_SOCKET) {
		closesocket(sock_);
		sock_ = INVALID_SOCKET;
	}

	WSACleanup();
}

void FanucUdpReceiver::run()
{
	char buffer[PACKET_SIZE];

	while (running_.load()) {
		sockaddr_in from_addr{};
		int from_len = sizeof(from_addr);
		int ret = recvfrom(sock_, buffer, PACKET_SIZE, 0, (sockaddr*)&from_addr, &from_len);
		if (ret == PACKET_SIZE) {
			FanucSample sample{};
			std::memcpy(&sample.t, buffer + 0, 8);
			std::memcpy(&sample.latency, buffer + 8, 8);
			std::memcpy(sample.q.data(), buffer + 16, 48);
			{
				std::lock_guard<std::mutex> lock(mutex_);
				latest_sample_ = sample;
				has_.store(true, std::memory_order_release);
			}
		} 
		Sleep(1);
	}
}

bool FanucUdpReceiver::getLatestSample(FanucSample& sample)
{
	if (!has_.exchange(false, std::memory_order_acq_rel)) {
		return false;
	}
	std::lock_guard<std::mutex> lock(mutex_);
	sample = latest_sample_;
	return true;
}