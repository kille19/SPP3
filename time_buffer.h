#pragma once
#include <deque>
#include <mutex>
#include <optional>
#include <cstddef>

#include "fusion.h"

template <typename T>

class TimeBuffer {
public:
	void push(double t, const T& value, size_t max_size = 2000)
	{
		std::lock_guard<std::mutex> lock(m_);

		if (!buffer_.empty() && t < buffer_.back().t) {
			return;
		}

		buffer_.push_back(Item{ t, value });
		while (buffer_.size() > max_size) {
			buffer_.pop_front();
		}
	}

	std::optional<T> hold(double t) const 
	{
		std::lock_guard<std::mutex> lock(m_);
		if (buffer_.empty()) {
			return std::nullopt;
		}
		if (t < buffer_.front().t) {
			return std::nullopt;
		}

		size_t i = 0;
		while (i + 1 < buffer_.size() && buffer_[i + 1].t <= t) {
			++i;
		}
		return buffer_[i].v;
	}

	std::optional<T> interpolate(double t) const
	{
		std::lock_guard<std::mutex> lock(m_);
		if (buffer_.size() < 2) {
			return std::nullopt;
		}
		if (t < buffer_.front().t || t > buffer_.back().t) {
			return std::nullopt;
		}
		size_t i = 0;
		while (i + 1 < buffer_.size() && buffer_[i + 1].t < t) {
			++i;
		}
		if (i == 0) {
			return buffer_.front().v;
		}
		if (i >= buffer_.size()) {
			return buffer_.back().v;
		}
		const auto& a = buffer_[i - 1];
		const auto& b = buffer_[i];

		const double dt = b.t - a.t;
		if (dt <= 1e-12) {
			return b.v;
		}

		const double f = (t - a.t) / dt;
		return lerp(a.v, b.v, f);
	}

	void clear() 
	{
		std::lock_guard<std::mutex> lock(m_);
		buffer_.clear();
	}

	size_t size() const 
	{
		std::lock_guard<std::mutex> lock(m_);
		return buffer_.size();
	}

private:
	struct Item {
		double t;
		T v;
	};

	static T lerp(const T& a, const T& b, double f) {
		return a + (b - a) * f;
	}

	mutable std::mutex m_;
	std::deque<Item> buffer_;
};

template<>
inline Measurement TimeBuffer<Measurement>::lerp(const Measurement& a, const Measurement& b, double f) {
	Measurement m;
	m.ax = a.ax + (b.ax - a.ax) * f;
	m.ay = a.ay + (b.ay - a.ay) * f;
	m.az = a.az + (b.az - a.az) * f;
	m.gx = a.gx + (b.gx - a.gx) * f;
	m.gy = a.gy + (b.gy - a.gy) * f;
	m.gz = a.gz + (b.gz - a.gz) * f;
	m.timestamp = a.timestamp + (b.timestamp - a.timestamp) * f;
	return m;
}