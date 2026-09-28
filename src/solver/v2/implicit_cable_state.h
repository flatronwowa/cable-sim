#pragma once

#include "solver/cable_state.h"

#include <cstddef>
#include <vector>

namespace cable::v2 {

struct ImplicitVec3 {
	double x = 0.0;
	double y = 0.0;
	double z = 0.0;

	ImplicitVec3() = default;
	ImplicitVec3(double p_x, double p_y, double p_z) : x(p_x), y(p_y), z(p_z) {}
	explicit ImplicitVec3(const Vec3 &p_value) : x(p_value.x), y(p_value.y), z(p_value.z) {}

	ImplicitVec3 operator+(const ImplicitVec3 &p_other) const {
		return ImplicitVec3(x + p_other.x, y + p_other.y, z + p_other.z);
	}
	ImplicitVec3 operator-(const ImplicitVec3 &p_other) const {
		return ImplicitVec3(x - p_other.x, y - p_other.y, z - p_other.z);
	}
	ImplicitVec3 operator*(double p_scale) const {
		return ImplicitVec3(x * p_scale, y * p_scale, z * p_scale);
	}
};

struct ImplicitCableState {
	std::vector<ImplicitVec3> accepted_position;
	std::vector<ImplicitVec3> trial_position;
	std::vector<ImplicitVec3> velocity;
	std::vector<double> mass;
	std::vector<double> rest_length;
	std::vector<uint8_t> pinned;

	int count = 0;
	size_t capacity_growths = 0;
	size_t peak_capacity_bytes = 0;

	void initialize(const CableState &p_legacy, double p_mass_per_meter);
	void update_peak_capacity();
	bool is_initialized() const { return count >= 2; }
};

} // namespace cable::v2
