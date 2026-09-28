#pragma once

#include "solver/cable_params.h"
#include "solver/cable_state.h"
#include "solver/spatial_hash.h"

#include <vector>

namespace cable {

struct SelfContact {
	int segment_a = -1;
	int segment_b = -1;
	float parameter_a = 0.0f;
	float parameter_b = 0.0f;
	Vec3 normal;
	float lambda_normal = 0.0f;
};

class SelfCollisionSolver {
public:
	void reserve(int p_particle_capacity);
	float solve_positions(CableState &r_state, const CableParams &p_params);
	void solve_velocities(CableState &r_state, const CableParams &p_params, float p_h);
	int contact_count() const { return static_cast<int>(contacts.size()); }

private:
	SpatialHash hash;
	std::vector<Vec3> midpoints;
	std::vector<uint8_t> flags;
	std::vector<SelfContact> contacts;
};

} // namespace cable
