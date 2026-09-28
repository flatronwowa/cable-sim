#pragma once

#include "solver/v2/implicit_cable_state.h"

namespace cable::v2 {

struct LocalEnergy {
	double energy = 0.0;
	double gradient[9] = {};
	double hessian[81] = {};
	int dof_count = 0;
	bool valid = true;
};

LocalEnergy evaluate_stretch(const ImplicitVec3 &p_a, const ImplicitVec3 &p_b, double p_rest_length,
		double p_axial_stiffness);
LocalEnergy evaluate_bending(const ImplicitVec3 &p_a, const ImplicitVec3 &p_b, const ImplicitVec3 &p_c,
		double p_rest_voronoi_length, double p_bending_stiffness);

} // namespace cable::v2
