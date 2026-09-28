#pragma once

#include "solver/cable_params.h"
#include "solver/cable_state.h"

namespace cable {

// Изотропная DER-кривизна Bergou et al.: |kb| = 2 tan(theta / 2).
// Material frame не нужен, пока круглое сечение и свободный twist позволяют
// моделировать только динамику центральной линии.
bool solve_der_bend(Vec3 &r_prev, Vec3 &r_mid, Vec3 &r_next, float p_w_prev, float p_w_mid,
		float p_w_next, float p_rest_prev, float p_rest_next, float p_compliance,
		float p_h, float &r_lambda);

bool solve_anchor_bend(Vec3 &r_prev, Vec3 &r_mid, Vec3 &r_next, float p_w_prev, float p_w_mid,
		float p_w_next, float p_arm, float p_compliance, float p_h, float &r_lambda);

void solve_bend(CableState &r_state, const CableParams &p_params, float p_h);

// Односторонний угловой constraint, согласованный с DER-кривизной:
// theta_max = 2 atan(dual_length / (2 R)).
void solve_min_bend_radius(CableState &r_state, const CableParams &p_params);

float normalized_bending_energy(const CableState &p_state);

} // namespace cable
