#include "solver/constraints_bend.h"

namespace cable {

namespace {

struct BendGeometry {
	float angle = 0.0f;
	float integrated_curvature = 0.0f;
	Vec3 gradient_prev;
	Vec3 gradient_mid;
	Vec3 gradient_next;
};

bool angle_geometry(const Vec3 &p_prev, const Vec3 &p_mid, const Vec3 &p_next,
		bool p_allow_fold, BendGeometry &r_geometry) {
	const Vec3 edge_prev = p_mid - p_prev;
	const Vec3 edge_next = p_next - p_mid;
	const float length_prev = length(edge_prev);
	const float length_next = length(edge_next);
	if (length_prev < EPS_LENGTH || length_next < EPS_LENGTH) {
		return false;
	}

	const Vec3 tangent_prev = edge_prev * (1.0f / length_prev);
	const Vec3 tangent_next = edge_next * (1.0f / length_next);
	const float cosine = clampf(dot(tangent_prev, tangent_next), -1.0f, 1.0f);
	const Vec3 crossed = cross(tangent_prev, tangent_next);
	const float sine = length(crossed);

	Vec3 bend_normal;
	if (sine > EPS_ANGLE) {
		bend_normal = crossed * (1.0f / sine);
	} else if (cosine < 0.0f && p_allow_fold) {
		bend_normal = any_perpendicular(tangent_prev);
	} else {
		return false;
	}

	r_geometry.angle = std::atan2(sine, cosine);
	if (cosine < 0.0f && sine <= EPS_ANGLE) {
		r_geometry.angle = PI_F;
	}
	r_geometry.integrated_curvature = 2.0f * std::tan(0.5f * r_geometry.angle);

	r_geometry.gradient_prev = cross(bend_normal, tangent_prev) * (1.0f / length_prev);
	r_geometry.gradient_next = cross(bend_normal, tangent_next) * (1.0f / length_next);
	r_geometry.gradient_mid = -(r_geometry.gradient_prev + r_geometry.gradient_next);
	return true;
}

float weighted_gradient_sum(const BendGeometry &p_geometry,
		float p_w_prev, float p_w_mid, float p_w_next) {
	return p_w_prev * length_squared(p_geometry.gradient_prev) +
			p_w_mid * length_squared(p_geometry.gradient_mid) +
			p_w_next * length_squared(p_geometry.gradient_next);
}

} // namespace

bool solve_der_bend(Vec3 &r_prev, Vec3 &r_mid, Vec3 &r_next, float p_w_prev, float p_w_mid,
		float p_w_next, float p_rest_prev, float p_rest_next, float p_compliance,
		float p_h, float &r_lambda) {
	BendGeometry geometry;
	if (!angle_geometry(r_prev, r_mid, r_next, false, geometry)) {
		r_lambda = 0.0f;
		return false;
	}

	const float dual_length = 0.5f * (p_rest_prev + p_rest_next);
	const float h2 = p_h * p_h;
	if (dual_length < EPS_LENGTH || h2 < EPS_DENOM) {
		return false;
	}

	const float derivative = 2.0f / maxf(1.0f + std::cos(geometry.angle), EPS_ANGLE);
	geometry.gradient_prev *= derivative;
	geometry.gradient_mid *= derivative;
	geometry.gradient_next *= derivative;

	const float alpha_tilde = p_compliance * dual_length / h2;
	const float w_sum = weighted_gradient_sum(geometry, p_w_prev, p_w_mid, p_w_next);
	if (w_sum + alpha_tilde < EPS_DENOM) {
		return false;
	}

	const float delta_lambda =
			(-geometry.integrated_curvature - alpha_tilde * r_lambda) / (w_sum + alpha_tilde);
	float limited_delta_lambda = delta_lambda;
	const float max_correction = 0.01f * minf(p_rest_prev, p_rest_next);
	const float correction = maxf(
			length(geometry.gradient_prev * (p_w_prev * delta_lambda)),
			maxf(length(geometry.gradient_mid * (p_w_mid * delta_lambda)),
					length(geometry.gradient_next * (p_w_next * delta_lambda))));
	if (correction > max_correction) {
		limited_delta_lambda *= max_correction / correction;
	}

	r_lambda += limited_delta_lambda;
	r_prev += geometry.gradient_prev * (p_w_prev * limited_delta_lambda);
	r_mid += geometry.gradient_mid * (p_w_mid * limited_delta_lambda);
	r_next += geometry.gradient_next * (p_w_next * limited_delta_lambda);
	return true;
}

bool solve_anchor_bend(Vec3 &r_prev, Vec3 &r_mid, Vec3 &r_next, float p_w_prev, float p_w_mid,
		float p_w_next, float p_arm, float p_compliance, float p_h, float &r_lambda) {
	BendGeometry geometry;
	if (!angle_geometry(r_prev, r_mid, r_next, true, geometry)) {
		r_lambda = 0.0f;
		return false;
	}

	const float h2 = p_h * p_h;
	if (p_arm < EPS_LENGTH || h2 < EPS_DENOM) {
		return false;
	}

	const float alpha_tilde = p_compliance * p_arm / h2;
	const float w_sum = weighted_gradient_sum(geometry, p_w_prev, p_w_mid, p_w_next);
	if (w_sum + alpha_tilde < EPS_DENOM) {
		return false;
	}

	const float delta_lambda = (-geometry.angle - alpha_tilde * r_lambda) / (w_sum + alpha_tilde);
	r_lambda += delta_lambda;
	r_prev += geometry.gradient_prev * (p_w_prev * delta_lambda);
	r_mid += geometry.gradient_mid * (p_w_mid * delta_lambda);
	r_next += geometry.gradient_next * (p_w_next * delta_lambda);
	return true;
}

void solve_bend(CableState &r_state, const CableParams &p_params, float p_h) {
	if (r_state.count < 3) {
		return;
	}

	for (int i = 1; i < r_state.count - 1; ++i) {
		solve_der_bend(r_state.position[i - 1], r_state.position[i], r_state.position[i + 1],
				r_state.solve_inv_mass(i - 1), r_state.solve_inv_mass(i), r_state.solve_inv_mass(i + 1),
				r_state.rest_length[i - 1], r_state.rest_length[i],
				p_params.bending_compliance, p_h, r_state.lambda_bend[i]);
	}
}

void solve_min_bend_radius(CableState &r_state, const CableParams &p_params) {
	if (r_state.count < 3 || p_params.minimum_bend_radius < EPS_LENGTH) {
		return;
	}
	const float radius = p_params.minimum_bend_radius;

	for (int i = 1; i < r_state.count - 1; ++i) {
		const float dual_length = 0.5f * (r_state.rest_length[i - 1] + r_state.rest_length[i]);
		if (dual_length < EPS_LENGTH) {
			continue;
		}

		BendGeometry geometry;
		if (!angle_geometry(r_state.position[i - 1], r_state.position[i],
					r_state.position[i + 1], true, geometry)) {
			continue;
		}
		const float max_angle = minf(2.0f * std::atan(dual_length / (2.0f * radius)), PI_F - EPS_ANGLE);
		if (geometry.angle <= max_angle) {
			continue;
		}

		const float w_prev = r_state.solve_inv_mass(i - 1);
		const float w_mid = r_state.solve_inv_mass(i);
		const float w_next = r_state.solve_inv_mass(i + 1);
		const float w_sum = weighted_gradient_sum(geometry, w_prev, w_mid, w_next);
		if (w_sum < EPS_DENOM) {
			continue;
		}

		float delta_lambda = -(geometry.angle - max_angle) / w_sum;
		const float max_correction =
				0.001f * minf(r_state.rest_length[i - 1], r_state.rest_length[i]);
		const float correction = maxf(
				length(geometry.gradient_prev * (w_prev * delta_lambda)),
				maxf(length(geometry.gradient_mid * (w_mid * delta_lambda)),
						length(geometry.gradient_next * (w_next * delta_lambda))));
		if (correction > max_correction) {
			delta_lambda *= max_correction / correction;
		}

		r_state.project(i - 1, geometry.gradient_prev * (w_prev * delta_lambda));
		r_state.project(i, geometry.gradient_mid * (w_mid * delta_lambda));
		r_state.project(i + 1, geometry.gradient_next * (w_next * delta_lambda));
	}
}

float normalized_bending_energy(const CableState &p_state) {
	float total = 0.0f;
	for (int i = 1; i < p_state.count - 1; ++i) {
		BendGeometry geometry;
		if (!angle_geometry(p_state.position[i - 1], p_state.position[i],
					p_state.position[i + 1], false, geometry)) {
			continue;
		}
		const float dual_length = 0.5f * (p_state.rest_length[i - 1] + p_state.rest_length[i]);
		if (dual_length > EPS_LENGTH) {
			total += geometry.integrated_curvature * geometry.integrated_curvature / dual_length;
		}
	}
	return total;
}

} // namespace cable
