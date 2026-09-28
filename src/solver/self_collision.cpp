#include "solver/self_collision.h"

namespace cable {

namespace {

struct ClosestSegments {
	float a = 0.0f;
	float b = 0.0f;
	Vec3 point_a;
	Vec3 point_b;
};

ClosestSegments closest_segments(const Vec3 &p_a0, const Vec3 &p_a1,
		const Vec3 &p_b0, const Vec3 &p_b1) {
	const Vec3 edge_a = p_a1 - p_a0;
	const Vec3 edge_b = p_b1 - p_b0;
	const Vec3 offset = p_a0 - p_b0;
	const float length_a2 = length_squared(edge_a);
	const float length_b2 = length_squared(edge_b);
	const float edge_b_offset = dot(edge_b, offset);

	ClosestSegments result;
	if (length_a2 < EPS_DENOM && length_b2 < EPS_DENOM) {
		result.point_a = p_a0;
		result.point_b = p_b0;
		return result;
	}
	if (length_a2 < EPS_DENOM) {
		result.b = clampf(edge_b_offset / length_b2, 0.0f, 1.0f);
	} else {
		const float edge_a_offset = dot(edge_a, offset);
		if (length_b2 < EPS_DENOM) {
			result.a = clampf(-edge_a_offset / length_a2, 0.0f, 1.0f);
		} else {
			const float edges_dot = dot(edge_a, edge_b);
			const float denominator = length_a2 * length_b2 - edges_dot * edges_dot;
			if (denominator > EPS_DENOM * maxf(length_a2 * length_b2, 1.0f)) {
				result.a = clampf((edges_dot * edge_b_offset - edge_a_offset * length_b2) /
						denominator, 0.0f, 1.0f);
			}
			result.b = (edges_dot * result.a + edge_b_offset) / length_b2;
			if (result.b < 0.0f) {
				result.b = 0.0f;
				result.a = clampf(-edge_a_offset / length_a2, 0.0f, 1.0f);
			} else if (result.b > 1.0f) {
				result.b = 1.0f;
				result.a = clampf((edges_dot - edge_a_offset) / length_a2, 0.0f, 1.0f);
			}
		}
	}

	result.point_a = lerp(p_a0, p_a1, result.a);
	result.point_b = lerp(p_b0, p_b1, result.b);
	return result;
}

Vec3 contact_normal(const CableState &p_state, int p_segment_a, int p_segment_b,
		const ClosestSegments &p_closest) {
	const Vec3 separation = p_closest.point_a - p_closest.point_b;
	const float separation_length = length(separation);
	if (separation_length > EPS_LENGTH) {
		return separation * (1.0f / separation_length);
	}

	const Vec3 previous_a = lerp(p_state.prev_position[p_segment_a],
			p_state.prev_position[p_segment_a + 1], p_closest.a);
	const Vec3 previous_b = lerp(p_state.prev_position[p_segment_b],
			p_state.prev_position[p_segment_b + 1], p_closest.b);
	const Vec3 previous_separation = previous_a - previous_b;
	if (length_squared(previous_separation) > EPS_NORMAL) {
		return safe_normalize(previous_separation);
	}

	const Vec3 edge_a = p_state.position[p_segment_a + 1] - p_state.position[p_segment_a];
	const Vec3 edge_b = p_state.position[p_segment_b + 1] - p_state.position[p_segment_b];
	const Vec3 crossed = cross(edge_a, edge_b);
	if (length_squared(crossed) > EPS_NORMAL) {
		return safe_normalize(crossed);
	}

	const Vec3 tangent = safe_normalize(length_squared(edge_a) > length_squared(edge_b) ? edge_a : edge_b);
	const Vec3 midpoint_delta = (p_state.position[p_segment_a] + p_state.position[p_segment_a + 1]) -
			(p_state.position[p_segment_b] + p_state.position[p_segment_b + 1]);
	const Vec3 perpendicular = midpoint_delta - tangent * dot(midpoint_delta, tangent);
	if (length_squared(perpendicular) > EPS_NORMAL) {
		return safe_normalize(perpendicular);
	}
	return any_perpendicular(tangent);
}

float contact_inverse_mass(const CableState &p_state, int p_segment_a, int p_segment_b,
		float p_a, float p_b) {
	const float a0 = 1.0f - p_a;
	const float b0 = 1.0f - p_b;
	return p_state.solve_inv_mass(p_segment_a) * a0 * a0 +
			p_state.solve_inv_mass(p_segment_a + 1) * p_a * p_a +
			p_state.solve_inv_mass(p_segment_b) * b0 * b0 +
			p_state.solve_inv_mass(p_segment_b + 1) * p_b * p_b;
}

void apply_velocity_impulse(CableState &r_state, const SelfContact &p_contact, const Vec3 &p_impulse) {
	const float a0 = 1.0f - p_contact.parameter_a;
	const float b0 = 1.0f - p_contact.parameter_b;
	r_state.velocity[p_contact.segment_a] +=
			p_impulse * (r_state.solve_inv_mass(p_contact.segment_a) * a0);
	r_state.velocity[p_contact.segment_a + 1] +=
			p_impulse * (r_state.solve_inv_mass(p_contact.segment_a + 1) * p_contact.parameter_a);
	r_state.velocity[p_contact.segment_b] -=
			p_impulse * (r_state.solve_inv_mass(p_contact.segment_b) * b0);
	r_state.velocity[p_contact.segment_b + 1] -=
			p_impulse * (r_state.solve_inv_mass(p_contact.segment_b + 1) * p_contact.parameter_b);
}

} // namespace

void SelfCollisionSolver::reserve(int p_particle_capacity) {
	const int segment_capacity = p_particle_capacity > 1 ? p_particle_capacity - 1 : 1;
	hash.reserve(segment_capacity);
	midpoints.assign(segment_capacity, Vec3());
	flags.assign(segment_capacity, 0);
	contacts.reserve(static_cast<size_t>(segment_capacity) * 4u);
}

float SelfCollisionSolver::solve_positions(CableState &r_state, const CableParams &p_params) {
	contacts.clear();
	const int segment_count = r_state.segment_count();
	const float target = 2.0f * p_params.cable_radius;
	if (segment_count < 3 || target < EPS_LENGTH) {
		return 0.0f;
	}

	float max_segment_length = 0.0f;
	for (int i = 0; i < segment_count; ++i) {
		midpoints[i] = (r_state.position[i] + r_state.position[i + 1]) * 0.5f;
		max_segment_length = maxf(max_segment_length,
				distance(r_state.position[i], r_state.position[i + 1]));
	}
	hash.build(midpoints.data(), flags.data(), segment_count, 0,
			maxf(max_segment_length + target, EPS_LENGTH));

	float max_penetration = 0.0f;
	for (int i = 0; i < segment_count; ++i) {
		hash.for_each_neighbor(midpoints[i], [&](int j) {
			if (j <= i + 1) {
				return;
			}
			if (r_state.is_asleep(i) && r_state.is_asleep(i + 1) &&
					r_state.is_asleep(j) && r_state.is_asleep(j + 1)) {
				return;
			}

			const ClosestSegments closest = closest_segments(
					r_state.position[i], r_state.position[i + 1],
					r_state.position[j], r_state.position[j + 1]);
			const float separation = distance(closest.point_a, closest.point_b);
			if (separation >= target + p_params.collision_margin) {
				return;
			}

			if (r_state.is_asleep(i) || r_state.is_asleep(i + 1)) {
				r_state.wake_window(i);
			}
			if (r_state.is_asleep(j) || r_state.is_asleep(j + 1)) {
				r_state.wake_window(j);
			}

			const Vec3 normal = contact_normal(r_state, i, j, closest);
			const float w_sum = contact_inverse_mass(r_state, i, j, closest.a, closest.b);
			if (length_squared(normal) < EPS_NORMAL || w_sum < EPS_DENOM) {
				return;
			}

			SelfContact contact;
			contact.segment_a = i;
			contact.segment_b = j;
			contact.parameter_a = closest.a;
			contact.parameter_b = closest.b;
			contact.normal = normal;

			if (separation < target) {
				const float penetration = target - separation;
				contact.lambda_normal = penetration / w_sum;
				max_penetration = maxf(max_penetration, penetration);
				const float a0 = 1.0f - closest.a;
				const float b0 = 1.0f - closest.b;
				r_state.project(i, normal * (r_state.solve_inv_mass(i) * a0 * contact.lambda_normal));
				r_state.project(i + 1, normal * (r_state.solve_inv_mass(i + 1) * closest.a * contact.lambda_normal));
				r_state.project(j, normal * (-r_state.solve_inv_mass(j) * b0 * contact.lambda_normal));
				r_state.project(j + 1, normal * (-r_state.solve_inv_mass(j + 1) * closest.b * contact.lambda_normal));
			}
			contacts.push_back(contact);
		});
	}
	return max_penetration;
}

void SelfCollisionSolver::solve_velocities(CableState &r_state, const CableParams &p_params, float p_h) {
	if (p_h < EPS_LENGTH) {
		return;
	}
	for (const SelfContact &contact : contacts) {
		const float w_sum = contact_inverse_mass(r_state, contact.segment_a, contact.segment_b,
				contact.parameter_a, contact.parameter_b);
		if (w_sum < EPS_DENOM) {
			continue;
		}

		const Vec3 velocity_a = lerp(r_state.velocity[contact.segment_a],
				r_state.velocity[contact.segment_a + 1], contact.parameter_a);
		const Vec3 velocity_b = lerp(r_state.velocity[contact.segment_b],
				r_state.velocity[contact.segment_b + 1], contact.parameter_b);
		Vec3 relative = velocity_a - velocity_b;
		const float normal_speed = dot(relative, contact.normal);
		float normal_impulse = contact.lambda_normal / p_h;
		if (normal_speed < 0.0f) {
			const float impulse = -normal_speed / w_sum;
			normal_impulse += impulse;
			apply_velocity_impulse(r_state, contact, contact.normal * impulse);
			relative -= contact.normal * normal_speed;
		}

		const Vec3 tangent_velocity = relative - contact.normal * dot(relative, contact.normal);
		const float tangent_speed = length(tangent_velocity);
		if (tangent_speed < EPS_LENGTH || normal_impulse <= 0.0f) {
			continue;
		}
		const float required = tangent_speed / w_sum;
		const float static_limit = p_params.static_friction * normal_impulse;
		const float impulse = required <= static_limit
				? required
				: minf(required, p_params.dynamic_friction * normal_impulse);
		apply_velocity_impulse(r_state, contact,
				tangent_velocity * (-impulse / tangent_speed));
	}
}

} // namespace cable
