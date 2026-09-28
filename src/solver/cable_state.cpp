#include "solver/cable_state.h"

namespace cable {

void CableState::allocate(int p_capacity) {
	if (p_capacity < 2) {
		p_capacity = 2;
	}
	position.assign(p_capacity, Vec3());
	prev_position.assign(p_capacity, Vec3());
	frame_position.assign(p_capacity, Vec3());
	velocity.assign(p_capacity, Vec3());
	projection_delta.assign(p_capacity, Vec3());
	inv_mass.assign(p_capacity, 0.0f);
	flags.assign(p_capacity, 0);
	sleep_counter.assign(p_capacity, 0);
	rest_length.assign(p_capacity, 0.0f);
	arc_from_start.assign(p_capacity, 0.0f);
	lambda_stretch.assign(p_capacity, 0.0f);
	lambda_bend.assign(p_capacity, 0.0f);
	contacts.assign(p_capacity, ContactAnchor());
	segment_contacts.assign(p_capacity, SegmentContact());
	count = 0;
}

void CableState::append(const Vec3 &p_position, float p_inv_mass) {
	if (count >= capacity()) {
		return;
	}
	const int i = count;
	position[i] = p_position;
	prev_position[i] = p_position;
	frame_position[i] = p_position;
	velocity[i] = Vec3();
	projection_delta[i] = Vec3();
	inv_mass[i] = p_inv_mass;
	flags[i] = 0;
	sleep_counter[i] = 0;
	rest_length[i] = 0.0f;
	arc_from_start[i] = 0.0f;
	lambda_stretch[i] = 0.0f;
	lambda_bend[i] = 0.0f;
	contacts[i].clear();
	segment_contacts[i].clear();
	count = i + 1;
}

void CableState::wake(int i) {
	if (i < 0 || i >= count) {
		return;
	}
	flags[i] &= static_cast<uint8_t>(~PARTICLE_ASLEEP);
	sleep_counter[i] = 0;
}

void CableState::wake_window(int i) {
	const int from = i - WAKE_WINDOW < 0 ? 0 : i - WAKE_WINDOW;
	const int to = i + WAKE_WINDOW >= count ? count - 1 : i + WAKE_WINDOW;
	for (int k = from; k <= to; ++k) {
		wake(k);
	}
}

float CableState::deployed_length() const {
	float total = 0.0f;
	for (int i = 0; i < segment_count(); ++i) {
		total += rest_length[i];
	}
	return total;
}

float CableState::polyline_length() const {
	float total = 0.0f;
	for (int i = 0; i < segment_count(); ++i) {
		total += distance(position[i], position[i + 1]);
	}
	return total;
}

Bounds CableState::compute_bounds(float p_radius) const {
	Bounds b;
	for (int i = 0; i < count; ++i) {
		b.expand(position[i]);
	}
	if (b.is_valid()) {
		b.grow(p_radius);
	}
	return b;
}

void CableState::recompute_arc_lengths() {
	if (count == 0) {
		return;
	}
	arc_from_start[0] = 0.0f;
	for (int i = 1; i < count; ++i) {
		arc_from_start[i] = arc_from_start[i - 1] + rest_length[i - 1];
	}
}

void CableState::distribute_masses(const CableParams &p_params) {
	for (int i = 0; i < count; ++i) {
		if (is_pinned(i)) {
			inv_mass[i] = 0.0f;
			continue;
		}
		// Масса частицы — половина каждого прилегающего сегмента; у концевых
		// остаётся половина от внутренней.
		float owned = 0.0f;
		if (i > 0) {
			owned += rest_length[i - 1] * 0.5f;
		}
		if (i < count - 1) {
			owned += rest_length[i] * 0.5f;
		}
		const float mass = p_params.mass_per_meter * owned;
		inv_mass[i] = mass > EPS_LENGTH ? 1.0f / mass : 0.0f;
	}
}

} // namespace cable
