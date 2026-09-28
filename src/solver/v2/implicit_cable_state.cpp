#include "solver/v2/implicit_cable_state.h"

namespace cable::v2 {

namespace {

template <typename T>
void resize_tracked(std::vector<T> &p_values, size_t p_size, size_t &r_growths) {
	const size_t old_capacity = p_values.capacity();
	p_values.resize(p_size);
	if (p_values.capacity() != old_capacity) {
		++r_growths;
	}
}

template <typename T>
size_t capacity_bytes(const std::vector<T> &p_values) {
	return p_values.capacity() * sizeof(T);
}

} // namespace

void ImplicitCableState::initialize(const CableState &p_legacy, double p_mass_per_meter) {
	count = p_legacy.count;
	const size_t node_count = static_cast<size_t>(count);
	const size_t edge_count = count > 0 ? static_cast<size_t>(count - 1) : 0;

	resize_tracked(accepted_position, node_count, capacity_growths);
	resize_tracked(trial_position, node_count, capacity_growths);
	resize_tracked(velocity, node_count, capacity_growths);
	resize_tracked(mass, node_count, capacity_growths);
	resize_tracked(rest_length, edge_count, capacity_growths);
	resize_tracked(pinned, node_count, capacity_growths);

	for (int i = 0; i < count; ++i) {
		accepted_position[i] = ImplicitVec3(p_legacy.position[i]);
		trial_position[i] = ImplicitVec3(p_legacy.position[i]);
		velocity[i] = ImplicitVec3(p_legacy.velocity[i]);
		pinned[i] = p_legacy.is_pinned(i) ? 1 : 0;

		double owned_length = 0.0;
		if (i > 0) {
			owned_length += 0.5 * p_legacy.rest_length[i - 1];
		}
		if (i + 1 < count) {
			owned_length += 0.5 * p_legacy.rest_length[i];
		}
		mass[i] = p_mass_per_meter * owned_length;
	}
	for (int i = 0; i + 1 < count; ++i) {
		rest_length[i] = p_legacy.rest_length[i];
	}
	update_peak_capacity();
}

void ImplicitCableState::update_peak_capacity() {
	const size_t bytes =
			capacity_bytes(accepted_position) +
			capacity_bytes(trial_position) +
			capacity_bytes(velocity) +
			capacity_bytes(mass) +
			capacity_bytes(rest_length) +
			capacity_bytes(pinned);
	if (bytes > peak_capacity_bytes) {
		peak_capacity_bytes = bytes;
	}
}

} // namespace cable::v2
