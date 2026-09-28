#include "solver/v2/implicit_cable_solver.h"

#include "solver/v2/elastic_energy.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

namespace cable::v2 {

namespace {

constexpr int SCALAR_HALF_BANDWIDTH = 8;

using Clock = std::chrono::steady_clock;

double elapsed_us(Clock::time_point p_start) {
	return std::chrono::duration<double, std::micro>(Clock::now() - p_start).count();
}

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

double squared_length(const ImplicitVec3 &p_value) {
	return p_value.x * p_value.x + p_value.y * p_value.y + p_value.z * p_value.z;
}

double implicit_dot(const ImplicitVec3 &p_a, const ImplicitVec3 &p_b) {
	return p_a.x * p_b.x + p_a.y * p_b.y + p_a.z * p_b.z;
}

ImplicitVec3 implicit_cross(const ImplicitVec3 &p_a, const ImplicitVec3 &p_b) {
	return ImplicitVec3(
			p_a.y * p_b.z - p_a.z * p_b.y,
			p_a.z * p_b.x - p_a.x * p_b.z,
			p_a.x * p_b.y - p_a.y * p_b.x);
}

double implicit_distance(const ImplicitVec3 &p_a, const ImplicitVec3 &p_b) {
	return std::sqrt(squared_length(p_b - p_a));
}

} // namespace

void ImplicitCableSolver::initialize(const CableState &p_legacy, double p_mass_per_meter) {
	state.initialize(p_legacy, p_mass_per_meter);
	resize_tracked(dof_map, static_cast<size_t>(state.count * 3), growths);
	resize_tracked(predictor, static_cast<size_t>(state.count), growths);
	resize_tracked(candidate, static_cast<size_t>(state.count), growths);
	build_dof_map();
	update_peak_capacity();
}

bool ImplicitCableSolver::build_dof_map() {
	if (static_cast<int>(dof_map.size()) != state.count * 3) {
		return false;
	}
	int next_dof = 0;
	for (int node = 0; node < state.count; ++node) {
		for (int axis = 0; axis < 3; ++axis) {
			dof_map[node * 3 + axis] = state.pinned[node] ? -1 : next_dof++;
		}
	}
	resize_tracked(residual, static_cast<size_t>(next_dof), growths);
	resize_tracked(right_hand_side, static_cast<size_t>(next_dof), growths);
	resize_tracked(newton_step, static_cast<size_t>(next_dof), growths);
	return true;
}

void ImplicitCableSolver::add_local(const int *p_nodes, int p_node_count,
		const double *p_gradient, const double *p_hessian) {
	const int local_dof_count = p_node_count * 3;
	for (int local_row = 0; local_row < local_dof_count; ++local_row) {
		const int node_row = p_nodes[local_row / 3];
		const int global_row = dof_map[node_row * 3 + local_row % 3];
		if (global_row < 0) {
			continue;
		}
		residual[global_row] += p_gradient[local_row];
		for (int local_column = 0; local_column < local_dof_count; ++local_column) {
			const int node_column = p_nodes[local_column / 3];
			const int global_column = dof_map[node_column * 3 + local_column % 3];
			if (global_column < 0 || global_column > global_row) {
				continue;
			}
			system.add(global_row, global_column,
					p_hessian[local_row * 9 + local_column]);
		}
	}
}

bool ImplicitCableSolver::assemble(const std::vector<ImplicitVec3> &p_position,
		const std::vector<ImplicitVec3> &p_predictor, double p_dt, double &r_energy) {
	std::fill(residual.begin(), residual.end(), 0.0);
	system.clear_numeric();
	r_energy = 0.0;
	const double inverse_dt_squared = 1.0 / (p_dt * p_dt);

	const Clock::time_point inertia_start = Clock::now();
	for (int node = 0; node < state.count; ++node) {
		if (state.pinned[node]) {
			continue;
		}
		const ImplicitVec3 difference = p_position[node] - p_predictor[node];
		const double coefficient = state.mass[node] * inverse_dt_squared;
		r_energy += 0.5 * coefficient * squared_length(difference);
		for (int axis = 0; axis < 3; ++axis) {
			const int dof = dof_map[node * 3 + axis];
			const double value = axis == 0 ? difference.x : (axis == 1 ? difference.y : difference.z);
			residual[dof] += coefficient * value;
			system.add(dof, dof, coefficient);
		}
	}
	diagnostics.inertia_assembly_us += elapsed_us(inertia_start);

	const Clock::time_point stretch_start = Clock::now();
	for (int edge = 0; edge + 1 < state.count; ++edge) {
		const LocalEnergy local = evaluate_stretch(
				p_position[edge], p_position[edge + 1], state.rest_length[edge],
				params.axial_stiffness);
		if (!local.valid || !std::isfinite(local.energy)) {
			return false;
		}
		const int nodes[2] = { edge, edge + 1 };
		add_local(nodes, 2, local.gradient, local.hessian);
		r_energy += local.energy;
	}
	diagnostics.stretch_assembly_us += elapsed_us(stretch_start);

	const Clock::time_point bending_start = Clock::now();
	for (int center = 1; center + 1 < state.count; ++center) {
		const double rest_voronoi =
				0.5 * (state.rest_length[center - 1] + state.rest_length[center]);
		const LocalEnergy local = evaluate_bending(
				p_position[center - 1], p_position[center], p_position[center + 1],
				rest_voronoi, params.bending_stiffness);
		if (!local.valid || !std::isfinite(local.energy)) {
			return false;
		}
		const int nodes[3] = { center - 1, center, center + 1 };
		add_local(nodes, 3, local.gradient, local.hessian);
		r_energy += local.energy;
	}
	diagnostics.bending_assembly_us += elapsed_us(bending_start);
	return std::isfinite(r_energy);
}

bool ImplicitCableSolver::evaluate_potential(const std::vector<ImplicitVec3> &p_position,
		const std::vector<ImplicitVec3> &p_predictor, double p_dt, double &r_energy) const {
	r_energy = 0.0;
	const double inverse_dt_squared = 1.0 / (p_dt * p_dt);
	for (int node = 0; node < state.count; ++node) {
		if (!state.pinned[node]) {
			const ImplicitVec3 difference = p_position[node] - p_predictor[node];
			r_energy += 0.5 * state.mass[node] * inverse_dt_squared * squared_length(difference);
		}
	}
	for (int edge = 0; edge + 1 < state.count; ++edge) {
		const double rest = state.rest_length[edge];
		const double edge_length = implicit_distance(p_position[edge], p_position[edge + 1]);
		if (rest <= 0.0 || edge_length <= 0.0) {
			return false;
		}
		const double extension = edge_length - rest;
		r_energy += 0.5 * params.axial_stiffness / rest * extension * extension;
	}
	for (int center = 1; center + 1 < state.count; ++center) {
		const ImplicitVec3 before = p_position[center] - p_position[center - 1];
		const ImplicitVec3 after = p_position[center + 1] - p_position[center];
		const double before_length = std::sqrt(squared_length(before));
		const double after_length = std::sqrt(squared_length(after));
		const double denominator =
				before_length * after_length + implicit_dot(before, after);
		if (before_length <= 0.0 || after_length <= 0.0 ||
				denominator <= 1.0e-12 * before_length * after_length) {
			return false;
		}
		const ImplicitVec3 curvature = implicit_cross(before, after) * (2.0 / denominator);
		const double rest_voronoi =
				0.5 * (state.rest_length[center - 1] + state.rest_length[center]);
		r_energy +=
				0.5 * params.bending_stiffness / rest_voronoi * squared_length(curvature);
	}
	return std::isfinite(r_energy);
}

bool ImplicitCableSolver::line_search(const std::vector<ImplicitVec3> &p_predictor, double p_dt,
		double p_energy, double p_directional_derivative, double &r_accepted_alpha) {
	double alpha = 1.0;
	for (int iteration = 0; iteration < params.max_line_search_iterations; ++iteration) {
		candidate = state.trial_position;
		for (int node = 0; node < state.count; ++node) {
			if (state.pinned[node]) {
				continue;
			}
			const int base = node * 3;
			candidate[node].x += alpha * newton_step[dof_map[base]];
			candidate[node].y += alpha * newton_step[dof_map[base + 1]];
			candidate[node].z += alpha * newton_step[dof_map[base + 2]];
		}
		double candidate_energy = 0.0;
		const Clock::time_point assembly_start = Clock::now();
		const bool assembled = evaluate_potential(candidate, p_predictor, p_dt, candidate_energy);
		const double evaluation_us = elapsed_us(assembly_start);
		diagnostics.assembly_us += evaluation_us;
		diagnostics.line_search_evaluation_us += evaluation_us;
		++diagnostics.line_search_iterations;
		if (assembled && std::isfinite(candidate_energy) &&
				candidate_energy <= p_energy + 1.0e-4 * alpha * p_directional_derivative) {
			state.trial_position.swap(candidate);
			r_accepted_alpha = alpha;
			return true;
		}
		alpha *= 0.5;
	}
	return false;
}

double ImplicitCableSolver::residual_norm() const {
	double squared_norm = 0.0;
	for (double value : residual) {
		squared_norm += value * value;
	}
	return residual.empty() ? 0.0 : std::sqrt(squared_norm / static_cast<double>(residual.size()));
}

double ImplicitCableSolver::max_stretch_error(const std::vector<ImplicitVec3> &p_position) const {
	double maximum = 0.0;
	for (int edge = 0; edge + 1 < state.count; ++edge) {
		const double rest = state.rest_length[edge];
		if (rest > 0.0) {
			maximum = std::max(maximum,
					std::fabs(implicit_distance(p_position[edge], p_position[edge + 1]) / rest - 1.0));
		}
	}
	return maximum;
}

bool ImplicitCableSolver::step(double p_dt, const Vec3 &p_anchor_start,
		const Vec3 *p_anchor_end) {
	diagnostics = ImplicitSolverDiagnostics();
	if (!state.is_initialized() || p_dt <= 0.0) {
		return false;
	}

	state.pinned[0] = 1;
	state.pinned[state.count - 1] = p_anchor_end != nullptr ? 1 : 0;
	if (!build_dof_map()) {
		return false;
	}

	state.trial_position = state.accepted_position;
	state.trial_position[0] = ImplicitVec3(p_anchor_start);
	if (p_anchor_end != nullptr) {
		state.trial_position[state.count - 1] = ImplicitVec3(*p_anchor_end);
	}

	const double dt_squared = p_dt * p_dt;
	const ImplicitVec3 acceleration(
			params.gravity.x * params.gravity_scale,
			params.gravity.y * params.gravity_scale,
			params.gravity.z * params.gravity_scale);
	for (int node = 0; node < state.count; ++node) {
		if (state.pinned[node]) {
			predictor[node] = state.trial_position[node];
		} else {
			predictor[node] = state.accepted_position[node] +
					state.velocity[node] * p_dt +
					acceleration * dt_squared;
		}
	}

	const Clock::time_point symbolic_start = Clock::now();
	if (!system.analyze_pattern(static_cast<int>(residual.size()), SCALAR_HALF_BANDWIDTH)) {
		return false;
	}
	diagnostics.symbolic_analysis_us = elapsed_us(symbolic_start);
	update_peak_capacity();
	diagnostics.capacity_growths =
			growths + state.capacity_growths + system.capacity_growths();
	diagnostics.peak_capacity_bytes =
			peak_bytes + state.peak_capacity_bytes + system.peak_capacity_bytes();
	diagnostics.max_stretch_error = max_stretch_error(state.accepted_position);

	double initial_residual = 0.0;
	double last_relative_step = std::numeric_limits<double>::infinity();
	double mean_rest_length = 0.0;
	for (double rest : state.rest_length) {
		mean_rest_length += rest;
	}
	mean_rest_length /= static_cast<double>(state.rest_length.size());
	bool converged = false;
	for (int iteration = 0; iteration < params.max_newton_iterations; ++iteration) {
		double energy = 0.0;
		const Clock::time_point assembly_start = Clock::now();
		const bool assembled = assemble(state.trial_position, predictor, p_dt, energy);
		diagnostics.assembly_us += elapsed_us(assembly_start);
		if (!assembled) {
			return false;
		}

		const double current_residual = residual_norm();
		diagnostics.residual = current_residual;
		diagnostics.newton_iterations = iteration;
		if (iteration == 0) {
			initial_residual = current_residual;
		}
		const double tolerance = std::max(
				params.absolute_residual_tolerance,
				params.relative_residual_tolerance * initial_residual);
		const double current_stretch = max_stretch_error(state.trial_position);
		diagnostics.max_stretch_error = current_stretch;
		if (current_residual <= tolerance &&
				iteration == 0) {
			converged = true;
			break;
		}

		const Clock::time_point factor_start = Clock::now();
		bool factored = false;
		double diagonal_shift = 0.0;
		const double diagonal_scale = std::max(1.0, system.max_abs_diagonal());
		for (int attempt = 0; attempt < 12; ++attempt) {
			if (system.factorize(diagonal_shift)) {
				factored = true;
				break;
			}
			diagonal_shift = diagonal_shift == 0.0
					? diagonal_scale * 1.0e-10
					: diagonal_shift * 10.0;
		}
		diagnostics.numeric_factorization_us += elapsed_us(factor_start);
		if (!factored) {
			return false;
		}
		for (size_t i = 0; i < residual.size(); ++i) {
			right_hand_side[i] = -residual[i];
		}
		const Clock::time_point solve_start = Clock::now();
		const bool solved = system.solve(right_hand_side, newton_step);
		diagnostics.linear_solve_us += elapsed_us(solve_start);
		if (!solved) {
			return false;
		}

		double directional_derivative = 0.0;
		double squared_step = 0.0;
		for (size_t i = 0; i < residual.size(); ++i) {
			directional_derivative += residual[i] * newton_step[i];
			squared_step += newton_step[i] * newton_step[i];
		}
		last_relative_step = residual.empty() ? 0.0
											 : std::sqrt(squared_step / static_cast<double>(residual.size())) /
						mean_rest_length;
		diagnostics.relative_step = last_relative_step;
		if (current_residual <= tolerance &&
				last_relative_step <= params.relative_step_tolerance) {
			converged = true;
			break;
		}
		double accepted_alpha = 0.0;
		if (!std::isfinite(directional_derivative) || directional_derivative >= 0.0 ||
				!line_search(predictor, p_dt, energy, directional_derivative, accepted_alpha)) {
			return false;
		}
		last_relative_step *= accepted_alpha;
		diagnostics.relative_step = last_relative_step;
		diagnostics.newton_iterations = iteration + 1;
	}

	if (!converged) {
		double final_energy = 0.0;
		const Clock::time_point assembly_start = Clock::now();
		const bool assembled = assemble(state.trial_position, predictor, p_dt, final_energy);
		diagnostics.assembly_us += elapsed_us(assembly_start);
		diagnostics.residual = assembled ? residual_norm() : std::numeric_limits<double>::infinity();
		diagnostics.max_stretch_error = max_stretch_error(state.trial_position);
		const double tolerance = std::max(
				params.absolute_residual_tolerance,
				params.relative_residual_tolerance * initial_residual);
		converged = assembled &&
				diagnostics.residual <= tolerance &&
				last_relative_step <= params.relative_step_tolerance;
	}
	if (!converged) {
		return false;
	}

	for (int node = 0; node < state.count; ++node) {
		state.velocity[node] =
				(state.trial_position[node] - state.accepted_position[node]) *
				(1.0 / p_dt);
	}
	state.accepted_position = state.trial_position;
	diagnostics.max_stretch_error = max_stretch_error(state.accepted_position);
	diagnostics.step_accepted = true;
	update_peak_capacity();
	diagnostics.capacity_growths =
			growths + state.capacity_growths + system.capacity_growths();
	diagnostics.peak_capacity_bytes =
			peak_bytes + state.peak_capacity_bytes + system.peak_capacity_bytes();
	return true;
}

void ImplicitCableSolver::copy_to(CableState &r_legacy) const {
	if (r_legacy.count != state.count) {
		return;
	}
	for (int node = 0; node < state.count; ++node) {
		r_legacy.prev_position[node] = r_legacy.position[node];
		r_legacy.position[node] = Vec3(
				static_cast<float>(state.accepted_position[node].x),
				static_cast<float>(state.accepted_position[node].y),
				static_cast<float>(state.accepted_position[node].z));
		r_legacy.velocity[node] = Vec3(
				static_cast<float>(state.velocity[node].x),
				static_cast<float>(state.velocity[node].y),
				static_cast<float>(state.velocity[node].z));
	}
}

void ImplicitCableSolver::update_peak_capacity() {
	const size_t bytes =
			capacity_bytes(dof_map) +
			capacity_bytes(predictor) +
			capacity_bytes(candidate) +
			capacity_bytes(residual) +
			capacity_bytes(right_hand_side) +
			capacity_bytes(newton_step);
	if (bytes > peak_bytes) {
		peak_bytes = bytes;
	}
	state.update_peak_capacity();
}

} // namespace cable::v2
