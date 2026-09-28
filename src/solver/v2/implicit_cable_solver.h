#pragma once

#include "solver/v2/implicit_cable_state.h"
#include "solver/v2/sparse_system.h"

#include <cstddef>
#include <vector>

namespace cable::v2 {

struct ImplicitCableParams {
	double axial_stiffness = 100000.0;
	double bending_stiffness = 0.01;
	Vec3 gravity = Vec3(0.0f, -9.8f, 0.0f);
	double gravity_scale = 1.0;
	int max_newton_iterations = 12;
	int max_line_search_iterations = 12;
	double absolute_residual_tolerance = 1.0e-6;
	double relative_residual_tolerance = 1.0e-8;
	double relative_step_tolerance = 1.0e-7;
};

struct ImplicitSolverDiagnostics {
	double assembly_us = 0.0;
	double inertia_assembly_us = 0.0;
	double stretch_assembly_us = 0.0;
	double bending_assembly_us = 0.0;
	double line_search_evaluation_us = 0.0;
	double symbolic_analysis_us = 0.0;
	double numeric_factorization_us = 0.0;
	double linear_solve_us = 0.0;
	double residual = 0.0;
	double max_stretch_error = 0.0;
	double relative_step = 0.0;
	int newton_iterations = 0;
	int line_search_iterations = 0;
	size_t capacity_growths = 0;
	size_t peak_capacity_bytes = 0;
	bool step_accepted = false;
};

class ImplicitCableSolver {
public:
	ImplicitCableParams params;
	ImplicitCableState state;

	void initialize(const CableState &p_legacy, double p_mass_per_meter);
	bool step(double p_dt, const Vec3 &p_anchor_start, const Vec3 *p_anchor_end);
	void copy_to(CableState &r_legacy) const;

	const ImplicitSolverDiagnostics &get_diagnostics() const { return diagnostics; }

private:
	bool build_dof_map();
	bool assemble(const std::vector<ImplicitVec3> &p_position, const std::vector<ImplicitVec3> &p_predictor,
			double p_dt, double &r_energy);
	bool evaluate_potential(const std::vector<ImplicitVec3> &p_position,
			const std::vector<ImplicitVec3> &p_predictor, double p_dt, double &r_energy) const;
	bool line_search(const std::vector<ImplicitVec3> &p_predictor, double p_dt, double p_energy,
			double p_directional_derivative, double &r_accepted_alpha);
	void add_local(const int *p_nodes, int p_node_count, const double *p_gradient,
			const double *p_hessian);
	double residual_norm() const;
	double max_stretch_error(const std::vector<ImplicitVec3> &p_position) const;
	void update_peak_capacity();

	SparseSystem system;
	std::vector<int> dof_map;
	std::vector<ImplicitVec3> predictor;
	std::vector<ImplicitVec3> candidate;
	std::vector<double> residual;
	std::vector<double> right_hand_side;
	std::vector<double> newton_step;
	ImplicitSolverDiagnostics diagnostics;
	size_t growths = 0;
	size_t peak_bytes = 0;
};

} // namespace cable::v2
