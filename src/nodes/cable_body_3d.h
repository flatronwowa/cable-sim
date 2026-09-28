#pragma once

#include "nodes/cable_debug_draw.h"
#include "nodes/cable_mesh_builder.h"
#include "nodes/world_snapshot_builder.h"
#include "solver/cable_solver.h"
#include "solver/v2/implicit_cable_solver.h"

#include <godot_cpp/classes/array_mesh.hpp>
#include <godot_cpp/classes/immediate_mesh.hpp>
#include <godot_cpp/classes/material.hpp>
#include <godot_cpp/classes/mesh_instance3d.hpp>
#include <godot_cpp/classes/node3d.hpp>

#include <vector>

namespace godot {

class CableBody3D : public Node3D {
	GDCLASS(CableBody3D, Node3D)

protected:
	static void _bind_methods();
	void _notification(int p_what);

public:
	enum SolverBackend {
		LEGACY_XPBD = 0,
		IMPLICIT_V2 = 1,
	};

	CableBody3D();

	void _ready() override;
	void _physics_process(double p_delta) override;
	void _process(double p_delta) override;
	PackedStringArray _get_configuration_warnings() const override;

	// Геометрия.
	void set_cable_length(double p_length);
	double get_cable_length() const { return solver.params.cable_length; }
	void set_cable_radius(double p_radius);
	double get_cable_radius() const { return solver.params.cable_radius; }
	void set_target_segment_length(double p_length);
	double get_target_segment_length() const { return solver.params.target_segment_length; }

	// Материал и физика.
	void set_mass_per_meter(double p_value);
	double get_mass_per_meter() const { return solver.params.mass_per_meter; }
	void set_stretch_compliance(double p_value);
	double get_stretch_compliance() const { return solver.params.stretch_compliance; }
	void set_bending_compliance(double p_value);
	double get_bending_compliance() const { return solver.params.bending_compliance; }
	void set_minimum_bend_radius(double p_value);
	double get_minimum_bend_radius() const { return solver.params.minimum_bend_radius; }
	void set_damping(double p_value);
	double get_damping() const { return solver.params.damping; }
	void set_static_friction(double p_value);
	double get_static_friction() const { return solver.params.static_friction; }
	void set_dynamic_friction(double p_value);
	double get_dynamic_friction() const { return solver.params.dynamic_friction; }
	void set_gravity_scale(double p_value);
	double get_gravity_scale() const { return solver.params.gravity_scale; }
	void set_anchor_stiffness_compliance(double p_value);
	double get_anchor_stiffness_compliance() const { return solver.params.anchor_stiffness_compliance; }

	// Солвер.
	void set_substeps_min(int p_value);
	int get_substeps_min() const { return solver.params.substeps_min; }
	void set_substeps_max(int p_value);
	int get_substeps_max() const { return solver.params.substeps_max; }
	void set_solver_iterations(int p_value);
	int get_solver_iterations() const { return solver.params.solver_iterations; }
	void set_collision_margin(double p_value);
	double get_collision_margin() const { return solver.params.collision_margin; }
	void set_self_collision_enabled(bool p_enabled);
	bool is_self_collision_enabled() const { return solver.params.self_collision_enabled; }
	void set_sleep_enabled(bool p_enabled);
	bool is_sleep_enabled() const { return solver.params.sleep_enabled; }
	void set_solver_backend(SolverBackend p_backend);
	SolverBackend get_solver_backend() const { return solver_backend; }
	void set_axial_stiffness(double p_value);
	double get_axial_stiffness() const { return implicit_solver.params.axial_stiffness; }
	void set_v2_bending_stiffness(double p_value);
	double get_v2_bending_stiffness() const { return implicit_solver.params.bending_stiffness; }

	// Коллизии.
	void set_collision_mask(int p_mask);
	int get_collision_mask() const { return static_cast<int>(collision_mask); }
	void set_collide_with_bodies(bool p_enabled);
	bool is_collide_with_bodies() const { return collide_with_bodies; }
	void set_collide_with_areas(bool p_enabled);
	bool is_collide_with_areas() const { return collide_with_areas; }
	void add_collision_exception(Node *p_node);
	void remove_collision_exception(Node *p_node);
	void clear_collision_exceptions();

	// Якоря и рендеринг.
	void set_anchor_a(const NodePath &p_path);
	NodePath get_anchor_a() const { return anchor_a_path; }
	void set_anchor_b(const NodePath &p_path);
	NodePath get_anchor_b() const { return anchor_b_path; }
	void set_radial_segments(int p_value);
	int get_radial_segments() const { return radial_segments; }
	void set_cable_material(const Ref<Material> &p_material);
	Ref<Material> get_cable_material() const { return cable_material; }
	void set_debug_draw_enabled(bool p_enabled);
	bool is_debug_draw_enabled() const { return debug_draw_enabled; }
	void set_simulation_enabled(bool p_enabled);
	bool is_simulation_enabled() const { return simulation_enabled; }

	// Публичный API.
	PackedVector3Array get_centerline() const;
	Vector3 constrain_motion(const Vector3 &p_from, const Vector3 &p_to);
	double get_remaining_length() const;
	double get_deployed_length() const;
	double get_tension() const;
	Vector3 get_tension_direction() const;
	int get_particle_count() const { return solver.state.count; }
	int get_awake_particle_count() const { return solver.get_diagnostics().awake_particles; }
	int get_contact_count() const { return solver.get_diagnostics().contact_count; }
	int get_substep_count() const { return solver.get_diagnostics().substeps; }
	int get_solver_iteration_count() const;
	int get_world_segment_contact_count() const { return solver.get_diagnostics().world_segment_contact_count; }
	int get_self_contact_count() const { return solver.get_diagnostics().self_contact_count; }
	double get_kinetic_energy() const { return solver.get_diagnostics().kinetic_energy; }
	double get_bending_energy() const { return solver.get_diagnostics().normalized_bending_energy; }
	double get_max_stretch_error() const;
	double get_max_projection() const { return solver.get_diagnostics().max_projection; }
	double get_v2_assembly_us() const { return implicit_solver.get_diagnostics().assembly_us; }
	double get_v2_inertia_assembly_us() const { return implicit_solver.get_diagnostics().inertia_assembly_us; }
	double get_v2_stretch_assembly_us() const { return implicit_solver.get_diagnostics().stretch_assembly_us; }
	double get_v2_bending_assembly_us() const { return implicit_solver.get_diagnostics().bending_assembly_us; }
	double get_v2_line_search_evaluation_us() const { return implicit_solver.get_diagnostics().line_search_evaluation_us; }
	double get_v2_symbolic_analysis_us() const { return implicit_solver.get_diagnostics().symbolic_analysis_us; }
	double get_v2_numeric_factorization_us() const { return implicit_solver.get_diagnostics().numeric_factorization_us; }
	double get_v2_linear_solve_us() const { return implicit_solver.get_diagnostics().linear_solve_us; }
	double get_v2_residual() const { return implicit_solver.get_diagnostics().residual; }
	double get_v2_relative_step() const { return implicit_solver.get_diagnostics().relative_step; }
	int get_v2_newton_iterations() const { return implicit_solver.get_diagnostics().newton_iterations; }
	int get_v2_line_search_iterations() const { return implicit_solver.get_diagnostics().line_search_iterations; }
	int64_t get_v2_capacity_growths() const { return implicit_solver.get_diagnostics().capacity_growths; }
	int64_t get_v2_peak_capacity_bytes() const { return implicit_solver.get_diagnostics().peak_capacity_bytes; }
	bool was_v2_step_accepted() const { return implicit_solver.get_diagnostics().step_accepted; }
	void rebuild_cable();

private:
	void ensure_children();
	void resolve_anchors();
	void apply_parameter_change();
	void update_anchor_inputs(double p_delta);
	void build_snapshot();
	void update_mesh(double p_interpolation);
	void store_render_snapshot();
	Transform3D world_to_local() const;
	Node3D *anchor_node(const NodePath &p_path) const;

	cable::CableSolver solver;
	cable::v2::ImplicitCableSolver implicit_solver;
	cable::WorldSnapshot snapshot;
	WorldSnapshotBuilder snapshot_builder;
	CableMeshBuilder mesh_builder;
	CableDebugDraw debug_draw;

	MeshInstance3D *mesh_instance = nullptr;
	MeshInstance3D *debug_instance = nullptr;
	Ref<ArrayMesh> cable_mesh;
	Ref<ImmediateMesh> debug_mesh;
	Ref<Material> cable_material;

	NodePath anchor_a_path;
	NodePath anchor_b_path;

	// Две последние физические позиции центральной линии: меш обновляется в _process
	// по интерполированным значениям, иначе кабель дёргается при рендере выше 60 Гц.
	std::vector<cable::Vec3> render_previous;
	std::vector<cable::Vec3> render_current;
	std::vector<cable::Vec3> render_interpolated;

	HashSet<uint64_t> exceptions;

	uint32_t collision_mask = 0xFFFFFFFF;
	bool collide_with_bodies = true;
	bool collide_with_areas = false;
	int radial_segments = 8;
	bool debug_draw_enabled = false;
	bool simulation_enabled = true;
	bool needs_rebuild = true;
	bool prerelaxed = false;
	bool taut_limit_active = false;
	SolverBackend solver_backend = LEGACY_XPBD;
	float last_uv_length = -1.0f;
};

} // namespace godot

VARIANT_ENUM_CAST(godot::CableBody3D::SolverBackend)
