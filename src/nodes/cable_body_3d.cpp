#include "nodes/cable_body_3d.h"

#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/classes/project_settings.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

using namespace godot;
using cable::Vec3;

namespace {

Vec3 to_solver(const Vector3 &v) {
	return Vec3(static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z));
}

Vector3 to_godot(const Vec3 &v) {
	return Vector3(v.x, v.y, v.z);
}

} // namespace

CableBody3D::CableBody3D() {
	set_physics_process(true);
	set_process(true);
}

void CableBody3D::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_cable_length", "length"), &CableBody3D::set_cable_length);
	ClassDB::bind_method(D_METHOD("get_cable_length"), &CableBody3D::get_cable_length);
	ClassDB::bind_method(D_METHOD("set_cable_radius", "radius"), &CableBody3D::set_cable_radius);
	ClassDB::bind_method(D_METHOD("get_cable_radius"), &CableBody3D::get_cable_radius);
	ClassDB::bind_method(D_METHOD("set_target_segment_length", "length"), &CableBody3D::set_target_segment_length);
	ClassDB::bind_method(D_METHOD("get_target_segment_length"), &CableBody3D::get_target_segment_length);

	ClassDB::bind_method(D_METHOD("set_mass_per_meter", "mass"), &CableBody3D::set_mass_per_meter);
	ClassDB::bind_method(D_METHOD("get_mass_per_meter"), &CableBody3D::get_mass_per_meter);
	ClassDB::bind_method(D_METHOD("set_stretch_compliance", "compliance"), &CableBody3D::set_stretch_compliance);
	ClassDB::bind_method(D_METHOD("get_stretch_compliance"), &CableBody3D::get_stretch_compliance);
	ClassDB::bind_method(D_METHOD("set_bending_compliance", "compliance"), &CableBody3D::set_bending_compliance);
	ClassDB::bind_method(D_METHOD("get_bending_compliance"), &CableBody3D::get_bending_compliance);
	ClassDB::bind_method(D_METHOD("set_minimum_bend_radius", "radius"), &CableBody3D::set_minimum_bend_radius);
	ClassDB::bind_method(D_METHOD("get_minimum_bend_radius"), &CableBody3D::get_minimum_bend_radius);
	ClassDB::bind_method(D_METHOD("set_damping", "damping"), &CableBody3D::set_damping);
	ClassDB::bind_method(D_METHOD("get_damping"), &CableBody3D::get_damping);
	ClassDB::bind_method(D_METHOD("set_static_friction", "friction"), &CableBody3D::set_static_friction);
	ClassDB::bind_method(D_METHOD("get_static_friction"), &CableBody3D::get_static_friction);
	ClassDB::bind_method(D_METHOD("set_dynamic_friction", "friction"), &CableBody3D::set_dynamic_friction);
	ClassDB::bind_method(D_METHOD("get_dynamic_friction"), &CableBody3D::get_dynamic_friction);
	ClassDB::bind_method(D_METHOD("set_gravity_scale", "scale"), &CableBody3D::set_gravity_scale);
	ClassDB::bind_method(D_METHOD("get_gravity_scale"), &CableBody3D::get_gravity_scale);
	ClassDB::bind_method(D_METHOD("set_anchor_stiffness_compliance", "compliance"), &CableBody3D::set_anchor_stiffness_compliance);
	ClassDB::bind_method(D_METHOD("get_anchor_stiffness_compliance"), &CableBody3D::get_anchor_stiffness_compliance);

	ClassDB::bind_method(D_METHOD("set_substeps_min", "substeps"), &CableBody3D::set_substeps_min);
	ClassDB::bind_method(D_METHOD("get_substeps_min"), &CableBody3D::get_substeps_min);
	ClassDB::bind_method(D_METHOD("set_substeps_max", "substeps"), &CableBody3D::set_substeps_max);
	ClassDB::bind_method(D_METHOD("get_substeps_max"), &CableBody3D::get_substeps_max);
	ClassDB::bind_method(D_METHOD("set_solver_iterations", "iterations"), &CableBody3D::set_solver_iterations);
	ClassDB::bind_method(D_METHOD("get_solver_iterations"), &CableBody3D::get_solver_iterations);
	ClassDB::bind_method(D_METHOD("set_collision_margin", "margin"), &CableBody3D::set_collision_margin);
	ClassDB::bind_method(D_METHOD("get_collision_margin"), &CableBody3D::get_collision_margin);
	ClassDB::bind_method(D_METHOD("set_self_collision_enabled", "enabled"), &CableBody3D::set_self_collision_enabled);
	ClassDB::bind_method(D_METHOD("is_self_collision_enabled"), &CableBody3D::is_self_collision_enabled);
	ClassDB::bind_method(D_METHOD("set_sleep_enabled", "enabled"), &CableBody3D::set_sleep_enabled);
	ClassDB::bind_method(D_METHOD("is_sleep_enabled"), &CableBody3D::is_sleep_enabled);
	ClassDB::bind_method(D_METHOD("set_solver_backend", "backend"), &CableBody3D::set_solver_backend);
	ClassDB::bind_method(D_METHOD("get_solver_backend"), &CableBody3D::get_solver_backend);
	ClassDB::bind_method(D_METHOD("set_axial_stiffness", "stiffness"), &CableBody3D::set_axial_stiffness);
	ClassDB::bind_method(D_METHOD("get_axial_stiffness"), &CableBody3D::get_axial_stiffness);
	ClassDB::bind_method(D_METHOD("set_v2_bending_stiffness", "stiffness"), &CableBody3D::set_v2_bending_stiffness);
	ClassDB::bind_method(D_METHOD("get_v2_bending_stiffness"), &CableBody3D::get_v2_bending_stiffness);

	ClassDB::bind_method(D_METHOD("set_collision_mask", "mask"), &CableBody3D::set_collision_mask);
	ClassDB::bind_method(D_METHOD("get_collision_mask"), &CableBody3D::get_collision_mask);
	ClassDB::bind_method(D_METHOD("set_collide_with_bodies", "enabled"), &CableBody3D::set_collide_with_bodies);
	ClassDB::bind_method(D_METHOD("is_collide_with_bodies"), &CableBody3D::is_collide_with_bodies);
	ClassDB::bind_method(D_METHOD("set_collide_with_areas", "enabled"), &CableBody3D::set_collide_with_areas);
	ClassDB::bind_method(D_METHOD("is_collide_with_areas"), &CableBody3D::is_collide_with_areas);
	ClassDB::bind_method(D_METHOD("add_collision_exception", "node"), &CableBody3D::add_collision_exception);
	ClassDB::bind_method(D_METHOD("remove_collision_exception", "node"), &CableBody3D::remove_collision_exception);
	ClassDB::bind_method(D_METHOD("clear_collision_exceptions"), &CableBody3D::clear_collision_exceptions);

	ClassDB::bind_method(D_METHOD("set_anchor_a", "path"), &CableBody3D::set_anchor_a);
	ClassDB::bind_method(D_METHOD("get_anchor_a"), &CableBody3D::get_anchor_a);
	ClassDB::bind_method(D_METHOD("set_anchor_b", "path"), &CableBody3D::set_anchor_b);
	ClassDB::bind_method(D_METHOD("get_anchor_b"), &CableBody3D::get_anchor_b);
	ClassDB::bind_method(D_METHOD("set_radial_segments", "segments"), &CableBody3D::set_radial_segments);
	ClassDB::bind_method(D_METHOD("get_radial_segments"), &CableBody3D::get_radial_segments);
	ClassDB::bind_method(D_METHOD("set_cable_material", "material"), &CableBody3D::set_cable_material);
	ClassDB::bind_method(D_METHOD("get_cable_material"), &CableBody3D::get_cable_material);
	ClassDB::bind_method(D_METHOD("set_debug_draw_enabled", "enabled"), &CableBody3D::set_debug_draw_enabled);
	ClassDB::bind_method(D_METHOD("is_debug_draw_enabled"), &CableBody3D::is_debug_draw_enabled);
	ClassDB::bind_method(D_METHOD("set_simulation_enabled", "enabled"), &CableBody3D::set_simulation_enabled);
	ClassDB::bind_method(D_METHOD("is_simulation_enabled"), &CableBody3D::is_simulation_enabled);

	ClassDB::bind_method(D_METHOD("get_centerline"), &CableBody3D::get_centerline);
	ClassDB::bind_method(D_METHOD("constrain_motion", "from", "to"), &CableBody3D::constrain_motion);
	ClassDB::bind_method(D_METHOD("get_remaining_length"), &CableBody3D::get_remaining_length);
	ClassDB::bind_method(D_METHOD("get_deployed_length"), &CableBody3D::get_deployed_length);
	ClassDB::bind_method(D_METHOD("get_tension"), &CableBody3D::get_tension);
	ClassDB::bind_method(D_METHOD("get_tension_direction"), &CableBody3D::get_tension_direction);
	ClassDB::bind_method(D_METHOD("get_particle_count"), &CableBody3D::get_particle_count);
	ClassDB::bind_method(D_METHOD("get_awake_particle_count"), &CableBody3D::get_awake_particle_count);
	ClassDB::bind_method(D_METHOD("get_contact_count"), &CableBody3D::get_contact_count);
	ClassDB::bind_method(D_METHOD("get_substep_count"), &CableBody3D::get_substep_count);
	ClassDB::bind_method(D_METHOD("get_solver_iteration_count"), &CableBody3D::get_solver_iteration_count);
	ClassDB::bind_method(D_METHOD("get_world_segment_contact_count"), &CableBody3D::get_world_segment_contact_count);
	ClassDB::bind_method(D_METHOD("get_self_contact_count"), &CableBody3D::get_self_contact_count);
	ClassDB::bind_method(D_METHOD("get_kinetic_energy"), &CableBody3D::get_kinetic_energy);
	ClassDB::bind_method(D_METHOD("get_bending_energy"), &CableBody3D::get_bending_energy);
	ClassDB::bind_method(D_METHOD("get_max_stretch_error"), &CableBody3D::get_max_stretch_error);
	ClassDB::bind_method(D_METHOD("get_max_projection"), &CableBody3D::get_max_projection);
	ClassDB::bind_method(D_METHOD("get_v2_assembly_us"), &CableBody3D::get_v2_assembly_us);
	ClassDB::bind_method(D_METHOD("get_v2_inertia_assembly_us"), &CableBody3D::get_v2_inertia_assembly_us);
	ClassDB::bind_method(D_METHOD("get_v2_stretch_assembly_us"), &CableBody3D::get_v2_stretch_assembly_us);
	ClassDB::bind_method(D_METHOD("get_v2_bending_assembly_us"), &CableBody3D::get_v2_bending_assembly_us);
	ClassDB::bind_method(D_METHOD("get_v2_line_search_evaluation_us"), &CableBody3D::get_v2_line_search_evaluation_us);
	ClassDB::bind_method(D_METHOD("get_v2_symbolic_analysis_us"), &CableBody3D::get_v2_symbolic_analysis_us);
	ClassDB::bind_method(D_METHOD("get_v2_numeric_factorization_us"), &CableBody3D::get_v2_numeric_factorization_us);
	ClassDB::bind_method(D_METHOD("get_v2_linear_solve_us"), &CableBody3D::get_v2_linear_solve_us);
	ClassDB::bind_method(D_METHOD("get_v2_residual"), &CableBody3D::get_v2_residual);
	ClassDB::bind_method(D_METHOD("get_v2_relative_step"), &CableBody3D::get_v2_relative_step);
	ClassDB::bind_method(D_METHOD("get_v2_newton_iterations"), &CableBody3D::get_v2_newton_iterations);
	ClassDB::bind_method(D_METHOD("get_v2_line_search_iterations"), &CableBody3D::get_v2_line_search_iterations);
	ClassDB::bind_method(D_METHOD("get_v2_capacity_growths"), &CableBody3D::get_v2_capacity_growths);
	ClassDB::bind_method(D_METHOD("get_v2_peak_capacity_bytes"), &CableBody3D::get_v2_peak_capacity_bytes);
	ClassDB::bind_method(D_METHOD("was_v2_step_accepted"), &CableBody3D::was_v2_step_accepted);
	ClassDB::bind_method(D_METHOD("rebuild_cable"), &CableBody3D::rebuild_cable);

	BIND_ENUM_CONSTANT(LEGACY_XPBD);
	BIND_ENUM_CONSTANT(IMPLICIT_V2);

	ADD_GROUP("Geometry", "");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "cable_length", PROPERTY_HINT_RANGE, "0.1,200.0,0.01,or_greater,suffix:m"), "set_cable_length", "get_cable_length");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "cable_radius", PROPERTY_HINT_RANGE, "0.001,0.5,0.001,or_greater,suffix:m"), "set_cable_radius", "get_cable_radius");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "target_segment_length", PROPERTY_HINT_RANGE, "0.005,1.0,0.001,or_greater,suffix:m"), "set_target_segment_length", "get_target_segment_length");

	ADD_GROUP("Anchors", "");
	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "anchor_a", PROPERTY_HINT_NODE_PATH_VALID_TYPES, "Node3D"), "set_anchor_a", "get_anchor_a");
	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "anchor_b", PROPERTY_HINT_NODE_PATH_VALID_TYPES, "Node3D"), "set_anchor_b", "get_anchor_b");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "anchor_stiffness_compliance", PROPERTY_HINT_RANGE, "0.0,0.01,0.000001,or_greater"), "set_anchor_stiffness_compliance", "get_anchor_stiffness_compliance");

	ADD_GROUP("Material", "");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "mass_per_meter", PROPERTY_HINT_RANGE, "0.001,10.0,0.001,or_greater,suffix:kg/m"), "set_mass_per_meter", "get_mass_per_meter");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "stretch_compliance", PROPERTY_HINT_RANGE, "0.0,0.001,0.0000001,or_greater"), "set_stretch_compliance", "get_stretch_compliance");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "bending_compliance", PROPERTY_HINT_RANGE, "0.0,1.0,0.00001,or_greater"), "set_bending_compliance", "get_bending_compliance");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "minimum_bend_radius", PROPERTY_HINT_RANGE, "0.0,2.0,0.001,or_greater,suffix:m"), "set_minimum_bend_radius", "get_minimum_bend_radius");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "damping", PROPERTY_HINT_RANGE, "0.0,10.0,0.01"), "set_damping", "get_damping");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "static_friction", PROPERTY_HINT_RANGE, "0.0,2.0,0.01"), "set_static_friction", "get_static_friction");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "dynamic_friction", PROPERTY_HINT_RANGE, "0.0,2.0,0.01"), "set_dynamic_friction", "get_dynamic_friction");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "gravity_scale", PROPERTY_HINT_RANGE, "-2.0,2.0,0.01"), "set_gravity_scale", "get_gravity_scale");

	ADD_GROUP("Solver", "");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "substeps_min", PROPERTY_HINT_RANGE, "1,32,1"), "set_substeps_min", "get_substeps_min");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "substeps_max", PROPERTY_HINT_RANGE, "1,64,1"), "set_substeps_max", "get_substeps_max");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "solver_iterations", PROPERTY_HINT_RANGE, "1,8,1"), "set_solver_iterations", "get_solver_iterations");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "collision_margin", PROPERTY_HINT_RANGE, "0.0,0.05,0.0001,suffix:m"), "set_collision_margin", "get_collision_margin");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "self_collision_enabled"), "set_self_collision_enabled", "is_self_collision_enabled");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "sleep_enabled"), "set_sleep_enabled", "is_sleep_enabled");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "simulation_enabled"), "set_simulation_enabled", "is_simulation_enabled");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "solver_backend", PROPERTY_HINT_ENUM, "LEGACY_XPBD,IMPLICIT_V2"), "set_solver_backend", "get_solver_backend");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "axial_stiffness", PROPERTY_HINT_RANGE, "0.0,10000000.0,1.0,or_greater,suffix:N"), "set_axial_stiffness", "get_axial_stiffness");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "v2_bending_stiffness", PROPERTY_HINT_RANGE, "0.0,1000.0,0.0001,or_greater,suffix:N*m^2"), "set_v2_bending_stiffness", "get_v2_bending_stiffness");

	ADD_GROUP("Collision", "");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "collision_mask", PROPERTY_HINT_LAYERS_3D_PHYSICS), "set_collision_mask", "get_collision_mask");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "collide_with_bodies"), "set_collide_with_bodies", "is_collide_with_bodies");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "collide_with_areas"), "set_collide_with_areas", "is_collide_with_areas");

	ADD_GROUP("Rendering", "");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "radial_segments", PROPERTY_HINT_RANGE, "3,32,1"), "set_radial_segments", "get_radial_segments");
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "cable_material", PROPERTY_HINT_RESOURCE_TYPE, "BaseMaterial3D,ShaderMaterial"), "set_cable_material", "get_cable_material");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "debug_draw_enabled"), "set_debug_draw_enabled", "is_debug_draw_enabled");

	ADD_SIGNAL(MethodInfo("taut_limit_reached"));
}

void CableBody3D::_notification(int p_what) {
	if (p_what == NOTIFICATION_EXIT_TREE) {
		snapshot_builder.invalidate_cache();
	}
}

void CableBody3D::ensure_children() {
	if (mesh_instance == nullptr) {
		mesh_instance = memnew(MeshInstance3D);
		mesh_instance->set_name("CableMesh");
		add_child(mesh_instance, false, Node::INTERNAL_MODE_BACK);
	}
	if (debug_instance == nullptr) {
		debug_instance = memnew(MeshInstance3D);
		debug_instance->set_name("CableDebug");
		add_child(debug_instance, false, Node::INTERNAL_MODE_BACK);
	}
}

Node3D *CableBody3D::anchor_node(const NodePath &p_path) const {
	if (p_path.is_empty()) {
		return nullptr;
	}
	return Object::cast_to<Node3D>(get_node_or_null(p_path));
}

void CableBody3D::_ready() {
	ensure_children();

	const double gravity = ProjectSettings::get_singleton()->get_setting("physics/3d/default_gravity", 9.8);
	const Vector3 gravity_vector = ProjectSettings::get_singleton()->get_setting("physics/3d/default_gravity_vector", Vector3(0, -1, 0));
	solver.params.gravity = to_solver(gravity_vector.normalized() * gravity);
	implicit_solver.params.gravity = solver.params.gravity;

	rebuild_cable();
}

void CableBody3D::apply_parameter_change() {
	needs_rebuild = true;
	if (is_inside_tree()) {
		update_configuration_warnings();
	}
}

void CableBody3D::resolve_anchors() {
	// Якорь A по умолчанию — сама нода: кабель работоспособен сразу после добавления.
	Node3D *node_a = anchor_node(anchor_a_path);
	const Transform3D transform_a = node_a != nullptr ? node_a->get_global_transform() : get_global_transform();
	solver.anchor_start.active = true;
	solver.anchor_start.position = to_solver(transform_a.origin);
	// Кабель выходит из разъёма вдоль -Z локального базиса якоря.
	solver.anchor_start.direction = to_solver(-transform_a.basis.get_column(2).normalized());

	Node3D *node_b = anchor_node(anchor_b_path);
	solver.anchor_end.active = node_b != nullptr;
	if (node_b != nullptr) {
		const Transform3D transform_b = node_b->get_global_transform();
		solver.anchor_end.position = to_solver(transform_b.origin);
		solver.anchor_end.direction = to_solver(-transform_b.basis.get_column(2).normalized());
	}
}

void CableBody3D::rebuild_cable() {
	// Вне дерева глобальные трансформы недоступны и вернулись бы единичными, то есть
	// кабель собрался бы не в том месте. Пересборка откладывается до входа в дерево,
	// где её выполнит _physics_process по needs_rebuild.
	if (!is_inside_tree()) {
		needs_rebuild = true;
		return;
	}

	ensure_children();
	resolve_anchors();

	const Vector3 from = to_godot(solver.anchor_start.position);
	Vector3 to = from + Vector3(0, 0, -1) * static_cast<real_t>(solver.params.cable_length * 0.5);
	if (solver.anchor_end.active) {
		to = to_godot(solver.anchor_end.position);
	}

	solver.rebuild(to_solver(from), to_solver(to));
	if (solver_backend == IMPLICIT_V2) {
		implicit_solver.initialize(solver.state, solver.params.mass_per_meter);
	}
	snapshot_builder.invalidate_cache();

	mesh_builder.configure(radial_segments, solver.state.capacity());
	cable_mesh = mesh_builder.create_mesh();
	mesh_instance->set_mesh(cable_mesh);
	mesh_instance->set_material_override(cable_material);
	mesh_instance->set_transform(Transform3D());

	debug_mesh = debug_draw.create_mesh();
	debug_instance->set_mesh(debug_mesh);
	debug_instance->set_visible(debug_draw_enabled);
	debug_instance->set_transform(Transform3D());

	last_uv_length = -1.0f;
	needs_rebuild = false;
	prerelaxed = false;
	taut_limit_active = false;

	if (Engine::get_singleton()->is_editor_hint()) {
		// В редакторе физика не крутится каждый кадр: показывается форма покоя,
		// посчитанная без геометрии мира.
		snapshot.begin_frame();
		if (solver_backend == LEGACY_XPBD) {
			solver.prerelax(snapshot);
		}
		prerelaxed = true;
	}

	store_render_snapshot();
	render_previous = render_current;
	update_mesh(1.0);
}

void CableBody3D::update_anchor_inputs(double p_delta) {
	const float inv_delta = p_delta > 0.0 ? static_cast<float>(1.0 / p_delta) : 0.0f;

	Node3D *node_a = anchor_node(anchor_a_path);
	const Transform3D transform_a = node_a != nullptr ? node_a->get_global_transform() : get_global_transform();
	const Vec3 position_a = to_solver(transform_a.origin);
	// Скорость якоря берётся из его собственного перемещения за кадр; прыжок больше
	// порога солвер трактует как телепорт и не превращает в скорость.
	solver.anchor_start.velocity = (position_a - solver.anchor_start.position) * inv_delta;
	solver.anchor_start.position = position_a;
	solver.anchor_start.direction = to_solver(-transform_a.basis.get_column(2).normalized());

	Node3D *node_b = anchor_node(anchor_b_path);
	const bool had_end = solver.anchor_end.active;
	solver.anchor_end.active = node_b != nullptr;
	if (node_b != nullptr) {
		const Transform3D transform_b = node_b->get_global_transform();
		const Vec3 position_b = to_solver(transform_b.origin);
		solver.anchor_end.velocity = (position_b - solver.anchor_end.position) * inv_delta;
		solver.anchor_end.position = position_b;
		solver.anchor_end.direction = to_solver(-transform_b.basis.get_column(2).normalized());
		if (!had_end) {
			needs_rebuild = true;
		}
	} else if (had_end) {
		needs_rebuild = true;
	}
}

void CableBody3D::build_snapshot() {
	// Зона интереса: AABB кабеля, расширенный на максимальное перемещение за кадр
	// плюс запас.
	cable::Bounds interest = solver.state.compute_bounds(solver.params.cable_radius);
	if (!interest.is_valid()) {
		snapshot.begin_frame();
		return;
	}
	interest.grow(solver.params.effective_segment_length() * 4.0f + solver.params.collision_margin);

	snapshot_builder.set_collision_mask(collision_mask);
	snapshot_builder.set_collide_with_bodies(collide_with_bodies);
	snapshot_builder.set_collide_with_areas(collide_with_areas);
	snapshot_builder.set_exceptions(exceptions);
	snapshot_builder.build(this, interest, snapshot);
}

void CableBody3D::store_render_snapshot() {
	render_previous = render_current;
	render_current.assign(solver.state.position.begin(), solver.state.position.begin() + solver.state.count);
	if (render_previous.size() != render_current.size()) {
		render_previous = render_current;
	}
}

void CableBody3D::_physics_process(double p_delta) {
	if (needs_rebuild) {
		rebuild_cable();
	}
	if (Engine::get_singleton()->is_editor_hint() || !simulation_enabled || !solver.is_initialized()) {
		return;
	}

	update_anchor_inputs(p_delta);
	if (solver_backend == LEGACY_XPBD) {
		build_snapshot();
		if (!prerelaxed) {
			solver.prerelax(snapshot);
			prerelaxed = true;
		}
		solver.step(static_cast<float>(p_delta), snapshot);
	} else {
		const Vec3 *end = solver.anchor_end.active ? &solver.anchor_end.position : nullptr;
		implicit_solver.step(p_delta, solver.anchor_start.position, end);
		implicit_solver.copy_to(solver.state);
		prerelaxed = true;
	}
	store_render_snapshot();

	const bool exhausted = solver.get_diagnostics().length_exhausted;
	if (exhausted && !taut_limit_active) {
		emit_signal("taut_limit_reached");
	}
	taut_limit_active = exhausted;
}

void CableBody3D::_process(double p_delta) {
	if (!solver.is_initialized() || cable_mesh.is_null()) {
		return;
	}
	const double fraction = Engine::get_singleton()->is_editor_hint()
			? 1.0
			: Engine::get_singleton()->get_physics_interpolation_fraction();
	update_mesh(fraction);
}

void CableBody3D::update_mesh(double p_interpolation) {
	const int count = static_cast<int>(render_current.size());
	if (count < 2) {
		return;
	}

	render_interpolated.resize(static_cast<size_t>(count));
	const float t = static_cast<float>(cable::clampf(static_cast<float>(p_interpolation), 0.0f, 1.0f));
	const bool can_interpolate = render_previous.size() == render_current.size();
	for (int i = 0; i < count; ++i) {
		render_interpolated[i] = can_interpolate ? cable::lerp(render_previous[i], render_current[i], t) : render_current[i];
	}

	const Transform3D to_local = world_to_local();
	mesh_builder.update(cable_mesh, render_interpolated.data(), count, to_local, solver.params.cable_radius);

	// UV лежат в буфере атрибутов и обновляются только при изменении размотки.
	const float deployed = static_cast<float>(solver.get_diagnostics().deployed_length);
	if (std::fabs(deployed - last_uv_length) > cable::EPS_LENGTH) {
		mesh_builder.update_uv(cable_mesh, solver.state.arc_from_start.data(), count);
		last_uv_length = deployed;
	}

	// custom_aabb выставляется вручную: движок не знает, что вершины изменились.
	AABB aabb = mesh_builder.get_local_aabb();
	aabb = aabb.grow(solver.params.cable_radius);
	mesh_instance->set_custom_aabb(aabb);

#ifdef DEBUG_ENABLED
	if (debug_draw_enabled && debug_mesh.is_valid()) {
		debug_draw.update(debug_mesh, solver, snapshot, to_local);
	}
#endif
}

Transform3D CableBody3D::world_to_local() const {
	// Единственное место, где world space солвера переходит в local space: граница
	// рендеринга.
	return get_global_transform().affine_inverse();
}

PackedVector3Array CableBody3D::get_centerline() const {
	PackedVector3Array out;
	out.resize(solver.state.count);
	for (int i = 0; i < solver.state.count; ++i) {
		out[i] = to_godot(solver.state.position[i]);
	}
	return out;
}

Vector3 CableBody3D::constrain_motion(const Vector3 &p_from, const Vector3 &p_to) {
	bool limited = false;
	const Vec3 allowed = solver.constrain_player_motion(to_solver(p_from), to_solver(p_to), limited);
	if (limited && !taut_limit_active) {
		emit_signal("taut_limit_reached");
		taut_limit_active = true;
	}
	return to_godot(allowed);
}

double CableBody3D::get_remaining_length() const {
	return solver.get_diagnostics().stowed_length;
}

double CableBody3D::get_deployed_length() const {
	return solver.get_diagnostics().deployed_length;
}

double CableBody3D::get_tension() const {
	return solver.get_diagnostics().max_tension;
}

Vector3 CableBody3D::get_tension_direction() const {
	return to_godot(solver.get_diagnostics().tension_direction);
}

int CableBody3D::get_solver_iteration_count() const {
	return solver_backend == IMPLICIT_V2
			? implicit_solver.get_diagnostics().newton_iterations
			: solver.get_diagnostics().solver_iterations;
}

double CableBody3D::get_max_stretch_error() const {
	return solver_backend == IMPLICIT_V2
			? implicit_solver.get_diagnostics().max_stretch_error
			: solver.get_diagnostics().max_stretch_error;
}

PackedStringArray CableBody3D::_get_configuration_warnings() const {
	PackedStringArray warnings = snapshot_builder.get_warnings();

	if (solver.params.target_segment_length < solver.params.min_segment_length()) {
		warnings.push_back("target_segment_length меньше 2 * cable_radius: более мелкие сегменты делают самопересечение вырожденным и выигрыша не дают.");
	}
	const float needed = solver.params.cable_length / solver.params.effective_segment_length();
	if (needed > static_cast<float>(cable::MAX_PARTICLES)) {
		warnings.push_back(vformat("cable_length требует %d частиц, предел %d: увеличьте target_segment_length.",
				static_cast<int>(needed), cable::MAX_PARTICLES));
	}
	if (anchor_b_path.is_empty()) {
		warnings.push_back("anchor_b не задан: конец кабеля свободен, катушка и ограничение перемещения игрока неактивны.");
	}
	if (solver_backend == IMPLICIT_V2) {
		warnings.push_back("IMPLICIT_V2 — contact-free profiling prototype: world/self contacts, friction, CCD, катушка и sleeping отключены.");
	}
	warnings.push_back("constrain_motion() вызывается контроллером игрока ДО перемещения, как move_and_slide; вызов после перемещения даёт рывок.");
	return warnings;
}

void CableBody3D::set_cable_length(double p_length) {
	solver.params.cable_length = cable::maxf(static_cast<float>(p_length), 0.1f);
	apply_parameter_change();
}

void CableBody3D::set_cable_radius(double p_radius) {
	solver.params.cable_radius = cable::maxf(static_cast<float>(p_radius), 1.0e-3f);
	apply_parameter_change();
}

void CableBody3D::set_target_segment_length(double p_length) {
	solver.params.target_segment_length = cable::maxf(static_cast<float>(p_length), 5.0e-3f);
	apply_parameter_change();
}

void CableBody3D::set_mass_per_meter(double p_value) {
	solver.params.mass_per_meter = cable::maxf(static_cast<float>(p_value), 1.0e-3f);
	solver.state.distribute_masses(solver.params);
	if (solver_backend == IMPLICIT_V2) {
		apply_parameter_change();
	}
}

void CableBody3D::set_stretch_compliance(double p_value) {
	solver.params.stretch_compliance = cable::maxf(static_cast<float>(p_value), 0.0f);
}

void CableBody3D::set_bending_compliance(double p_value) {
	solver.params.bending_compliance = cable::maxf(static_cast<float>(p_value), 0.0f);
}

void CableBody3D::set_minimum_bend_radius(double p_value) {
	solver.params.minimum_bend_radius = cable::maxf(static_cast<float>(p_value), 0.0f);
}

void CableBody3D::set_damping(double p_value) {
	solver.params.damping = cable::maxf(static_cast<float>(p_value), 0.0f);
}

void CableBody3D::set_static_friction(double p_value) {
	solver.params.static_friction = cable::maxf(static_cast<float>(p_value), 0.0f);
}

void CableBody3D::set_dynamic_friction(double p_value) {
	solver.params.dynamic_friction = cable::maxf(static_cast<float>(p_value), 0.0f);
}

void CableBody3D::set_gravity_scale(double p_value) {
	solver.params.gravity_scale = static_cast<float>(p_value);
	implicit_solver.params.gravity_scale = p_value;
}

void CableBody3D::set_anchor_stiffness_compliance(double p_value) {
	solver.params.anchor_stiffness_compliance = cable::maxf(static_cast<float>(p_value), 0.0f);
}

void CableBody3D::set_substeps_min(int p_value) {
	solver.params.substeps_min = p_value < 1 ? 1 : p_value;
	if (solver.params.substeps_max < solver.params.substeps_min) {
		solver.params.substeps_max = solver.params.substeps_min;
	}
}

void CableBody3D::set_substeps_max(int p_value) {
	solver.params.substeps_max = p_value < 1 ? 1 : p_value;
	if (solver.params.substeps_min > solver.params.substeps_max) {
		solver.params.substeps_min = solver.params.substeps_max;
	}
}

void CableBody3D::set_solver_iterations(int p_value) {
	solver.params.solver_iterations = p_value < 1 ? 1 : p_value;
}

void CableBody3D::set_collision_margin(double p_value) {
	solver.params.collision_margin = cable::maxf(static_cast<float>(p_value), 0.0f);
}

void CableBody3D::set_self_collision_enabled(bool p_enabled) {
	solver.params.self_collision_enabled = p_enabled;
}

void CableBody3D::set_sleep_enabled(bool p_enabled) {
	solver.params.sleep_enabled = p_enabled;
}

void CableBody3D::set_solver_backend(SolverBackend p_backend) {
	if (p_backend != LEGACY_XPBD && p_backend != IMPLICIT_V2) {
		return;
	}
	if (solver_backend == p_backend) {
		return;
	}
	solver_backend = p_backend;
	apply_parameter_change();
}

void CableBody3D::set_axial_stiffness(double p_value) {
	implicit_solver.params.axial_stiffness = p_value > 0.0 ? p_value : 0.0;
}

void CableBody3D::set_v2_bending_stiffness(double p_value) {
	implicit_solver.params.bending_stiffness = p_value > 0.0 ? p_value : 0.0;
}

void CableBody3D::set_collision_mask(int p_mask) {
	collision_mask = static_cast<uint32_t>(p_mask);
	snapshot_builder.invalidate_cache();
}

void CableBody3D::set_collide_with_bodies(bool p_enabled) {
	collide_with_bodies = p_enabled;
	snapshot_builder.invalidate_cache();
}

void CableBody3D::set_collide_with_areas(bool p_enabled) {
	collide_with_areas = p_enabled;
	snapshot_builder.invalidate_cache();
}

void CableBody3D::add_collision_exception(Node *p_node) {
	if (p_node != nullptr) {
		exceptions.insert(p_node->get_instance_id());
	}
}

void CableBody3D::remove_collision_exception(Node *p_node) {
	if (p_node != nullptr) {
		exceptions.erase(p_node->get_instance_id());
	}
}

void CableBody3D::clear_collision_exceptions() {
	exceptions.clear();
}

void CableBody3D::set_anchor_a(const NodePath &p_path) {
	anchor_a_path = p_path;
	apply_parameter_change();
}

void CableBody3D::set_anchor_b(const NodePath &p_path) {
	anchor_b_path = p_path;
	apply_parameter_change();
}

void CableBody3D::set_radial_segments(int p_value) {
	radial_segments = p_value < 3 ? 3 : p_value;
	apply_parameter_change();
}

void CableBody3D::set_cable_material(const Ref<Material> &p_material) {
	cable_material = p_material;
	if (mesh_instance != nullptr) {
		mesh_instance->set_material_override(cable_material);
	}
}

void CableBody3D::set_debug_draw_enabled(bool p_enabled) {
	debug_draw_enabled = p_enabled;
	if (debug_instance != nullptr) {
		debug_instance->set_visible(p_enabled);
	}
}

void CableBody3D::set_simulation_enabled(bool p_enabled) {
	simulation_enabled = p_enabled;
}
