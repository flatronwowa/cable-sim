#include "nodes/cable_debug_draw.h"

using namespace godot;
using cable::Vec3;

namespace {

const Color COLOR_AWAKE(0.2f, 1.0f, 0.3f);
const Color COLOR_ASLEEP(0.3f, 0.4f, 0.9f);
const Color COLOR_CONTACT(1.0f, 0.8f, 0.1f);
const Color COLOR_NORMAL(1.0f, 0.2f, 0.2f);
const Color COLOR_ANCHOR(1.0f, 0.4f, 1.0f);
const Color COLOR_INTEREST(0.4f, 0.4f, 0.4f);

Vector3 to_godot(const Vec3 &v) {
	return Vector3(v.x, v.y, v.z);
}

void add_line(const Ref<ImmediateMesh> &p_mesh, const Vector3 &p_from, const Vector3 &p_to, const Color &p_color) {
	p_mesh->surface_set_color(p_color);
	p_mesh->surface_add_vertex(p_from);
	p_mesh->surface_set_color(p_color);
	p_mesh->surface_add_vertex(p_to);
}

void add_cross(const Ref<ImmediateMesh> &p_mesh, const Vector3 &p_center, float p_size, const Color &p_color) {
	add_line(p_mesh, p_center - Vector3(p_size, 0, 0), p_center + Vector3(p_size, 0, 0), p_color);
	add_line(p_mesh, p_center - Vector3(0, p_size, 0), p_center + Vector3(0, p_size, 0), p_color);
	add_line(p_mesh, p_center - Vector3(0, 0, p_size), p_center + Vector3(0, 0, p_size), p_color);
}

void add_box(const Ref<ImmediateMesh> &p_mesh, const cable::Bounds &p_bounds, const Transform3D &p_to_local, const Color &p_color) {
	if (!p_bounds.is_valid()) {
		return;
	}
	Vector3 corner[8];
	for (int i = 0; i < 8; ++i) {
		corner[i] = p_to_local.xform(Vector3(
				(i & 1) ? p_bounds.max.x : p_bounds.min.x,
				(i & 2) ? p_bounds.max.y : p_bounds.min.y,
				(i & 4) ? p_bounds.max.z : p_bounds.min.z));
	}
	const int edges[12][2] = { { 0, 1 }, { 2, 3 }, { 4, 5 }, { 6, 7 }, { 0, 2 }, { 1, 3 }, { 4, 6 }, { 5, 7 }, { 0, 4 }, { 1, 5 }, { 2, 6 }, { 3, 7 } };
	for (int e = 0; e < 12; ++e) {
		add_line(p_mesh, corner[edges[e][0]], corner[edges[e][1]], p_color);
	}
}

} // namespace

Ref<ImmediateMesh> CableDebugDraw::create_mesh() {
	if (material.is_null()) {
		material.instantiate();
		material->set_shading_mode(BaseMaterial3D::SHADING_MODE_UNSHADED);
		material->set_flag(BaseMaterial3D::FLAG_ALBEDO_FROM_VERTEX_COLOR, true);
		material->set_flag(BaseMaterial3D::FLAG_DISABLE_DEPTH_TEST, true);
	}
	Ref<ImmediateMesh> mesh;
	mesh.instantiate();
	return mesh;
}

void CableDebugDraw::update(const Ref<ImmediateMesh> &p_mesh, const cable::CableSolver &p_solver,
		const cable::WorldSnapshot &p_world, const Transform3D &p_world_to_local) {
	if (p_mesh.is_null()) {
		return;
	}
	p_mesh->clear_surfaces();
	const cable::CableState &state = p_solver.state;
	if (state.count < 2) {
		return;
	}

	p_mesh->surface_begin(Mesh::PRIMITIVE_LINES, material);

	const float marker = p_solver.params.cable_radius * 1.5f;
	const float max_tension = cable::maxf(p_solver.get_diagnostics().max_tension, 1.0f);

	for (int i = 0; i < state.count; ++i) {
		const Vector3 point = p_world_to_local.xform(to_godot(state.position[i]));
		add_cross(p_mesh, point, marker, state.is_asleep(i) ? COLOR_ASLEEP : COLOR_AWAKE);

		if (i + 1 < state.count) {
			// Цвет сегмента по натяжению: от зелёного в покое до красного под нагрузкой.
			const float load = cable::clampf(std::fabs(state.lambda_stretch[i]) / max_tension, 0.0f, 1.0f);
			const Color color(load, 1.0f - load, 0.2f);
			add_line(p_mesh, point, p_world_to_local.xform(to_godot(state.position[i + 1])), color);
		}

		const cable::ContactAnchor &anchor = state.contacts[i];
		if (anchor.collider_key == 0) {
			continue;
		}
		add_line(p_mesh, point, point + p_world_to_local.basis.xform(to_godot(anchor.normal)) * marker * 6.0f, COLOR_NORMAL);
		const int shape = p_world.find_shape(anchor.collider_key);
		if (shape >= 0) {
			add_cross(p_mesh, p_world_to_local.xform(to_godot(p_world.to_world(shape, anchor.anchor_local))), marker * 0.6f, COLOR_ANCHOR);
		} else {
			add_cross(p_mesh, point, marker * 0.6f, COLOR_CONTACT);
		}
	}

	add_box(p_mesh, state.compute_bounds(p_solver.params.cable_radius), p_world_to_local, COLOR_INTEREST);
	p_mesh->surface_end();
}
