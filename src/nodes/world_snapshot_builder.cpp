#include "nodes/world_snapshot_builder.h"

#include <godot_cpp/classes/capsule_shape3d.hpp>
#include <godot_cpp/classes/collision_object3d.hpp>
#include <godot_cpp/classes/concave_polygon_shape3d.hpp>
#include <godot_cpp/classes/convex_polygon_shape3d.hpp>
#include <godot_cpp/classes/cylinder_shape3d.hpp>
#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/classes/physics_body3d.hpp>
#include <godot_cpp/classes/physics_direct_space_state3d.hpp>
#include <godot_cpp/classes/sphere_shape3d.hpp>
#include <godot_cpp/classes/static_body3d.hpp>
#include <godot_cpp/classes/viewport.hpp>
#include <godot_cpp/classes/world3d.hpp>
#include <godot_cpp/classes/world_boundary_shape3d.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

using namespace godot;
using cable::ConvexPlane;
using cable::ShapeData;
using cable::ShapeType;
using cable::Vec3;

namespace {

constexpr int MAX_QUERY_RESULTS = 64;
// Выше этого числа точек перебор опорных плоскостей становится дорогим, и форма
// заменяется своим AABB с предупреждением, а не молча тормозит.
constexpr int MAX_CONVEX_POINTS = 48;

Vec3 to_solver(const Vector3 &v) {
	return Vec3(static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z));
}

cable::Xform to_solver_xform(const Transform3D &p_transform) {
	cable::Xform out;
	out.basis.col[0] = to_solver(p_transform.basis.get_column(0));
	out.basis.col[1] = to_solver(p_transform.basis.get_column(1));
	out.basis.col[2] = to_solver(p_transform.basis.get_column(2));
	out.origin = to_solver(p_transform.origin);
	return out;
}

cable::Bounds transform_bounds(const cable::Xform &p_transform, const cable::Bounds &p_local) {
	cable::Bounds out;
	if (!p_local.is_valid()) {
		return out;
	}
	for (int corner = 0; corner < 8; ++corner) {
		const Vec3 point(
				(corner & 1) ? p_local.max.x : p_local.min.x,
				(corner & 2) ? p_local.max.y : p_local.min.y,
				(corner & 4) ? p_local.max.z : p_local.min.z);
		out.expand(cable::xform_point(p_transform, point));
	}
	return out;
}

// Опорные плоскости выпуклой оболочки восстанавливаются перебором троек точек:
// ConvexPolygonShape3D отдаёт только вершины. Операция одноразовая и кешируется.
void build_convex_planes(const PackedVector3Array &p_points, const Vector3 &p_scale, std::vector<ConvexPlane> &r_planes) {
	r_planes.clear();
	const int count = p_points.size();
	if (count < 4) {
		return;
	}

	std::vector<Vec3> points;
	points.reserve(static_cast<size_t>(count));
	float extent = 1.0f;
	for (int i = 0; i < count; ++i) {
		const Vector3 scaled = p_points[i] * p_scale;
		points.push_back(to_solver(scaled));
		extent = cable::maxf(extent, cable::length(points.back()));
	}
	const float tolerance = 1.0e-4f * extent;

	for (int i = 0; i < count; ++i) {
		for (int j = i + 1; j < count; ++j) {
			for (int k = j + 1; k < count; ++k) {
				Vec3 normal = cable::cross(points[j] - points[i], points[k] - points[i]);
				normal = cable::safe_normalize(normal);
				if (cable::length_squared(normal) == 0.0f) {
					continue;
				}
				float d = cable::dot(normal, points[i]);

				int side = 0;
				bool supporting = true;
				for (int m = 0; m < count; ++m) {
					const float signed_distance = cable::dot(normal, points[m]) - d;
					if (signed_distance > tolerance) {
						if (side < 0) {
							supporting = false;
							break;
						}
						side = 1;
					} else if (signed_distance < -tolerance) {
						if (side > 0) {
							supporting = false;
							break;
						}
						side = -1;
					}
				}
				if (!supporting || side == 0) {
					continue;
				}
				// Оболочка лежит с одной стороны; нормаль разворачивается наружу.
				if (side > 0) {
					normal = -normal;
					d = -d;
				}

				bool duplicate = false;
				for (const ConvexPlane &existing : r_planes) {
					if (cable::dot(existing.normal, normal) > 0.9999f && std::fabs(existing.d - d) < tolerance) {
						duplicate = true;
						break;
					}
				}
				if (!duplicate) {
					r_planes.push_back({ normal, d });
				}
			}
		}
	}
}

uint64_t make_collider_key(uint64_t p_instance_id, int p_shape_sequence) {
	return p_instance_id ^ (static_cast<uint64_t>(p_shape_sequence + 1) * 0x9E3779B97F4A7C15ull);
}

} // namespace

WorldSnapshotBuilder::WorldSnapshotBuilder() {
	query_box.instantiate();
	query_params.instantiate();
}

void WorldSnapshotBuilder::invalidate_cache() {
	cache.clear();
}

void WorldSnapshotBuilder::emit_warning_once(const String &p_message) {
	if (reported_warnings.has(p_message)) {
		return;
	}
	reported_warnings.insert(p_message);
	warnings.push_back(p_message);
	UtilityFunctions::push_warning(p_message);
}

bool WorldSnapshotBuilder::extract_shape(const Ref<Shape3D> &p_shape, const Transform3D &p_transform, uint64_t p_key,
		bool p_is_static, uint32_t p_layer, CachedCollider &r_cached) {
	ShapeData &data = r_cached.data;
	data.key = p_key;
	data.layer = p_layer;
	data.is_static = p_is_static;

	// Масштаб запекается в параметры формы, а в трансформе остаётся ортонормированный
	// базис: обращение такого преобразования — транспонирование, без матричной инверсии.
	const Vector3 scale = p_transform.basis.get_scale().abs();
	Transform3D rigid = p_transform;
	rigid.basis = p_transform.basis.orthonormalized();
	data.transform = to_solver_xform(rigid);
	r_cached.last_scale = scale;

	cable::Bounds local;

	if (Ref<BoxShape3D> box = p_shape; box.is_valid()) {
		data.type = ShapeType::BOX;
		const Vector3 half = box->get_size() * 0.5f * scale;
		data.half_extents = to_solver(half);
		local.expand(-data.half_extents);
		local.expand(data.half_extents);
	} else if (Ref<SphereShape3D> sphere = p_shape; sphere.is_valid()) {
		data.type = ShapeType::SPHERE;
		data.radius = static_cast<float>(sphere->get_radius()) * static_cast<float>(scale.y > scale.x ? (scale.y > scale.z ? scale.y : scale.z) : (scale.x > scale.z ? scale.x : scale.z));
		local.expand(Vec3(-data.radius, -data.radius, -data.radius));
		local.expand(Vec3(data.radius, data.radius, data.radius));
	} else if (Ref<CapsuleShape3D> capsule = p_shape; capsule.is_valid()) {
		data.type = ShapeType::CAPSULE;
		data.radius = static_cast<float>(capsule->get_radius()) * static_cast<float>(scale.x > scale.z ? scale.x : scale.z);
		data.height = static_cast<float>(capsule->get_height()) * static_cast<float>(scale.y);
		const float half_height = cable::maxf(data.height * 0.5f, data.radius);
		local.expand(Vec3(-data.radius, -half_height, -data.radius));
		local.expand(Vec3(data.radius, half_height, data.radius));
	} else if (Ref<CylinderShape3D> cylinder = p_shape; cylinder.is_valid()) {
		data.type = ShapeType::CYLINDER;
		data.radius = static_cast<float>(cylinder->get_radius()) * static_cast<float>(scale.x > scale.z ? scale.x : scale.z);
		data.height = static_cast<float>(cylinder->get_height()) * static_cast<float>(scale.y);
		local.expand(Vec3(-data.radius, -data.height * 0.5f, -data.radius));
		local.expand(Vec3(data.radius, data.height * 0.5f, data.radius));
	} else if (Ref<WorldBoundaryShape3D> boundary = p_shape; boundary.is_valid()) {
		data.type = ShapeType::PLANE;
		const Plane plane = boundary->get_plane();
		const Vector3 world_normal = rigid.basis.xform(plane.normal).normalized();
		data.plane_normal = to_solver(world_normal);
		data.plane_d = static_cast<float>(plane.d) + static_cast<float>(world_normal.dot(rigid.origin));
		r_cached.has_local_bounds = false;
		r_cached.parameters_valid = true;
		return true;
	} else if (Ref<ConvexPolygonShape3D> convex = p_shape; convex.is_valid()) {
		const PackedVector3Array points = convex->get_points();
		if (points.size() > MAX_CONVEX_POINTS) {
			emit_warning_once(vformat("CableBody3D: выпуклая оболочка с %d вершинами заменена своим AABB, порог %d.", points.size(), MAX_CONVEX_POINTS));
		} else {
			build_convex_planes(points, scale, r_cached.planes);
		}
		for (int i = 0; i < points.size(); ++i) {
			local.expand(to_solver(points[i] * scale));
		}
		if (r_cached.planes.empty()) {
			// Запасной путь: вырожденная или слишком тяжёлая оболочка становится боксом,
			// симметричным относительно локального начала, чтобы трансформ остался общим.
			data.type = ShapeType::BOX;
			data.half_extents = cable::vec_max(cable::vec_abs(local.min), cable::vec_abs(local.max));
			local.min = -data.half_extents;
			local.max = data.half_extents;
		} else {
			data.type = ShapeType::CONVEX;
		}
	} else if (Ref<ConcavePolygonShape3D> concave = p_shape; concave.is_valid()) {
		data.type = ShapeType::MESH;
		const PackedVector3Array faces = concave->get_faces();
		const int triangles = faces.size() / 3;
		if (triangles <= 0) {
			r_cached.parameters_valid = false;
			return false;
		}
		std::vector<Vec3> scaled;
		scaled.reserve(static_cast<size_t>(triangles) * 3);
		for (int i = 0; i < triangles * 3; ++i) {
			scaled.push_back(to_solver(faces[i] * scale));
		}
		// Построение BVH — операция не для кадра, поэтому она идёт при первом
		// появлении сетки в зоне интереса и кешируется на время жизни коллайдера.
		r_cached.mesh = std::make_shared<cable::CollisionMesh>();
		r_cached.mesh->build(scaled.data(), triangles);
		if (!r_cached.mesh->is_valid()) {
			r_cached.parameters_valid = false;
			return false;
		}
		if (!r_cached.mesh->is_watertight()) {
			emit_warning_once("CableBody3D: незамкнутая ConcavePolygonShape3D, используется беззнаковое расстояние; сторона определяется по предыдущей позиции частицы.");
		}
		local = r_cached.mesh->get_local_bounds();
	} else {
		emit_warning_once("CableBody3D: неподдерживаемый тип Shape3D в зоне интереса пропущен.");
		r_cached.parameters_valid = false;
		return false;
	}

	r_cached.local_bounds = local;
	r_cached.has_local_bounds = true;
	r_cached.parameters_valid = true;
	return true;
}

bool WorldSnapshotBuilder::refresh_cached(const Ref<Shape3D> &p_shape, const Transform3D &p_transform, uint64_t p_key,
		bool p_is_static, uint32_t p_layer, CachedCollider &r_cached) {
	r_cached.moved_this_frame = !r_cached.parameters_valid || !r_cached.last_transform.is_equal_approx(p_transform);
	if (!r_cached.moved_this_frame) {
		return true;
	}

	const Vector3 scale = p_transform.basis.get_scale().abs();
	// Плоскости оболочки и BVH сетки зависят только от масштаба, поэтому обычное
	// перемещение статического коллайдера их не пересобирает.
	const bool geometry_reusable = r_cached.parameters_valid &&
			r_cached.data.type != ShapeType::PLANE &&
			r_cached.last_scale.is_equal_approx(scale);

	if (geometry_reusable) {
		Transform3D rigid = p_transform;
		rigid.basis = p_transform.basis.orthonormalized();
		r_cached.data.transform = to_solver_xform(rigid);
	} else {
		r_cached.planes.clear();
		r_cached.mesh.reset();
		if (!extract_shape(p_shape, p_transform, p_key, p_is_static, p_layer, r_cached)) {
			return false;
		}
	}

	r_cached.last_transform = p_transform;
	r_cached.last_scale = scale;
	return true;
}

void WorldSnapshotBuilder::build(Node3D *p_context, const cable::Bounds &p_interest, cable::WorldSnapshot &r_snapshot) {
	r_snapshot.begin_frame();
	if (p_context == nullptr || !p_interest.is_valid()) {
		return;
	}
	Viewport *viewport = p_context->get_viewport();
	if (viewport == nullptr) {
		return;
	}
	Ref<World3D> world = viewport->find_world_3d();
	if (world.is_null()) {
		return;
	}
	PhysicsDirectSpaceState3D *space = world->get_direct_space_state();
	if (space == nullptr) {
		return;
	}

	frame_counter += 1;

	const Vector3 minimum(p_interest.min.x, p_interest.min.y, p_interest.min.z);
	const Vector3 maximum(p_interest.max.x, p_interest.max.y, p_interest.max.z);
	query_box->set_size(maximum - minimum);
	query_params->set_shape(query_box);
	query_params->set_transform(Transform3D(Basis(), (minimum + maximum) * 0.5));
	query_params->set_collision_mask(collision_mask);
	query_params->set_collide_with_bodies(collide_with_bodies);
	query_params->set_collide_with_areas(collide_with_areas);

	const TypedArray<Dictionary> results = space->intersect_shape(query_params, MAX_QUERY_RESULTS);

	HashSet<uint64_t> visited;
	for (int i = 0; i < results.size(); ++i) {
		const Dictionary result = results[i];
		Object *object = result["collider"];
		CollisionObject3D *collider = Object::cast_to<CollisionObject3D>(object);
		if (collider == nullptr) {
			continue;
		}
		const uint64_t instance_id = collider->get_instance_id();
		if (exceptions.has(instance_id) || visited.has(instance_id)) {
			continue;
		}
		visited.insert(instance_id);

		// Один и тот же CollisionObject3D может дать несколько результатов запроса;
		// формы перебираются по владельцам один раз, лишние отсечёт AABB широкой фазы.
		const bool is_static = Object::cast_to<StaticBody3D>(collider) != nullptr;
		const Transform3D object_transform = collider->get_global_transform();
		const PackedInt32Array owners = collider->get_shape_owners();

		int shape_sequence = 0;
		for (int o = 0; o < owners.size(); ++o) {
			const uint32_t owner_id = static_cast<uint32_t>(owners[o]);
			if (collider->is_shape_owner_disabled(owner_id)) {
				shape_sequence += collider->shape_owner_get_shape_count(owner_id);
				continue;
			}
			const Transform3D owner_transform = object_transform * collider->shape_owner_get_transform(owner_id);
			const int shape_count = collider->shape_owner_get_shape_count(owner_id);

			for (int s = 0; s < shape_count; ++s, ++shape_sequence) {
				const Ref<Shape3D> shape = collider->shape_owner_get_shape(owner_id, s);
				if (shape.is_null()) {
					continue;
				}
				const uint64_t key = make_collider_key(instance_id, shape_sequence);

				CachedCollider &cached = cache[key];
				if (!refresh_cached(shape, owner_transform, key, is_static, collider->get_collision_layer(), cached)) {
					continue;
				}
				cached.last_frame = frame_counter;

				ShapeData data = cached.data;
				data.moved_this_frame = cached.moved_this_frame;
				if (cached.has_local_bounds) {
					data.world_aabb = transform_bounds(data.transform, cached.local_bounds);
					if (!data.world_aabb.overlaps(p_interest)) {
						continue;
					}
				}
				if (data.type == ShapeType::CONVEX) {
					data.convex_plane_first = r_snapshot.add_convex_planes(cached.planes.data(), static_cast<uint32_t>(cached.planes.size()));
					data.convex_plane_count = static_cast<uint32_t>(cached.planes.size());
				}
				if (data.type == ShapeType::MESH) {
					if (!cached.mesh) {
						continue;
					}
					data.mesh_index = r_snapshot.add_mesh(cached.mesh.get());
				}
				r_snapshot.add_shape(data);
			}
		}
	}

	// Коллайдеры, не встреченные несколько кадров, выпадают из кеша: иначе он растёт
	// вместе с пройденным уровнем.
	for (auto it = cache.begin(); it != cache.end();) {
		if (frame_counter - it->second.last_frame > 300) {
			it = cache.erase(it);
		} else {
			++it;
		}
	}
}
