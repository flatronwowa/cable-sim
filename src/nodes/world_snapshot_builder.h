#pragma once

#include "solver/collision_mesh.h"
#include "solver/world_snapshot.h"

#include <godot_cpp/classes/box_shape3d.hpp>
#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/classes/physics_shape_query_parameters3d.hpp>
#include <godot_cpp/classes/shape3d.hpp>
#include <godot_cpp/templates/hash_set.hpp>

#include <memory>
#include <unordered_map>
#include <vector>

namespace godot {

// Единственное место во всём проекте, обращающееся к PhysicsDirectSpaceState3D и к
// Shape3D. Переписывает коллайдеры зоны интереса в плоские структуры солвера.
class WorldSnapshotBuilder {
public:
	WorldSnapshotBuilder();

	void set_collision_mask(uint32_t p_mask) { collision_mask = p_mask; }
	void set_collide_with_bodies(bool p_enable) { collide_with_bodies = p_enable; }
	void set_collide_with_areas(bool p_enable) { collide_with_areas = p_enable; }
	void set_exceptions(const HashSet<uint64_t> &p_exceptions) { exceptions = p_exceptions; }

	void build(Node3D *p_context, const cable::Bounds &p_interest, cable::WorldSnapshot &r_snapshot);
	void invalidate_cache();

	const PackedStringArray &get_warnings() const { return warnings; }

private:
	// Статические коллайдеры не пересобираются, пока их трансформ не изменился:
	// пересчёт плоскостей выпуклой оболочки и BVH сетки — операции не для кадра.
	struct CachedCollider {
		cable::ShapeData data;
		Transform3D last_transform;
		Vector3 last_scale;
		std::vector<cable::ConvexPlane> planes;
		std::shared_ptr<cable::CollisionMesh> mesh;
		cable::Bounds local_bounds;
		bool has_local_bounds = false;
		bool parameters_valid = false;
		uint64_t last_frame = 0;
		bool moved_this_frame = true;
	};

	bool refresh_cached(const Ref<Shape3D> &p_shape, const Transform3D &p_transform, uint64_t p_key,
			bool p_is_static, uint32_t p_layer, CachedCollider &r_cached);
	bool extract_shape(const Ref<Shape3D> &p_shape, const Transform3D &p_transform, uint64_t p_key,
			bool p_is_static, uint32_t p_layer, CachedCollider &r_cached);
	void emit_warning_once(const String &p_message);

	std::unordered_map<uint64_t, CachedCollider> cache;
	HashSet<uint64_t> exceptions;
	HashSet<String> reported_warnings;
	PackedStringArray warnings;

	Ref<BoxShape3D> query_box;
	Ref<PhysicsShapeQueryParameters3D> query_params;

	uint32_t collision_mask = 0xFFFFFFFF;
	bool collide_with_bodies = true;
	bool collide_with_areas = false;
	uint64_t frame_counter = 0;
};

} // namespace godot
