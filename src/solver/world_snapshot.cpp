#include "solver/world_snapshot.h"

namespace cable {

void WorldSnapshot::begin_frame() {
	shapes.clear();
	convex_planes.clear();
	meshes.clear();
}

int WorldSnapshot::add_shape(const ShapeData &p_shape) {
	shapes.push_back(p_shape);
	return static_cast<int>(shapes.size()) - 1;
}

uint32_t WorldSnapshot::add_convex_planes(const ConvexPlane *p_planes, uint32_t p_count) {
	const uint32_t first = static_cast<uint32_t>(convex_planes.size());
	for (uint32_t i = 0; i < p_count; ++i) {
		convex_planes.push_back(p_planes[i]);
	}
	return first;
}

int WorldSnapshot::add_mesh(const CollisionMesh *p_mesh) {
	meshes.push_back(p_mesh);
	return static_cast<int>(meshes.size()) - 1;
}

int WorldSnapshot::find_shape(uint64_t p_key) const {
	for (int i = 0; i < static_cast<int>(shapes.size()); ++i) {
		if (shapes[i].key == p_key) {
			return i;
		}
	}
	return -1;
}

Vec3 WorldSnapshot::to_local(int p_shape_index, const Vec3 &p_point) const {
	return xform_inv_point(shapes[p_shape_index].transform, p_point);
}

Vec3 WorldSnapshot::to_world(int p_shape_index, const Vec3 &p_local) const {
	return xform_point(shapes[p_shape_index].transform, p_local);
}

ShapeSample WorldSnapshot::sample(int p_shape_index, const Vec3 &p_point, const Vec3 &p_reference) const {
	const ShapeData &shape = shapes[p_shape_index];
	if (shape.type != ShapeType::MESH) {
		return sample_primitive(shape, convex_planes.data(), p_point);
	}

	ShapeSample out;
	if (shape.mesh_index < 0 || shape.mesh_index >= static_cast<int32_t>(meshes.size())) {
		return out;
	}
	const CollisionMesh *mesh = meshes[shape.mesh_index];
	if (mesh == nullptr || !mesh->is_valid()) {
		return out;
	}
	out = mesh->sample(xform_inv_point(shape.transform, p_point), xform_inv_point(shape.transform, p_reference));
	if (out.valid) {
		out.gradient = xform_dir(shape.transform, out.gradient);
	}
	return out;
}

void Broadphase::build(const WorldSnapshot &p_world, const CableState &p_state, float p_slack) {
	const int count = p_state.count;
	offsets.assign(static_cast<size_t>(count) + 1, 0u);
	indices.clear();
	indices.reserve(static_cast<size_t>(count) * 4);

	const int shape_count = p_world.shape_count();
	for (int i = 0; i < count; ++i) {
		offsets[i] = static_cast<uint32_t>(indices.size());
		if (shape_count == 0) {
			continue;
		}

		Bounds bounds;
		bounds.expand(p_state.position[i]);
		if (i > 0) {
			bounds.expand(p_state.position[i - 1]);
		}
		if (i + 1 < count) {
			bounds.expand(p_state.position[i + 1]);
		}
		bounds.grow(p_slack);

		int added = 0;
		for (int s = 0; s < shape_count && added < MAX_CANDIDATE_SHAPES; ++s) {
			const ShapeData &shape = p_world.get_shape(s);
			// Полупространство не имеет конечного AABB и всегда остаётся кандидатом.
			if (shape.type != ShapeType::PLANE && !shape.world_aabb.overlaps(bounds)) {
				continue;
			}
			indices.push_back(static_cast<uint16_t>(s));
			++added;
		}
	}
	offsets[count] = static_cast<uint32_t>(indices.size());
}

} // namespace cable
