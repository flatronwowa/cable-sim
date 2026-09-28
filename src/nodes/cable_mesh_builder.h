#pragma once

#include "solver/solver_math.h"

#include <godot_cpp/classes/array_mesh.hpp>
#include <godot_cpp/variant/packed_byte_array.hpp>
#include <godot_cpp/variant/transform3d.hpp>

#include <vector>

namespace godot {

// Трубчатый меш по центральной линии. Топология и индексы постоянны, каждый кадр
// обновляются только позиции и нормали через surface_update_vertex_region: иначе в
// каждом кадре выделялись бы PackedArray.
class CableMeshBuilder {
public:
	void configure(int p_radial_segments, int p_max_rings);
	bool is_configured() const { return configured; }

	Ref<ArrayMesh> create_mesh() const;

	// Точки передаются в world space, преобразование в local space выполняется здесь —
	// это единственная граница, где пространства смешиваются.
	void update(const Ref<ArrayMesh> &p_mesh, const cable::Vec3 *p_points, int p_count,
			const Transform3D &p_world_to_local, float p_radius);

	// UV вдоль кабеля считается по дуговой длине, иначе текстура едет при изменении
	// провиса и при размотке. Обновляется только при изменении размотанной длины.
	void update_uv(const Ref<ArrayMesh> &p_mesh, const float *p_arc_from_start, int p_count);

	const AABB &get_local_aabb() const { return local_aabb; }

private:
	void ensure_offsets(const Ref<ArrayMesh> &p_mesh);
	void build_frames(const cable::Vec3 *p_points, int p_count);

	int radial_segments = 8;
	int max_rings = 0;
	int ring_stride = 0; // radial_segments + 1, шов дублируется ради UV
	bool configured = false;

	uint32_t surface_format = 0;
	uint32_t vertex_offset = 0;
	uint32_t vertex_stride = 0;
	uint32_t normal_offset = 0;
	uint32_t normal_stride = 0;
	uint32_t uv_offset = 0;
	uint32_t attribute_stride = 0;
	bool offsets_ready = false;

	std::vector<cable::Vec3> tangents;
	std::vector<cable::Vec3> frame_right;
	std::vector<cable::Vec3> frame_up;
	std::vector<float> ring_cos;
	std::vector<float> ring_sin;

	PackedByteArray position_bytes;
	PackedByteArray normal_bytes;
	PackedByteArray uv_bytes;

	AABB local_aabb;
};

} // namespace godot
