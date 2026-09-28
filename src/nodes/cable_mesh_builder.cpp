#include "nodes/cable_mesh_builder.h"

#include <godot_cpp/classes/rendering_server.hpp>
#include <godot_cpp/core/math.hpp>
#include <godot_cpp/variant/packed_int32_array.hpp>
#include <godot_cpp/variant/packed_vector2_array.hpp>
#include <godot_cpp/variant/packed_vector3_array.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

#include <cstring>

using namespace godot;
using cable::Vec3;

namespace {

Vector3 to_godot(const Vec3 &v) {
	return Vector3(v.x, v.y, v.z);
}

// Уплотнение нормали в формат вершинного буфера Godot: октаэдральная карта в два
// uint16.
uint32_t encode_normal(const Vector3 &p_normal) {
	const Vector2 encoded = p_normal.octahedron_encode();
	uint32_t value = static_cast<uint16_t>(CLAMP(encoded.x * 65535.0f, 0.0f, 65535.0f));
	value |= static_cast<uint32_t>(static_cast<uint16_t>(CLAMP(encoded.y * 65535.0f, 0.0f, 65535.0f))) << 16;
	return value;
}

} // namespace

void CableMeshBuilder::configure(int p_radial_segments, int p_max_rings) {
	radial_segments = p_radial_segments < 3 ? 3 : p_radial_segments;
	max_rings = p_max_rings < 2 ? 2 : p_max_rings;
	ring_stride = radial_segments + 1; // шов дублируется, чтобы UV не заворачивалась
	offsets_ready = false;

	ring_cos.resize(static_cast<size_t>(ring_stride));
	ring_sin.resize(static_cast<size_t>(ring_stride));
	for (int k = 0; k < ring_stride; ++k) {
		const float angle = 2.0f * cable::PI_F * static_cast<float>(k) / static_cast<float>(radial_segments);
		ring_cos[k] = std::cos(angle);
		ring_sin[k] = std::sin(angle);
	}
	configured = true;
}

Ref<ArrayMesh> CableMeshBuilder::create_mesh() const {
	const int vertex_count = max_rings * ring_stride;

	PackedVector3Array vertices;
	PackedVector3Array normals;
	PackedVector2Array uvs;
	vertices.resize(vertex_count);
	normals.resize(vertex_count);
	uvs.resize(vertex_count);
	for (int i = 0; i < vertex_count; ++i) {
		normals[i] = Vector3(0.0f, 1.0f, 0.0f);
	}

	PackedInt32Array indices;
	indices.resize((max_rings - 1) * radial_segments * 6);
	int cursor = 0;
	for (int ring = 0; ring < max_rings - 1; ++ring) {
		for (int k = 0; k < radial_segments; ++k) {
			const int a = ring * ring_stride + k;
			const int b = a + 1;
			const int c = (ring + 1) * ring_stride + k;
			const int d = c + 1;
			// Godot ожидает обход по часовой стрелке для лицевых треугольников.
			indices[cursor++] = a;
			indices[cursor++] = b;
			indices[cursor++] = d;
			indices[cursor++] = a;
			indices[cursor++] = d;
			indices[cursor++] = c;
		}
	}

	Array arrays;
	arrays.resize(Mesh::ARRAY_MAX);
	arrays[Mesh::ARRAY_VERTEX] = vertices;
	arrays[Mesh::ARRAY_NORMAL] = normals;
	arrays[Mesh::ARRAY_TEX_UV] = uvs;
	arrays[Mesh::ARRAY_INDEX] = indices;

	Ref<ArrayMesh> mesh;
	mesh.instantiate();
	mesh->add_surface_from_arrays(Mesh::PRIMITIVE_TRIANGLES, arrays, TypedArray<Array>(), Dictionary(),
			Mesh::ARRAY_FLAG_USE_DYNAMIC_UPDATE);
	return mesh;
}

void CableMeshBuilder::ensure_offsets(const Ref<ArrayMesh> &p_mesh) {
	if (offsets_ready || p_mesh.is_null() || p_mesh->get_surface_count() == 0) {
		return;
	}
	const int vertex_count = max_rings * ring_stride;
	RenderingServer *server = RenderingServer::get_singleton();
	surface_format = static_cast<uint32_t>(static_cast<int64_t>(p_mesh->surface_get_format(0)));

	// Раскладка вершинного буфера берётся у RenderingServer, а не зашивается числами.
	vertex_offset = server->mesh_surface_get_format_offset(surface_format, vertex_count, Mesh::ARRAY_VERTEX);
	vertex_stride = server->mesh_surface_get_format_vertex_stride(surface_format, vertex_count);
	normal_offset = server->mesh_surface_get_format_offset(surface_format, vertex_count, Mesh::ARRAY_NORMAL);
	normal_stride = server->mesh_surface_get_format_normal_tangent_stride(surface_format, vertex_count);
	uv_offset = server->mesh_surface_get_format_offset(surface_format, vertex_count, Mesh::ARRAY_TEX_UV);
	attribute_stride = server->mesh_surface_get_format_attribute_stride(surface_format, vertex_count);

	if (vertex_stride < sizeof(float) * 3 || normal_stride < sizeof(uint32_t) || attribute_stride < sizeof(float) * 2) {
		UtilityFunctions::push_error("CableBody3D: неожиданная раскладка вершинного буфера, обновление меша отключено.");
		return;
	}

	position_bytes.resize(static_cast<int64_t>(vertex_count) * vertex_stride);
	normal_bytes.resize(static_cast<int64_t>(vertex_count) * normal_stride);
	uv_bytes.resize(static_cast<int64_t>(vertex_count) * attribute_stride);
	offsets_ready = true;
}

void CableMeshBuilder::build_frames(const Vec3 *p_points, int p_count) {
	tangents.resize(static_cast<size_t>(p_count));
	frame_right.resize(static_cast<size_t>(p_count));
	frame_up.resize(static_cast<size_t>(p_count));

	for (int i = 0; i < p_count; ++i) {
		Vec3 tangent;
		if (p_count < 2) {
			tangent = Vec3(0.0f, 0.0f, 1.0f);
		} else if (i == 0) {
			tangent = p_points[1] - p_points[0];
		} else if (i == p_count - 1) {
			tangent = p_points[p_count - 1] - p_points[p_count - 2];
		} else {
			tangent = p_points[i + 1] - p_points[i - 1];
		}
		tangent = cable::safe_normalize(tangent);
		if (cable::length_squared(tangent) == 0.0f) {
			tangent = i > 0 ? tangents[i - 1] : Vec3(0.0f, 0.0f, 1.0f);
		}
		tangents[i] = tangent;
	}

	frame_right[0] = cable::any_perpendicular(tangents[0]);
	frame_up[0] = cable::cross(tangents[0], frame_right[0]);

	// Rotation-minimizing frame методом двойного отражения (Wang et al., 2008):
	// устойчив, не требует глобального опорного вектора и не даёт перекрута.
	for (int i = 1; i < p_count; ++i) {
		const Vec3 v1 = p_points[i] - p_points[i - 1];
		const float c1 = cable::length_squared(v1);
		Vec3 reflected_right = frame_right[i - 1];
		Vec3 reflected_tangent = tangents[i - 1];
		if (c1 > cable::EPS_DENOM) {
			reflected_right -= v1 * (2.0f / c1 * cable::dot(v1, reflected_right));
			reflected_tangent -= v1 * (2.0f / c1 * cable::dot(v1, reflected_tangent));
		}

		const Vec3 v2 = tangents[i] - reflected_tangent;
		const float c2 = cable::length_squared(v2);
		Vec3 right = reflected_right;
		if (c2 > cable::EPS_DENOM) {
			right -= v2 * (2.0f / c2 * cable::dot(v2, reflected_right));
		}

		right = cable::safe_normalize(right - tangents[i] * cable::dot(tangents[i], right));
		if (cable::length_squared(right) == 0.0f) {
			right = cable::any_perpendicular(tangents[i]);
		}
		frame_right[i] = right;
		frame_up[i] = cable::cross(tangents[i], right);
	}
}

void CableMeshBuilder::update(const Ref<ArrayMesh> &p_mesh, const Vec3 *p_points, int p_count,
		const Transform3D &p_world_to_local, float p_radius) {
	if (!configured || p_count < 2) {
		return;
	}
	ensure_offsets(p_mesh);
	if (!offsets_ready) {
		return;
	}

	const int rings = p_count < max_rings ? p_count : max_rings;
	build_frames(p_points, rings);

	uint8_t *position_data = position_bytes.ptrw();
	uint8_t *normal_data = normal_bytes.ptrw();
	local_aabb = AABB();
	bool aabb_started = false;

	for (int ring = 0; ring < max_rings; ++ring) {
		// Неиспользуемый хвост буфера схлопывается в вырожденные треугольники, а не
		// приводит к пересозданию поверхности.
		const int source = ring < rings ? ring : rings - 1;
		const Vector3 center = p_world_to_local.xform(to_godot(p_points[source]));
		const Vector3 right = p_world_to_local.basis.xform(to_godot(frame_right[source]));
		const Vector3 up = p_world_to_local.basis.xform(to_godot(frame_up[source]));
		const bool degenerate = ring >= rings;

		for (int k = 0; k < ring_stride; ++k) {
			const Vector3 normal = right * ring_cos[k] + up * ring_sin[k];
			const Vector3 point = degenerate ? center : center + normal * p_radius;
			const int index = ring * ring_stride + k;

			const float position[3] = { static_cast<float>(point.x), static_cast<float>(point.y), static_cast<float>(point.z) };
			memcpy(position_data + static_cast<size_t>(index) * vertex_stride, position, sizeof(position));

			const uint32_t packed = encode_normal(normal.normalized());
			memcpy(normal_data + static_cast<size_t>(index) * normal_stride, &packed, sizeof(packed));

			if (!degenerate) {
				if (aabb_started) {
					local_aabb = local_aabb.expand(point);
				} else {
					local_aabb = AABB(point, Vector3());
					aabb_started = true;
				}
			}
		}
	}

	p_mesh->surface_update_vertex_region(0, static_cast<int32_t>(vertex_offset), position_bytes);
	p_mesh->surface_update_vertex_region(0, static_cast<int32_t>(normal_offset), normal_bytes);
}

void CableMeshBuilder::update_uv(const Ref<ArrayMesh> &p_mesh, const float *p_arc_from_start, int p_count) {
	if (!configured || p_count < 2) {
		return;
	}
	ensure_offsets(p_mesh);
	if (!offsets_ready) {
		return;
	}

	const int rings = p_count < max_rings ? p_count : max_rings;
	uint8_t *uv_data = uv_bytes.ptrw();

	for (int ring = 0; ring < max_rings; ++ring) {
		const int source = ring < rings ? ring : rings - 1;
		const float u = p_arc_from_start[source];
		for (int k = 0; k < ring_stride; ++k) {
			const float uv[2] = { u, static_cast<float>(k) / static_cast<float>(radial_segments) };
			const int index = ring * ring_stride + k;
			memcpy(uv_data + static_cast<size_t>(index) * attribute_stride, uv, sizeof(uv));
		}
	}

	p_mesh->surface_update_attribute_region(0, static_cast<int32_t>(uv_offset), uv_bytes);
}
