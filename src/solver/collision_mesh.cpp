#include "solver/collision_mesh.h"

#include "solver/third_party/triangle_mesh_distance.h"

#include <array>
#include <cstring>
#include <unordered_map>

namespace cable {

namespace {

struct VertexKey {
	uint32_t x = 0;
	uint32_t y = 0;
	uint32_t z = 0;

	bool operator==(const VertexKey &o) const { return x == o.x && y == o.y && z == o.z; }
};

struct VertexKeyHash {
	size_t operator()(const VertexKey &k) const {
		size_t h = k.x * 73856093u;
		h ^= k.y * 19349663u;
		h ^= k.z * 83492791u;
		return h;
	}
};

// Вершины, пришедшие из одного меша, побитово совпадают, поэтому сшивание точное:
// квантование внесло бы ошибки на границах ячеек.
VertexKey make_key(const Vec3 &v) {
	VertexKey k;
	std::memcpy(&k.x, &v.x, sizeof(uint32_t));
	std::memcpy(&k.y, &v.y, sizeof(uint32_t));
	std::memcpy(&k.z, &v.z, sizeof(uint32_t));
	return k;
}

struct EdgeKey {
	int32_t a = 0;
	int32_t b = 0;

	bool operator==(const EdgeKey &o) const { return a == o.a && b == o.b; }
};

struct EdgeKeyHash {
	size_t operator()(const EdgeKey &k) const {
		return static_cast<size_t>(k.a) * 73856093u ^ static_cast<size_t>(k.b) * 19349663u;
	}
};

struct EdgeInfo {
	int count = 0;
	int orientation_sum = 0;
};

} // namespace

CollisionMesh::CollisionMesh() = default;
CollisionMesh::~CollisionMesh() = default;

void CollisionMesh::build(const Vec3 *p_faces, int p_triangle_count) {
	valid = false;
	watertight = false;
	local_bounds.reset();
	distance.reset();
	face_normals.clear();

	if (p_faces == nullptr || p_triangle_count <= 0) {
		return;
	}

	std::vector<std::array<double, 3>> vertices;
	std::vector<std::array<int, 3>> triangles;
	vertices.reserve(static_cast<size_t>(p_triangle_count) * 3);
	triangles.reserve(static_cast<size_t>(p_triangle_count));
	face_normals.reserve(static_cast<size_t>(p_triangle_count));

	std::unordered_map<VertexKey, int, VertexKeyHash> welded;
	welded.reserve(static_cast<size_t>(p_triangle_count) * 2);

	std::unordered_map<EdgeKey, EdgeInfo, EdgeKeyHash> edges;
	edges.reserve(static_cast<size_t>(p_triangle_count) * 3);

	for (int t = 0; t < p_triangle_count; ++t) {
		const Vec3 v[3] = { p_faces[t * 3 + 0], p_faces[t * 3 + 1], p_faces[t * 3 + 2] };
		const Vec3 normal = cross(v[1] - v[0], v[2] - v[0]);
		if (length(normal) < EPS_LENGTH) {
			continue; // вырожденный треугольник не несёт поверхности
		}

		int index[3];
		for (int c = 0; c < 3; ++c) {
			const VertexKey key = make_key(v[c]);
			auto it = welded.find(key);
			if (it == welded.end()) {
				index[c] = static_cast<int>(vertices.size());
				welded.emplace(key, index[c]);
				vertices.push_back({ static_cast<double>(v[c].x), static_cast<double>(v[c].y), static_cast<double>(v[c].z) });
			} else {
				index[c] = it->second;
			}
			local_bounds.expand(v[c]);
		}
		if (index[0] == index[1] || index[1] == index[2] || index[0] == index[2]) {
			continue;
		}

		triangles.push_back({ index[0], index[1], index[2] });
		face_normals.push_back(safe_normalize(normal));

		for (int c = 0; c < 3; ++c) {
			const int a = index[c];
			const int b = index[(c + 1) % 3];
			EdgeKey key;
			key.a = a < b ? a : b;
			key.b = a < b ? b : a;
			EdgeInfo &info = edges[key];
			info.count += 1;
			info.orientation_sum += (a < b) ? 1 : -1;
		}
	}

	if (triangles.empty()) {
		return;
	}

	// Замкнутость: каждое ребро принадлежит ровно двум треугольникам, и они
	// обходят его в противоположных направлениях.
	watertight = true;
	for (const auto &entry : edges) {
		if (entry.second.count != 2 || entry.second.orientation_sum != 0) {
			watertight = false;
			break;
		}
	}

	distance = std::make_unique<tmd::TriangleMeshDistance>();
	distance->construct(vertices, triangles);
	valid = true;
}

ShapeSample CollisionMesh::sample(const Vec3 &p_local, const Vec3 &p_reference) const {
	ShapeSample out;
	if (!valid) {
		return out;
	}

	const std::array<double, 3> query = { p_local.x, p_local.y, p_local.z };

	if (watertight) {
		const tmd::Result r = distance->signed_distance(query);
		const Vec3 nearest(static_cast<float>(r.nearest_point[0]), static_cast<float>(r.nearest_point[1]), static_cast<float>(r.nearest_point[2]));
		out.distance = static_cast<float>(r.distance);
		// Для отрицательного расстояния вектор от ближайшей точки направлен внутрь.
		Vec3 g = safe_normalize(p_local - nearest);
		if (length_squared(g) == 0.0f) {
			g = face_normal_or_zero(r.triangle_id);
		} else if (out.distance < 0.0f) {
			g = -g;
		}
		out.gradient = g;
		out.valid = length_squared(g) > 0.0f;
		return out;
	}

	// Незамкнутая сетка: знак неопределён, сторону задаёт предыдущая позиция частицы.
	const tmd::Result r = distance->unsigned_distance(query);
	const Vec3 nearest(static_cast<float>(r.nearest_point[0]), static_cast<float>(r.nearest_point[1]), static_cast<float>(r.nearest_point[2]));
	const Vec3 face_normal = face_normal_or_zero(r.triangle_id);
	if (length_squared(face_normal) == 0.0f) {
		return out;
	}

	const float offset_now = dot(p_local - nearest, face_normal);
	const float offset_ref = dot(p_reference - nearest, face_normal);
	const float side = (std::fabs(offset_ref) > EPS_NORMAL)
			? (offset_ref > 0.0f ? 1.0f : -1.0f)
			: (offset_now >= 0.0f ? 1.0f : -1.0f);

	const bool on_expected_side = offset_now * side >= 0.0f;
	out.distance = on_expected_side ? static_cast<float>(r.distance) : -static_cast<float>(r.distance);
	Vec3 g = on_expected_side ? safe_normalize(p_local - nearest) : face_normal * side;
	if (length_squared(g) == 0.0f) {
		g = face_normal * side;
	}
	out.gradient = g;
	out.valid = true;
	return out;
}

Vec3 CollisionMesh::face_normal_or_zero(int p_triangle_id) const {
	if (p_triangle_id < 0 || p_triangle_id >= static_cast<int>(face_normals.size())) {
		return Vec3();
	}
	return face_normals[p_triangle_id];
}

} // namespace cable
