#pragma once

#include "solver/collision_shapes.h"

#include <memory>
#include <vector>

namespace tmd {
class TriangleMeshDistance;
}

namespace cable {

// Обёртка над вендоренной TriangleMeshDistance. Знак расстояния определён только
// для водонепроницаемой сетки с согласованной ориентацией, поэтому замкнутость
// проверяется при построении, а для незамкнутой используется беззнаковое
// расстояние со стороной по предыдущей позиции частицы.
class CollisionMesh {
public:
	CollisionMesh();
	~CollisionMesh();
	CollisionMesh(const CollisionMesh &) = delete;
	CollisionMesh &operator=(const CollisionMesh &) = delete;

	// Треугольники подряд: по три вершины на треугольник, как отдаёт
	// ConcavePolygonShape3D::get_faces(). Вершины сшиваются по позиции, иначе
	// проверка замкнутости не имеет смысла.
	void build(const Vec3 *p_faces, int p_triangle_count);

	bool is_valid() const { return valid; }
	bool is_watertight() const { return watertight; }
	const Bounds &get_local_bounds() const { return local_bounds; }

	// p_reference — предыдущая позиция частицы в том же локальном пространстве;
	// для незамкнутой сетки она задаёт сторону поверхности.
	ShapeSample sample(const Vec3 &p_local, const Vec3 &p_reference) const;

private:
	Vec3 face_normal_or_zero(int p_triangle_id) const;

	std::unique_ptr<tmd::TriangleMeshDistance> distance;
	std::vector<Vec3> face_normals;
	bool valid = false;
	bool watertight = false;
	Bounds local_bounds;
};

} // namespace cable
