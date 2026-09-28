#pragma once

#include "solver/solver_math.h"

namespace cable {

enum class ShapeType : uint8_t {
	SPHERE,
	BOX,
	CAPSULE,
	CYLINDER,
	CONVEX,
	PLANE,
	MESH,
};

// Единый результат опроса геометрии: знаковое расстояние и направление наружу.
// Диспетчеризация по типу формы существует только внутри этого интерфейса.
struct ShapeSample {
	float distance = 1.0e30f;
	Vec3 gradient; // единичный, наружу
	bool valid = false;
};

struct ConvexPlane {
	Vec3 normal;
	float d = 0.0f;
};

struct ShapeData {
	ShapeType type = ShapeType::SPHERE;
	uint64_t key = 0; // стабильный идентификатор коллайдера, на него ссылается якорь контакта
	uint32_t layer = 0;
	bool is_static = false;
	bool moved_this_frame = true; // для пробуждения частиц на движущемся коллайдере

	Xform transform; // basis ортонормирован, масштаб запечён в параметры формы
	Vec3 half_extents; // BOX
	float radius = 0.0f; // SPHERE / CAPSULE / CYLINDER
	float height = 0.0f; // полная высота CAPSULE / CYLINDER, как в Godot

	Vec3 plane_normal; // PLANE, world space
	float plane_d = 0.0f;

	uint32_t convex_plane_first = 0; // CONVEX, индексы в общий массив плоскостей
	uint32_t convex_plane_count = 0;

	int32_t mesh_index = -1; // MESH

	Bounds world_aabb;
};

// Опрос всех типов, кроме MESH: треугольные сетки требуют BVH и опрашиваются
// через world_snapshot.
ShapeSample sample_primitive(const ShapeData &p_shape, const ConvexPlane *p_planes, const Vec3 &p_point);

} // namespace cable
