#pragma once

#include "solver/cable_state.h"
#include "solver/collision_mesh.h"
#include "solver/collision_shapes.h"

#include <vector>

namespace cable {

// Плоское представление геометрии мира. Заполняется слоем нод раз в кадр; солвер
// не обращается к физике движка. Сами объекты CollisionMesh живут в кеше строителя,
// здесь только указатели.
class WorldSnapshot {
public:
	void begin_frame();

	int add_shape(const ShapeData &p_shape);
	uint32_t add_convex_planes(const ConvexPlane *p_planes, uint32_t p_count);
	int add_mesh(const CollisionMesh *p_mesh);

	int shape_count() const { return static_cast<int>(shapes.size()); }
	const ShapeData &get_shape(int p_index) const { return shapes[p_index]; }
	int find_shape(uint64_t p_key) const;

	// Единый опрос геометрии. p_reference — предыдущая позиция частицы, нужна для
	// определения стороны у незамкнутых сеток.
	ShapeSample sample(int p_shape_index, const Vec3 &p_point, const Vec3 &p_reference) const;

	Vec3 to_local(int p_shape_index, const Vec3 &p_point) const;
	Vec3 to_world(int p_shape_index, const Vec3 &p_local) const;

private:
	std::vector<ShapeData> shapes;
	std::vector<ConvexPlane> convex_planes;
	std::vector<const CollisionMesh *> meshes;
};

// Широкая фаза: список форм-кандидатов на частицу. Строится раз в кадр, а не раз в
// подшаг — за кадр геометрия не меняется, а перемещение ограничено радиусом.
// Список частицы i покрывает и сегмент i, потому что строится по границам
// (x[i-1], x[i], x[i+1]).
class Broadphase {
public:
	void build(const WorldSnapshot &p_world, const CableState &p_state, float p_slack);

	int begin(int p_particle) const { return static_cast<int>(offsets[p_particle]); }
	int end(int p_particle) const { return static_cast<int>(offsets[p_particle + 1]); }
	int shape_at(int p_slot) const { return static_cast<int>(indices[p_slot]); }

private:
	std::vector<uint32_t> offsets;
	std::vector<uint16_t> indices;
};

} // namespace cable
