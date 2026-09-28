#pragma once

#include "solver/cable_solver.h"

#include <godot_cpp/classes/immediate_mesh.hpp>
#include <godot_cpp/classes/standard_material3d.hpp>
#include <godot_cpp/variant/transform3d.hpp>

namespace godot {

// Без отладочной отрисовки солвер не отлаживается: это условие возможности работы,
// а не удобство. В релизной сборке весь класс компилируется в пустые вызовы.
class CableDebugDraw {
public:
	Ref<ImmediateMesh> create_mesh();
	void update(const Ref<ImmediateMesh> &p_mesh, const cable::CableSolver &p_solver,
			const cable::WorldSnapshot &p_world, const Transform3D &p_world_to_local);

private:
	Ref<StandardMaterial3D> material;
};

} // namespace godot
