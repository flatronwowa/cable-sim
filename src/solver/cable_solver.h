#pragma once

#include "solver/cable_params.h"
#include "solver/cable_state.h"
#include "solver/self_collision.h"
#include "solver/world_snapshot.h"

namespace cable {

// Вход якоря. Скорость передаётся реальная, а не вычисленная из разности позиций:
// иначе телепорт якоря даёт огромную скорость и взрыв солвера.
struct AnchorInput {
	bool active = false;
	Vec3 position;
	Vec3 previous_position;
	Vec3 velocity;
	Vec3 direction; // ось выхода из разъёма, наружу
	Vec3 previous_direction;
	bool teleported = false;
};

struct SolverDiagnostics {
	int substeps = 0;
	int solver_iterations = 0;
	int awake_particles = 0;
	int contact_count = 0;
	int world_segment_contact_count = 0;
	int self_contact_count = 0;
	float max_tension = 0.0f;
	Vec3 tension_direction;
	float polyline_length = 0.0f;
	float deployed_length = 0.0f;
	float stowed_length = 0.0f;
	float kinetic_energy = 0.0f;
	float normalized_bending_energy = 0.0f;
	float max_stretch_error = 0.0f;
	float max_projection = 0.0f;
	bool length_exhausted = false;
};

class CableSolver {
public:
	CableParams params;
	CableState state;
	AnchorInput anchor_start;
	AnchorInput anchor_end;

	// Раскладывает частицы по начальному пути, избыточную длину распределяет как
	// провис перпендикулярно пути: набивание лишних частиц вдоль линии взрывает
	// солвер на первом же шаге.
	void rebuild(const Vec3 &p_from, const Vec3 &p_to);

	// Несколько десятков шагов до первого кадра, иначе при загрузке уровня кабель
	// оседает на глазах у игрока.
	void prerelax(const WorldSnapshot &p_world);

	void step(float p_dt, const WorldSnapshot &p_world);

	Vec3 constrain_player_motion(const Vec3 &p_from, const Vec3 &p_to, bool &r_limited) const;

	const SolverDiagnostics &get_diagnostics() const { return diagnostics; }
	bool is_initialized() const { return state.count >= 2; }

private:
	void apply_teleports();
	void wake_on_world_motion(const WorldSnapshot &p_world);
	int compute_substeps(float p_dt) const;
	void begin_substep(float p_h);
	void place_anchors(float p_alpha);
	void solve_anchor_orientation(float p_h, float p_alpha);
	void restore_velocities(float p_h);
	void damp_internal_velocities(float p_h);
	void update_sleep();
	void update_diagnostics(float p_h);
	float max_particle_speed() const;
	float max_stretch_error() const;

	SelfCollisionSolver self_collision;
	Broadphase broadphase;
	SolverDiagnostics diagnostics;

	// lambda суставов с виртуальными частицами якорей.
	float lambda_anchor_start = 0.0f;
	float lambda_anchor_end = 0.0f;
};

} // namespace cable
