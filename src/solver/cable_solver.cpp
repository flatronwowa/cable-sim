#include "solver/cable_solver.h"

#include "solver/constraints_bend.h"
#include "solver/constraints_stretch.h"
#include "solver/contacts.h"
#include "solver/self_collision.h"
#include "solver/spool.h"

namespace cable {

void CableSolver::rebuild(const Vec3 &p_from, const Vec3 &p_to) {
	state.allocate(params.particle_capacity());
	self_collision.reserve(state.capacity());

	const float segment = params.effective_segment_length();
	const float span = distance(p_from, p_to);

	// Стартовая размотка: прямой путь плюс небольшой провис. Если пролёт больше
	// cable_length, конфигурация невозможна — кабель остаётся максимально натянутым,
	// длина молча не увеличивается.
	float deployed = clampf(span * 1.02f + 2.0f * segment, segment, params.cable_length);

	int segments = static_cast<int>(deployed / segment + 0.5f);
	if (segments < 1) {
		segments = 1;
	}
	if (segments > state.capacity() - 1) {
		segments = state.capacity() - 1;
	}
	const float rest = deployed / static_cast<float>(segments);

	// Провис параболы с прогибом s и пролётом L имеет длину дуги примерно
	// L * (1 + 8/3 * (s/L)^2), отсюда s = L * sqrt(3/8 * (arc/L - 1)).
	float sag = 0.0f;
	if (span > EPS_LENGTH && deployed > span) {
		sag = span * std::sqrt(3.0f / 8.0f * (deployed / span - 1.0f));
	}

	Vec3 path = safe_normalize(p_to - p_from);
	if (length_squared(path) == 0.0f) {
		path = Vec3(1.0f, 0.0f, 0.0f);
	}
	Vec3 down = safe_normalize(params.gravity);
	if (length_squared(down) == 0.0f) {
		down = Vec3(0.0f, -1.0f, 0.0f);
	}
	Vec3 sag_direction = safe_normalize(down - path * dot(down, path));
	if (length_squared(sag_direction) == 0.0f) {
		sag_direction = any_perpendicular(path);
	}

	for (int i = 0; i <= segments; ++i) {
		const float t = static_cast<float>(i) / static_cast<float>(segments);
		const Vec3 point = lerp(p_from, p_to, t) + sag_direction * (sag * 4.0f * t * (1.0f - t));
		state.append(point, 1.0f);
	}
	for (int i = 0; i < segments; ++i) {
		state.rest_length[i] = rest;
	}

	state.flags[0] |= PARTICLE_PINNED;
	if (anchor_end.active) {
		state.flags[state.count - 1] |= PARTICLE_PINNED;
	}

	anchor_start.previous_position = p_from;
	anchor_start.position = p_from;
	anchor_start.previous_direction = anchor_start.direction;
	anchor_end.previous_position = p_to;
	anchor_end.position = p_to;
	anchor_end.previous_direction = anchor_end.direction;

	state.recompute_arc_lengths();
	state.distribute_masses(params);
	update_diagnostics(1.0f / 60.0f);
}

void CableSolver::prerelax(const WorldSnapshot &p_world) {
	if (!is_initialized()) {
		return;
	}
	const bool sleep_was_enabled = params.sleep_enabled;
	params.sleep_enabled = false;
	for (int i = 0; i < PRERELAX_STEPS; ++i) {
		step(1.0f / 60.0f, p_world);
	}
	for (int i = 0; i < state.count; ++i) {
		state.velocity[i] = Vec3();
		state.prev_position[i] = state.position[i];
		state.frame_position[i] = state.position[i];
		state.projection_delta[i] = Vec3();
	}
	anchor_start.velocity = Vec3();
	anchor_end.velocity = Vec3();
	anchor_start.previous_position = anchor_start.position;
	anchor_end.previous_position = anchor_end.position;
	params.sleep_enabled = sleep_was_enabled;
}

void CableSolver::apply_teleports() {
	// Смещение якоря больше порога — это телепорт: предыдущие позиции сбрасываются,
	// иначе разность позиций даст огромную скорость.
	if (anchor_start.teleported || distance(anchor_start.position, anchor_start.previous_position) > TELEPORT_DISTANCE) {
		anchor_start.previous_position = anchor_start.position;
		anchor_start.velocity = Vec3();
		state.position[0] = anchor_start.position;
		state.prev_position[0] = anchor_start.position;
		anchor_start.previous_direction = anchor_start.direction;
		anchor_start.teleported = false;
	}
	if (anchor_end.active &&
			(anchor_end.teleported || distance(anchor_end.position, anchor_end.previous_position) > TELEPORT_DISTANCE)) {
		const int last = state.count - 1;
		anchor_end.previous_position = anchor_end.position;
		anchor_end.velocity = Vec3();
		state.position[last] = anchor_end.position;
		state.prev_position[last] = anchor_end.position;
		anchor_end.previous_direction = anchor_end.direction;
		anchor_end.teleported = false;
	}

	if (distance(anchor_start.position, anchor_start.previous_position) > SLEEP_EPSILON) {
		state.wake_window(0);
	}
	if (anchor_end.active && distance(anchor_end.position, anchor_end.previous_position) > SLEEP_EPSILON) {
		state.wake_window(state.count - 1);
	}
}

void CableSolver::wake_on_world_motion(const WorldSnapshot &p_world) {
	for (int i = 0; i < state.count; ++i) {
		if (!state.is_asleep(i) || state.contacts[i].collider_key == 0) {
			continue;
		}
		const int shape = p_world.find_shape(state.contacts[i].collider_key);
		if (shape < 0 || p_world.get_shape(shape).moved_this_frame) {
			state.wake_window(i);
		}
	}
}

float CableSolver::max_particle_speed() const {
	float best = 0.0f;
	for (int i = 0; i < state.count; ++i) {
		if (state.is_asleep(i)) {
			continue;
		}
		best = maxf(best, length(state.velocity[i]));
	}
	return best;
}

float CableSolver::max_stretch_error() const {
	float best = 0.0f;
	for (int i = 0; i < state.segment_count(); ++i) {
		const float rest = state.rest_length[i];
		if (rest < EPS_LENGTH) {
			continue;
		}
		const float error = std::fabs(distance(state.position[i], state.position[i + 1]) / rest - 1.0f);
		best = maxf(best, error);
	}
	return best;
}

int CableSolver::compute_substeps(float p_dt) const {
	// За подшаг ни одна частица не должна проходить больше половины своей толщины:
	// это и есть основная защита от проскакивания.
	const float travel_budget = params.cable_radius * SUBSTEP_SAFETY;
	float speed = maxf(length(anchor_start.velocity), length(anchor_end.velocity));
	speed = maxf(speed, distance(anchor_start.position, anchor_start.previous_position) / maxf(p_dt, EPS_LENGTH));
	speed = maxf(speed, distance(anchor_end.position, anchor_end.previous_position) / maxf(p_dt, EPS_LENGTH));
	speed = maxf(speed, max_particle_speed());

	int substeps = params.substeps_min;
	if (travel_budget > EPS_LENGTH) {
		substeps = static_cast<int>(std::ceil(speed * p_dt / travel_budget));
	}
	if (substeps < params.substeps_min) {
		substeps = params.substeps_min;
	}
	if (substeps > params.substeps_max) {
		substeps = params.substeps_max;
	}
	return substeps < 1 ? 1 : substeps;
}

void CableSolver::begin_substep(float p_h) {
	const Vec3 gravity = params.gravity * params.gravity_scale;

	for (int i = 0; i < state.count; ++i) {
		state.prev_position[i] = state.position[i];
		state.projection_delta[i] = Vec3();
		state.contacts[i].lambda_normal = 0.0f; // импульс контакта копится за подшаг
		state.segment_contacts[i].clear();
		// lambda обнуляется на каждом подшаге: схема Small Steps делает одну
		// итерацию, накопление идёт внутри подшага.
		state.lambda_stretch[i] = 0.0f;
		state.lambda_bend[i] = 0.0f;

		if (state.is_pinned(i) || state.is_asleep(i)) {
			continue;
		}
		state.velocity[i] += gravity * p_h;
		state.position[i] += state.velocity[i] * p_h;
	}
	lambda_anchor_start = 0.0f;
	lambda_anchor_end = 0.0f;
}

void CableSolver::place_anchors(float p_alpha) {
	// Позиция якоря интерполируется по подшагам: иначе подшаги не защищают от
	// проскакивания при быстром движении игрока.
	state.position[0] = lerp(anchor_start.previous_position, anchor_start.position, p_alpha);
	state.velocity[0] = anchor_start.velocity;

	if (anchor_end.active) {
		const int last = state.count - 1;
		state.position[last] = lerp(anchor_end.previous_position, anchor_end.position, p_alpha);
		state.velocity[last] = anchor_end.velocity;
	}
}

void CableSolver::solve_anchor_orientation(float p_h, float p_alpha) {
	if (p_h * p_h < EPS_DENOM || state.count < 2) {
		return;
	}

	// Виртуальная частица перед концевой, вдоль оси якоря, с нулевой обратной массой.
	// Сустав (ghost, якорь, соседняя частица) с нулевой rest-кривизной решается тем же
	// constraint, что и обычный изгиб: отдельного механизма ориентации выхода из
	// constraint, что и обычный изгиб. Ghost и якорь неподвижны, поэтому двигается
	// только соседняя частица.
	Vec3 start_direction = safe_normalize(lerp(anchor_start.previous_direction, anchor_start.direction, p_alpha));
	if (length_squared(start_direction) > 0.0f) {
		if (dot(start_direction, state.position[1] - state.position[0]) < 0.0f) {
			start_direction = -start_direction;
		}
		const float arm = state.rest_length[0];
		Vec3 ghost = state.position[0] - start_direction * arm;
		Vec3 anchor = state.position[0];
		Vec3 neighbor = state.position[1];
		if (solve_anchor_bend(ghost, anchor, neighbor, 0.0f, 0.0f,
					state.solve_inv_mass(1), arm,
					params.anchor_stiffness_compliance, p_h, lambda_anchor_start)) {
			state.project(1, neighbor - state.position[1]);
		}
	}

	Vec3 end_direction = safe_normalize(lerp(anchor_end.previous_direction, anchor_end.direction, p_alpha));
	if (anchor_end.active && length_squared(end_direction) > 0.0f) {
		const int last = state.count - 1;
		if (dot(end_direction, state.position[last - 1] - state.position[last]) < 0.0f) {
			end_direction = -end_direction;
		}
		const float arm = state.rest_length[last - 1];
		Vec3 ghost = state.position[last] - end_direction * arm;
		Vec3 anchor = state.position[last];
		Vec3 neighbor = state.position[last - 1];
		if (solve_anchor_bend(ghost, anchor, neighbor, 0.0f, 0.0f,
					state.solve_inv_mass(last - 1), arm,
					params.anchor_stiffness_compliance, p_h, lambda_anchor_end)) {
			state.project(last - 1, neighbor - state.position[last - 1]);
		}
	}
}

void CableSolver::restore_velocities(float p_h) {
	const float inv_h = 1.0f / p_h;
	for (int i = 0; i < state.count; ++i) {
		if (state.is_pinned(i)) {
			continue; // скорость якоря задана извне и не выводится из позиций
		}
		if (state.is_asleep(i)) {
			state.velocity[i] = Vec3();
			continue;
		}
		state.velocity[i] = (state.position[i] - state.prev_position[i] - state.projection_delta[i]) * inv_h;
	}
}

// Вязкое трение между соседями по кабелю: гасит бегущие по длине волны и дрожание
// уложенной части. В отличие от глобального затухания скорости, не замедляет ни
// свободное падение, ни перенос кабеля игроком — обнуляется только относительное
// движение соседних частиц, суммарный импульс сохраняется.
void CableSolver::damp_internal_velocities(float p_h) {
	const float fraction = clampf(params.damping * p_h, 0.0f, 1.0f);
	if (fraction < EPS_DENOM) {
		return;
	}

	const int segments = state.segment_count();
	for (int i = 0; i < segments; ++i) {
		const int a = i;
		const int b = i + 1;
		if (state.is_asleep(a) && state.is_asleep(b)) {
			continue;
		}
		const float w_a = state.is_pinned(a) ? 0.0f : state.inv_mass[a];
		const float w_b = state.is_pinned(b) ? 0.0f : state.inv_mass[b];
		const float w_sum = w_a + w_b;
		if (w_sum < EPS_DENOM) {
			continue;
		}
		const Vec3 relative = state.velocity[b] - state.velocity[a];
		const Vec3 impulse = relative * (fraction / w_sum);
		state.velocity[a] += impulse * w_a;
		state.velocity[b] -= impulse * w_b;
	}
}

void CableSolver::update_sleep() {
	if (!params.sleep_enabled) {
		for (int i = 0; i < state.count; ++i) {
			state.flags[i] &= static_cast<uint8_t>(~PARTICLE_ASLEEP);
			state.sleep_counter[i] = 0;
		}
		return;
	}

	for (int i = 0; i < state.count; ++i) {
		if (state.is_pinned(i)) {
			state.wake(i); // концевые частицы не спят никогда
			continue;
		}
		const float displacement = distance(state.position[i], state.frame_position[i]);
		if (displacement > WAKE_DISPLACEMENT) {
			state.wake_window(i);
			continue;
		}
		if (displacement < SLEEP_EPSILON && (state.flags[i] & PARTICLE_IN_CONTACT) != 0) {
			if (state.sleep_counter[i] < 255) {
				state.sleep_counter[i] += 1;
			}
			if (state.sleep_counter[i] >= SLEEP_FRAMES) {
				state.flags[i] |= PARTICLE_ASLEEP;
			}
		} else {
			state.sleep_counter[i] = 0;
		}
	}
}

void CableSolver::update_diagnostics(float p_h) {
	diagnostics.awake_particles = 0;
	diagnostics.contact_count = 0;
	diagnostics.world_segment_contact_count = 0;
	diagnostics.self_contact_count = self_collision.contact_count();
	diagnostics.max_tension = 0.0f;
	diagnostics.tension_direction = Vec3();
	diagnostics.kinetic_energy = 0.0f;
	diagnostics.max_projection = 0.0f;

	for (int i = 0; i < state.count; ++i) {
		if (!state.is_asleep(i)) {
			diagnostics.awake_particles += 1;
		}
		if (state.contacts[i].collider_key != 0) {
			diagnostics.contact_count += 1;
		}
		if (state.inv_mass[i] > EPS_DENOM) {
			diagnostics.kinetic_energy +=
					0.5f * length_squared(state.velocity[i]) / state.inv_mass[i];
		}
		diagnostics.max_projection =
				maxf(diagnostics.max_projection, length(state.projection_delta[i]));
	}
	for (int i = 0; i < state.segment_count(); ++i) {
		if (state.segment_contacts[i].collider_key != 0) {
			diagnostics.world_segment_contact_count += 1;
		}
	}
	diagnostics.contact_count += diagnostics.world_segment_contact_count;
	diagnostics.contact_count += diagnostics.self_contact_count;

	// Натяжение берётся из уже посчитанной lambda натяжных constraints: f = lambda / h^2.
	const float h2 = maxf(p_h * p_h, EPS_DENOM);
	for (int i = 0; i < state.segment_count(); ++i) {
		diagnostics.max_tension = maxf(diagnostics.max_tension, std::fabs(state.lambda_stretch[i]) / h2);
	}

	if (state.count >= 2) {
		const int last = state.count - 1;
		diagnostics.tension_direction = safe_normalize(state.position[last - 1] - state.position[last]);
	}

	diagnostics.polyline_length = state.polyline_length();
	diagnostics.deployed_length = state.deployed_length();
	diagnostics.stowed_length = stowed_length(state, params);
	diagnostics.normalized_bending_energy = cable::normalized_bending_energy(state);
	diagnostics.max_stretch_error = max_stretch_error();
	diagnostics.length_exhausted = diagnostics.stowed_length <= EPS_LENGTH;
}

void CableSolver::step(float p_dt, const WorldSnapshot &p_world) {
	if (!is_initialized() || p_dt <= EPS_LENGTH) {
		return;
	}

	apply_teleports();

	// Катушка выдаёт кабель только вслед за удалением разъёма от последней опоры.
	const Vec3 end_target = anchor_end.active ? anchor_end.position : state.position[state.count - 1];
	const Vec3 previous_end_target = anchor_end.active ? anchor_end.previous_position : end_target;
	spool_update(state, params, end_target, previous_end_target);
	wake_on_world_motion(p_world);

	for (int i = 0; i < state.count; ++i) {
		state.frame_position[i] = state.position[i];
	}

	const float broadphase_slack = params.cable_radius + params.collision_margin +
			params.effective_segment_length() + max_particle_speed() * p_dt;
	broadphase.build(p_world, state, broadphase_slack);

	const int substeps = compute_substeps(p_dt);
	const float h = p_dt / static_cast<float>(substeps);
	diagnostics.substeps = substeps;
	diagnostics.solver_iterations = 0;

	const int iterations = params.solver_iterations < 1 ? 1 : params.solver_iterations;

	for (int s = 0; s < substeps; ++s) {
		begin_substep(h);
		const float substep_alpha = static_cast<float>(s + 1) / static_cast<float>(substeps);
		place_anchors(substep_alpha);

		for (int it = 0; it < iterations; ++it) {
			solve_min_bend_radius(state, params);
			for (int internal = 0; internal < BEND_STRETCH_ITERATIONS; ++internal) {
				solve_anchor_orientation(h, substep_alpha);
				solve_bend(state, params, h);
				// Изгиб обменивает угол на длину сегмента, поэтому каждый локальный
				// проход заканчивается сохранением длины (приоритет 2 против 5).
				solve_stretch(state, params, h);
			}
			solve_lra(state);

			// Коллизии с миром — приоритет 1, поэтому они замыкают итерацию: иначе
			// самоконтакт в куче кабеля вдавливает частицы в пол.
			const float self_penetration = params.self_collision_enabled
					? self_collision.solve_positions(state, params)
					: 0.0f;
			solve_world_contacts(state, params, p_world, broadphase);
			diagnostics.solver_iterations = diagnostics.solver_iterations > it + 1
					? diagnostics.solver_iterations
					: it + 1;
			if (max_stretch_error() <= SOLVER_STRETCH_TOLERANCE &&
					self_penetration <= params.collision_margin) {
				break;
			}
		}

		restore_velocities(h);
		damp_internal_velocities(h);
		if (params.self_collision_enabled) {
			self_collision.solve_velocities(state, params, h);
		}
		world_contacts_velocity_pass(state, params, h);
	}

	anchor_start.previous_position = anchor_start.position;
	anchor_start.previous_direction = anchor_start.direction;
	anchor_end.previous_position = anchor_end.position;
	anchor_end.previous_direction = anchor_end.direction;

	update_sleep();
	update_diagnostics(h);
}

Vec3 CableSolver::constrain_player_motion(const Vec3 &p_from, const Vec3 &p_to, bool &r_limited) const {
	return constrain_motion(state, params, p_from, p_to, r_limited);
}

} // namespace cable
