#include "solver/constraints_stretch.h"

namespace cable {

void solve_stretch(CableState &r_state, const CableParams &p_params, float p_h) {
	const float h2 = p_h * p_h;
	if (h2 < EPS_DENOM) {
		return;
	}
	// alpha_tilde = alpha / h^2: так жёсткость не зависит ни от timestep, ни от
	// числа итераций (XPBD, Macklin et al. 2016).
	const float alpha_tilde = p_params.stretch_compliance / h2;

	const int segments = r_state.segment_count();
	for (int i = 0; i < segments; ++i) {
		const int a = i;
		const int b = i + 1;
		const float w_a = r_state.solve_inv_mass(a);
		const float w_b = r_state.solve_inv_mass(b);
		const float w_sum = w_a + w_b;
		if (w_sum < EPS_DENOM) {
			continue;
		}

		const Vec3 d = r_state.position[a] - r_state.position[b];
		const float len = length(d);
		// Совпавшие частицы: градиент не определён, разводит их bending-проход.
		if (len < EPS_LENGTH) {
			continue;
		}
		const Vec3 n = d * (1.0f / len);
		const float c = len - r_state.rest_length[i];

		float &lambda = r_state.lambda_stretch[i];
		const float delta_lambda = (-c - alpha_tilde * lambda) / (w_sum + alpha_tilde);
		lambda += delta_lambda;

		r_state.position[a] += n * (w_a * delta_lambda);
		r_state.position[b] -= n * (w_b * delta_lambda);
	}
}

// Односторонняя проекция на сферу радиуса p_max_distance вокруг p_origin.
static inline void project_inside_sphere(CableState &r_state, int i, const Vec3 &p_origin, float p_max_distance) {
	if (r_state.solve_inv_mass(i) < EPS_DENOM) {
		return;
	}
	const Vec3 d = r_state.position[i] - p_origin;
	const float len = length(d);
	if (len <= p_max_distance || len < EPS_LENGTH) {
		return;
	}
	// LRA — позиционная правка длины, а не сила: она действует сразу по всей длине,
	// поэтому её вклад в скорость выглядел бы мгновенной отдачей всего кабеля.
	r_state.project(i, (p_origin + d * (p_max_distance / len)) - r_state.position[i]);
}

void solve_lra(CableState &r_state) {
	const int n = r_state.count;
	if (n < 2) {
		return;
	}

	// Конец на катушке обрабатывается первым, а неподвижная точка A — последней.
	// Порядок принципиален: в невозможной конфигурации (игрок дальше, чем позволяет
	// кабель) два LRA противоречат друг другу, и выиграть должна точка A, иначе
	// нарушение длины уезжает в первый сегмент вместо последнего, где его видят
	// катушка и ограничение перемещения игрока.
	if (r_state.is_pinned(n - 1)) {
		const Vec3 origin = r_state.position[n - 1];
		const float total = r_state.arc_from_start[n - 1];
		for (int i = n - 2; i >= 0; --i) {
			project_inside_sphere(r_state, i, origin, total - r_state.arc_from_start[i]);
		}
	}

	if (r_state.is_pinned(0)) {
		const Vec3 origin = r_state.position[0];
		for (int i = 1; i < n; ++i) {
			project_inside_sphere(r_state, i, origin, r_state.arc_from_start[i]);
		}
	}
}

} // namespace cable
