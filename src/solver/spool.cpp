#include "solver/spool.h"

namespace cable {

// Новая частица отщепляется от последнего сегмента. Конец кабеля переезжает на
// следующий индекс, поэтому операция стоит O(1), а arc_from_start пересчитывается
// только потому, что хвост у нас короткий.
static bool split_last_segment(CableState &r_state, const CableParams &p_params, const Vec3 &p_end_target) {
	const int n = r_state.count;
	if (n < 2 || n >= r_state.capacity()) {
		return false;
	}
	const int joint = n - 2; // сегмент joint..old_last — переменной длины
	const float target = p_params.effective_segment_length();
	const float min_remainder = maxf(0.5f * target, p_params.min_segment_length());
	if (r_state.rest_length[joint] <= target + min_remainder) {
		return false;
	}

	const int old_last = n - 1;
	const int new_last = n;

	r_state.position[new_last] = r_state.position[old_last];
	r_state.prev_position[new_last] = r_state.prev_position[old_last];
	r_state.frame_position[new_last] = r_state.frame_position[old_last];
	r_state.velocity[new_last] = r_state.velocity[old_last];
	r_state.inv_mass[new_last] = r_state.inv_mass[old_last];
	r_state.flags[new_last] = r_state.flags[old_last];
	r_state.sleep_counter[new_last] = 0;
	r_state.contacts[new_last] = r_state.contacts[old_last];

	// Новая частица садится на отрезок joint..end_target пропорционально доле
	// rest length, которую забирает. Шаг именно в долях, а не на target метров:
	// при выдаче нескольких частиц за кадр отсчёт в метрах перелетает за конец
	// кабеля, и частицы сваливаются в одну точку с разрывом за ней.
	const float previous_rest = r_state.rest_length[joint];
	const Vec3 to_end = p_end_target - r_state.position[joint];
	const float span = length(to_end);
	Vec3 direction = safe_normalize(to_end);
	if (length_squared(direction) == 0.0f) {
		direction = safe_normalize(r_state.position[joint] - r_state.position[joint > 0 ? joint - 1 : joint]);
	}
	if (length_squared(direction) == 0.0f) {
		direction = Vec3(0.0f, -1.0f, 0.0f);
	}

	r_state.position[old_last] = r_state.position[joint] + direction * (span * target / previous_rest);
	// Частица рождается со скоростью конца кабеля: нулевая скорость посреди
	// летящего кабеля даёт рывок distance-constraint и скачок числа substeps.
	r_state.prev_position[old_last] = r_state.position[old_last] -
			(r_state.position[new_last] - r_state.prev_position[new_last]);
	r_state.frame_position[old_last] = r_state.position[old_last];
	r_state.velocity[old_last] = r_state.velocity[new_last];
	r_state.flags[old_last] = 0; // новая частица не закреплена и не спит
	r_state.sleep_counter[old_last] = 0;
	r_state.lambda_stretch[old_last] = 0.0f;
	r_state.lambda_bend[old_last] = 0.0f;
	r_state.contacts[old_last].clear();

	r_state.rest_length[joint] = target;
	r_state.rest_length[old_last] = previous_rest - target;
	r_state.count = new_last + 1;

	r_state.recompute_arc_lengths();
	r_state.distribute_masses(p_params);
	return true;
}

// Последняя по длине частица, которой кабель держится за мир. Если контактов нет,
// опорой остаётся неподвижный конец.
static int last_support(const CableState &p_state) {
	for (int i = p_state.count - 2; i >= 0; --i) {
		if ((p_state.flags[i] & PARTICLE_IN_CONTACT) != 0) {
			return i;
		}
	}
	return 0;
}

void spool_update(CableState &r_state, const CableParams &p_params, const Vec3 &p_end_target,
		const Vec3 &p_previous_end_target) {
	const int n = r_state.count;
	if (n < 2) {
		return;
	}
	float stowed = stowed_length(r_state, p_params);
	if (stowed <= EPS_LENGTH) {
		return;
	}

	// Кабель сходит с катушки, когда пролёт от последней точки опоры до разъёма
	// требует больше кабеля, чем в этом пролёте есть. Опора — последняя частица в
	// контакте с миром: именно там кабель держится, а дальше свободен.
	//
	// Мерить растяжение последнего сегмента нельзя, хотя это и напрашивается: окно
	// шириной в один сегмент не видит провис, лежащий дальше по длине. Любое смещение
	// руки больше 7 см выглядит как спрос на кабель, и катушка разматывается от того,
	// что игрок просто вертит головой. Замер: за 600 кадров осмотра по сторонам без
	// единого шага число частиц выросло с 41 до 93, то есть катушка высыпала четыре
	// метра кабеля на пол. Смотки нет, и вернуть это уже нечем.
	//
	// Силовой критерий (выдавать, пока натяжение у разъёма выше обратного натяжения
	// барабана) выглядит ещё физичнее, но измеренно идёт в разгон: выданный кабель
	// добавляет веса на разъём, натяжение растёт, выдача ускоряется. За один прогон
	// катушка высыпала все 40 м, деформация дошла до 220%, скорости до 20 м/с.
	const int pivot = last_support(r_state);
	const int joint = n - 2;
	const float span = distance(r_state.position[pivot], p_end_target);
	const float previous_span = distance(r_state.position[pivot], p_previous_end_target);
	const float available = r_state.arc_from_start[n - 1] - r_state.arc_from_start[pivot];

	// Порог и цель различаются намеренно. Цель с запасом нужна, потому что кабель
	// нерастяжим: выдавая ровно в натяг, катушка оставляет за игроком натянутый канат.
	// Замер: на 147 частиц приходилось 1-2 контакта с миром — кабель висел в воздухе
	// между столбом и рукой вместо того, чтобы лечь на пол. Отсюда и ощущение, что вес
	// кабеля не учитывается, и звон натянутой струны, и невозможность уснуть: сон
	// требует контакта. При запасе 0.15 контактов стало 76 вместо 8.
	// Порог же ниже цели, иначе равновесие геометрически неточно и катушка
	// подтравливает кабель по доле миллиметра у стоящего игрока, будя соседей.
	if (available < span * (1.0f + SPOOL_SLACK_THRESHOLD)) {
		const float wanted = span * (1.0f + SPOOL_SLACK) - available;
		const float outward_travel = maxf(span - previous_span, 0.0f);
		const float max_payout = outward_travel * (1.0f + SPOOL_SLACK);
		const float given = minf(minf(wanted, stowed), max_payout);
		if (given > EPS_LENGTH) {
			r_state.rest_length[joint] += given;
			r_state.recompute_arc_lengths();
			r_state.distribute_masses(p_params);
			// Размотка означает движение: соседний участок не должен остаться спящим.
			r_state.wake_window(n - 1);
		}
	}

	while (split_last_segment(r_state, p_params, p_end_target)) {
	}
}

Vec3 constrain_motion(const CableState &p_state, const CableParams &p_params,
		const Vec3 &p_from, const Vec3 &p_to, bool &r_limited) {
	r_limited = false;
	if (p_state.count < 2) {
		return p_to;
	}

	// Точка перегиба — там же, где кабель держится за мир при выдаче.
	const int pivot = last_support(p_state);

	const Vec3 origin = p_state.position[pivot];
	const float deployed_available =
			p_state.arc_from_start[p_state.count - 1] - p_state.arc_from_start[pivot];
	const float available = deployed_available + stowed_length(p_state, p_params);

	const Vec3 radial = p_from - origin;
	const float radial_length = length(radial);
	if (radial_length < EPS_LENGTH) {
		return p_to;
	}
	const Vec3 outward = radial * (1.0f / radial_length);

	Vec3 motion = p_to - p_from;
	const float away = dot(motion, outward);
	const float slack = available - radial_length;
	if (away > slack) {
		motion -= outward * (away - maxf(slack, 0.0f));
		r_limited = true;
	}
	return p_from + motion;
}

} // namespace cable
