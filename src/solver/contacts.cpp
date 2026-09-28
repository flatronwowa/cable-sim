#include "solver/contacts.h"

namespace cable {

namespace {

// Число уточнений положения ближайшей к поверхности точки сегмента.
constexpr int SEGMENT_REFINE_ITERATIONS = 4;
// Ближе этого к концам сегмента контакт уже пойман проходом по частицам.
constexpr float SEGMENT_INTERIOR_MARGIN = 0.05f;
// Шагов подразбиения аварийного свипа.
constexpr int SWEEP_SAMPLES = 8;
// Чередований проекций для частицы, зажатой между несколькими формами.
constexpr int CONTACT_PASSES = 4;
// Один collider может менять нормаль скачком на ребре. Старый friction-anchor
// после такого перехода находится уже на другой грани и дёргает частицу назад.
constexpr float CONTACT_NORMAL_PERSISTENCE_COS = 0.8f;
// Свип помечает уже касающиеся формы битовой маской по слотам broadphase.
static_assert(MAX_CANDIDATE_SHAPES <= 32, "маска покоящихся форм в свипе не вместит слоты");

struct DeepestContact {
	int shape_index = -1;
	float distance = 1.0e30f;
	Vec3 normal;
};

// Ближайшая к поверхности точка сегмента ищется итерацией с неподвижной точкой:
// точка на сегменте -> ближайшая точка поверхности (p - sd * grad) -> проекция назад
// на сегмент. Для выпуклых форм это глобальный минимум, для остальных — локальный,
// чего достаточно, поскольку перемещение за подшаг ограничено радиусом.
float find_closest_param(const WorldSnapshot &p_world, int p_shape, const Vec3 &p_a, const Vec3 &p_b,
		const Vec3 &p_reference, ShapeSample &r_sample) {
	float best_t = 0.0f;
	ShapeSample best = p_world.sample(p_shape, p_a, p_reference);
	const float candidates[2] = { 0.5f, 1.0f };
	for (int k = 0; k < 2; ++k) {
		const ShapeSample s = p_world.sample(p_shape, lerp(p_a, p_b, candidates[k]), p_reference);
		if (s.valid && (!best.valid || s.distance < best.distance)) {
			best = s;
			best_t = candidates[k];
		}
	}

	for (int it = 0; it < SEGMENT_REFINE_ITERATIONS; ++it) {
		if (!best.valid) {
			break;
		}
		const Vec3 point = lerp(p_a, p_b, best_t);
		const Vec3 surface = point - best.gradient * best.distance;
		const float t = closest_param_on_segment(p_a, p_b, surface);
		const ShapeSample s = p_world.sample(p_shape, lerp(p_a, p_b, t), p_reference);
		if (!s.valid || s.distance >= best.distance) {
			break;
		}
		best = s;
		best_t = t;
	}

	r_sample = best;
	return best_t;
}

// Аварийный путь: рывок якоря или телепорт. Частица переставляется в последнюю точку
// своего перемещения, где её центр ещё не внутри тела.
//
// Критерий именно «центр внутри» (sd < 0), а не «ближе target к поверхности»: во
// втором случае под свип попадает нормальный покоящийся контакт, где частица и должна
// стоять в target от поверхности. Тогда свип откатывает весь подшаг целиком вместе с
// коррекцией длины, и частица примерзает к геометрии навсегда. Выталкивания по
// градиенту достаточно, пока центр остался снаружи.
void sweep_particle(CableState &r_state, const WorldSnapshot &p_world, const Broadphase &p_broadphase,
		int p_particle) {
	const Vec3 from = r_state.prev_position[p_particle];
	const Vec3 to = r_state.position[p_particle];

	// Формы, внутри которых частица находилась уже в начале подшага, исключаются:
	// откатывать перемещение к точке, которая сама внутри тела, бессмысленно.
	uint32_t inside_at_start = 0;
	int index = 0;
	for (int slot = p_broadphase.begin(p_particle); slot < p_broadphase.end(p_particle); ++slot, ++index) {
		const ShapeSample s = p_world.sample(p_broadphase.shape_at(slot), from, from);
		if (s.valid && s.distance < 0.0f) {
			inside_at_start |= 1u << index;
		}
	}

	for (int step = 1; step <= SWEEP_SAMPLES; ++step) {
		const float t = static_cast<float>(step) / SWEEP_SAMPLES;
		const Vec3 point = lerp(from, to, t);
		index = 0;
		for (int slot = p_broadphase.begin(p_particle); slot < p_broadphase.end(p_particle); ++slot, ++index) {
			if ((inside_at_start & (1u << index)) != 0) {
				continue;
			}
			const ShapeSample s = p_world.sample(p_broadphase.shape_at(slot), point, from);
			if (s.valid && s.distance < 0.0f) {
				const float safe_t = static_cast<float>(step - 1) / SWEEP_SAMPLES;
				const Vec3 safe_position = lerp(from, to, safe_t);
				r_state.project(p_particle, safe_position - r_state.position[p_particle]);
				return;
			}
		}
	}
}

void apply_segment_impulse(CableState &r_state, int p_segment, float p_parameter,
		const Vec3 &p_impulse) {
	const float bary_a = 1.0f - p_parameter;
	const float bary_b = p_parameter;
	r_state.velocity[p_segment] +=
			p_impulse * (r_state.solve_inv_mass(p_segment) * bary_a);
	r_state.velocity[p_segment + 1] +=
			p_impulse * (r_state.solve_inv_mass(p_segment + 1) * bary_b);
}

} // namespace

void solve_world_contacts(CableState &r_state, const CableParams &p_params,
		const WorldSnapshot &p_world, const Broadphase &p_broadphase) {
	if (p_world.shape_count() == 0) {
		return;
	}
	const float target = p_params.cable_radius + p_params.collision_margin;
	const float sweep_threshold = p_params.cable_radius * SUBSTEP_SAFETY;

	for (int i = 0; i < r_state.count; ++i) {
		const float w = r_state.solve_inv_mass(i);
		if (w < EPS_DENOM) {
			continue;
		}

		if (distance(r_state.position[i], r_state.prev_position[i]) > sweep_threshold) {
			sweep_particle(r_state, p_world, p_broadphase, i);
		}

		// Выталкивания форм применяются последовательно, поэтому в щели между двумя
		// поверхностями (шар на полу) последнее затирает предыдущее и частица остаётся
		// вдавленной в одну из них. Проекции чередуются до сходимости, но только когда
		// нарушителей действительно больше одного: у обычного контакта проход один.
		// deepest не сбрасывается между проходами — за нормальную силу отвечает самое
		// глубокое проникновение подшага, из него же берётся бюджет трения.
		DeepestContact deepest;
		for (int pass = 0; pass < CONTACT_PASSES; ++pass) {
			int violating = 0;
			for (int slot = p_broadphase.begin(i); slot < p_broadphase.end(i); ++slot) {
				const int shape = p_broadphase.shape_at(slot);
				const ShapeSample s = p_world.sample(shape, r_state.position[i], r_state.prev_position[i]);
				if (!s.valid) {
					// Нормаль вырождена: контакт пропускается явно, а не делится на ноль.
					r_state.flags[i] |= PARTICLE_BAD_NORMAL;
					continue;
				}
				if (s.distance >= target) {
					continue;
				}
				violating += 1;
				r_state.project(i, s.gradient * (target - s.distance));
				if (s.distance < deepest.distance) {
					deepest.distance = s.distance;
					deepest.normal = s.gradient;
					deepest.shape_index = shape;
				}
			}
			if (violating <= 1) {
				break;
			}
		}

		if (deepest.shape_index < 0) {
			r_state.contacts[i].clear();
			r_state.flags[i] &= static_cast<uint8_t>(~PARTICLE_IN_CONTACT);
			continue;
		}

		r_state.flags[i] |= PARTICLE_IN_CONTACT;
		r_state.flags[i] &= static_cast<uint8_t>(~PARTICLE_BAD_NORMAL);

		ContactAnchor &anchor = r_state.contacts[i];
		const uint64_t key = p_world.get_shape(deepest.shape_index).key;
		const bool normal_changed = length_squared(anchor.normal) >= EPS_NORMAL &&
				dot(anchor.normal, deepest.normal) < CONTACT_NORMAL_PERSISTENCE_COS;
		if (anchor.collider_key != key || normal_changed) {
			anchor.collider_key = key;
			anchor.anchor_local = p_world.to_local(deepest.shape_index, r_state.position[i]);
			anchor.age = 0;
		} else if (anchor.age < 255) {
			anchor.age += 1;
		}
		anchor.normal = deepest.normal;
		// lambda нормального contact-constraint при alpha = 0: C = sd - target.
		// Накапливается за подшаг, а не перезаписывается: при нескольких итерациях
		// последняя видит уже устранённое проникновение, оценивает нормальную силу как
		// ноль и обнуляет бюджет трения в velocity pass. Трение при этом пропадало
		// целиком, кабель скользил свободно и никогда не укладывался.
		anchor.lambda_normal += (target - deepest.distance) / w;

		// Статическое трение: тангенциальное смещение относительно якоря гасится
		// полностью, пока lambda_t < mu_s * lambda_n. Оба lambda делятся на одну и ту
		// же обратную массу, поэтому сравнение сводится к смещениям.
		const Vec3 anchor_world = p_world.to_world(deepest.shape_index, anchor.anchor_local);
		const Vec3 offset = r_state.position[i] - anchor_world;
		const Vec3 tangential = offset - anchor.normal * dot(anchor.normal, offset);
		const float slip = length(tangential);
		if (slip > EPS_LENGTH) {
			// Поправка ограничивается конусом Кулона (Macklin et al., Unified Particle
			// Physics), а не решается по принципу «всё или ничего». Полное гашение ниже
			// порога и полный отпуск выше давали разрыв ровно там, где трение должно
			// держать сильнее всего: накопленное смещение вылетало обратно как скорость.
			// Замер: рывок рукой на 2 см поднимал скорость до 3.3 м/с в середине длины,
			// а с выключенным трением — до 0.5 м/с у самой руки.
			const float limit = p_params.static_friction * (target - deepest.distance);
			r_state.position[i] -= tangential * (minf(slip, limit) / slip);
			if (slip > limit) {
				// Скольжение: якорь тащится за частицей, а не сбрасывается рывком.
				anchor.anchor_local = p_world.to_local(deepest.shape_index, r_state.position[i]);
				anchor.age = 0;
			}
		}
	}

	// Проход по сегментам: только капсульный тест ловит случай, когда обе соседние
	// частицы снаружи, а отрезок между ними протыкает стену.
	for (int i = 0; i < r_state.segment_count(); ++i) {
		const int a = i;
		const int b = i + 1;
		const float w_a = r_state.solve_inv_mass(a);
		const float w_b = r_state.solve_inv_mass(b);
		if (w_a + w_b < EPS_DENOM) {
			continue;
		}
		const Vec3 reference = lerp(r_state.prev_position[a], r_state.prev_position[b], 0.5f);
		const float half_length = 0.5f * distance(r_state.position[a], r_state.position[b]);

		for (int slot = p_broadphase.begin(a); slot < p_broadphase.end(a); ++slot) {
			const int shape = p_broadphase.shape_at(slot);

			// Signed distance липшицева с константой 1, поэтому внутренняя точка
			// сегмента не ближе к поверхности, чем конец минус половина длины. Пара
			// сэмплов на концах отсекает подавляющее большинство сегментов до поиска
			// ближайшей точки, который стоит семь сэмплов.
			const ShapeSample sa = p_world.sample(shape, r_state.position[a], reference);
			const ShapeSample sb = p_world.sample(shape, r_state.position[b], reference);
			if (sa.valid && sb.valid && minf(sa.distance, sb.distance) - half_length >= target) {
				continue;
			}

			ShapeSample s;
			const float t = find_closest_param(p_world, shape, r_state.position[a], r_state.position[b], reference, s);
			if (!s.valid || s.distance >= target) {
				continue;
			}
			if (t < SEGMENT_INTERIOR_MARGIN || t > 1.0f - SEGMENT_INTERIOR_MARGIN) {
				continue; // концы уже обработаны проходом по частицам
			}

			// Коррекция распределяется на обе концевые частицы по барицентрическому
			// весу точки контакта и обратным массам.
			const float bary_a = 1.0f - t;
			const float bary_b = t;
			const float denom = w_a * bary_a * bary_a + w_b * bary_b * bary_b;
			if (denom < EPS_DENOM) {
				continue;
			}
			const float lambda = (target - s.distance) / denom;
			r_state.project(a, s.gradient * (w_a * bary_a * lambda));
			r_state.project(b, s.gradient * (w_b * bary_b * lambda));

			// Позиционная проекция исключена из восстановленной скорости, поэтому
			// входящая скорость сегмента сама не исчезнет. Сохраняем самый сильный
			// контакт сегмента для отдельного неупругого velocity-pass.
			SegmentContact &contact = r_state.segment_contacts[i];
			const uint64_t key = p_world.get_shape(shape).key;
			if (contact.collider_key == key) {
				contact.lambda_normal += lambda;
				contact.parameter = t;
				contact.normal = s.gradient;
			} else if (lambda > contact.lambda_normal) {
				contact.collider_key = key;
				contact.parameter = t;
				contact.normal = s.gradient;
				contact.lambda_normal = lambda;
			}
		}
	}
}

void world_contacts_velocity_pass(CableState &r_state, const CableParams &p_params, float p_h) {
	if (p_h < EPS_LENGTH) {
		return;
	}

	for (int i = 0; i < r_state.count; ++i) {
		if (r_state.is_pinned(i) || r_state.is_asleep(i)) {
			continue;
		}
		const ContactAnchor &anchor = r_state.contacts[i];
		if (anchor.collider_key == 0) {
			continue;
		}
		const Vec3 n = anchor.normal;
		if (length_squared(n) < EPS_NORMAL) {
			continue;
		}

		// Нулевая restitution по Müller et al. (XPBD rigid bodies): у разрешённого
		// контакта нормальная скорость обнуляется целиком. Гасить одно сближение
		// недостаточно — позиционное выталкивание само рождает скорость отрыва
		// порядка g*h каждый подшаг, и уложенный кабель от неё дрожит вечно, а из-за
		// дрожания никогда не засыпает.
		Vec3 v = r_state.velocity[i];
		v -= n * dot(v, n);

		Vec3 v_tangent = v - n * dot(n, v);
		const float v_tangent_len = length(v_tangent);
		if (v_tangent_len > EPS_LENGTH) {
			// Бюджет динамического трения (Müller et al., XPBD rigid bodies):
			// dv = w * h * mu_d * f_n, где f_n = lambda_n / h^2.
			const float budget = p_params.dynamic_friction * r_state.inv_mass[i] * anchor.lambda_normal / p_h;
			v -= v_tangent * (minf(v_tangent_len, budget) / v_tangent_len);
		}

		r_state.velocity[i] = v;
	}

	for (int i = 0; i < r_state.segment_count(); ++i) {
		const SegmentContact &contact = r_state.segment_contacts[i];
		if (contact.collider_key == 0 || length_squared(contact.normal) < EPS_NORMAL) {
			continue;
		}

		const float bary_a = 1.0f - contact.parameter;
		const float bary_b = contact.parameter;
		const float w_sum =
				r_state.solve_inv_mass(i) * bary_a * bary_a +
				r_state.solve_inv_mass(i + 1) * bary_b * bary_b;
		if (w_sum < EPS_DENOM) {
			continue;
		}

		Vec3 velocity = lerp(r_state.velocity[i], r_state.velocity[i + 1], contact.parameter);
		const float normal_speed = dot(velocity, contact.normal);
		float normal_impulse = contact.lambda_normal / p_h;
		if (normal_speed < 0.0f) {
			const float impulse = -normal_speed / w_sum;
			normal_impulse += impulse;
			apply_segment_impulse(r_state, i, contact.parameter, contact.normal * impulse);
			velocity -= contact.normal * normal_speed;
		}

		const Vec3 tangent = velocity - contact.normal * dot(velocity, contact.normal);
		const float tangent_speed = length(tangent);
		if (tangent_speed < EPS_LENGTH || normal_impulse <= 0.0f) {
			continue;
		}
		const float required = tangent_speed / w_sum;
		const float static_limit = p_params.static_friction * normal_impulse;
		const float impulse = required <= static_limit
				? required
				: minf(required, p_params.dynamic_friction * normal_impulse);
		apply_segment_impulse(r_state, i, contact.parameter,
				tangent * (-impulse / tangent_speed));
	}
}

} // namespace cable
