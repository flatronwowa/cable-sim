#pragma once

#include "solver/solver_math.h"

namespace cable {

// Перемещение частицы за подшаг не должно превышать этой доли толщины кабеля —
// из этого выводится число подшагов и порог включения свипов.
constexpr float SUBSTEP_SAFETY = 0.5f;

// Усыпление.
// Доля запаса, с которым катушка отдаёт кабель: сошедший участок длиннее прямой на
// эту долю, поэтому кабель ложится на пол, а не остаётся натянутым канатом.
constexpr float SPOOL_SLACK = 0.15f;
// Порог, ниже которого катушка трогается. Строго меньше SPOOL_SLACK, иначе выдача
// не запирается и катушка подтравливает кабель у стоящего игрока.
constexpr float SPOOL_SLACK_THRESHOLD = 0.06f;

constexpr uint8_t SLEEP_FRAMES = 12;
constexpr float SLEEP_EPSILON = 2.0e-4f;
constexpr int WAKE_WINDOW = 4; // будится окно соседей, иначе волна пробуждения отстаёт
constexpr float WAKE_DISPLACEMENT = 4.0f * SLEEP_EPSILON;
// Отдельного пробуждения по натяжению нет: в цепочке длина сегмента меняется только
// когда движется соседняя частица, а это уже покрыто пробуждением по смещению.

// Широкая фаза: список форм-кандидатов на частицу строится раз в кадр.
constexpr int MAX_CANDIDATE_SHAPES = 16;

// Верхняя граница дискретизации. Без неё cable_length в сотни метров молча
// деградирует до неиграбельного фреймрейта.
constexpr int MAX_PARTICLES = 2048;

// Смещение якоря больше этого порога классифицируется как телепорт.
constexpr float TELEPORT_DISTANCE = 1.0f;

// Пре-релаксация перед первым кадром, иначе кабель оседает на глазах у игрока.
constexpr int PRERELAX_STEPS = 40;
constexpr float SOLVER_STRETCH_TOLERANCE = 0.02f;
constexpr int BEND_STRETCH_ITERATIONS = 4;

struct CableParams {
	// Геометрия.
	float cable_length = 10.0f;
	float cable_radius = 0.015f;
	float target_segment_length = 0.06f;

	// Материал и физика.
	float mass_per_meter = 0.2f;
	float stretch_compliance = 0.0f;
	float bending_compliance = 1.0f;
	// У реального кабеля минимальный радиус изгиба — это 4-8 его диаметров. При
	// радиусе по умолчанию 0.015 это и даёт 0.15: кабель ложится широкими витками,
	// а не сворачивается в узел толщиной с себя самого.
	float minimum_bend_radius = 0.15f;
	// Внутреннее вязкое трение кабеля, 1/с: гасится относительная скорость соседних
	// частиц. Ноль означает, что энергия волн не рассеивается вообще и уложенный
	// кабель дрожит бесконечно, поэтому значение по умолчанию заметное.
	float damping = 80.0f;
	float static_friction = 0.9f;
	float dynamic_friction = 0.7f;
	float gravity_scale = 1.0f;
	// Направление выхода по умолчанию практически свободно: constraint на плече
	// в один сегмент работает как актуатор и подкачивает энергию в кабель.
	float anchor_stiffness_compliance = 1.0e6f;

	Vec3 gravity = Vec3(0.0f, -9.8f, 0.0f);

	// Солвер.
	int substeps_min = 2;
	int substeps_max = 16;
	int solver_iterations = 1; // максимум; выход раньше при достижении допуска
	float collision_margin = 2.0e-3f;
	bool self_collision_enabled = true;
	bool sleep_enabled = false;

	// Сегмент короче двух радиусов делает самопересечение вырожденным и ничего не даёт.
	float min_segment_length() const { return 2.0f * cable_radius; }

	float effective_segment_length() const {
		return maxf(target_segment_length, min_segment_length());
	}

	// Число частиц, на которое выделяются массивы: реаллокаций в рантайме нет.
	int particle_capacity() const {
		const float seg = effective_segment_length();
		if (seg < EPS_LENGTH) {
			return MAX_PARTICLES;
		}
		const int needed = static_cast<int>(cable_length / seg) + 4;
		return needed < MAX_PARTICLES ? needed : MAX_PARTICLES;
	}

	// Максимальный угол между сегментами длины L при минимальном радиусе изгиба R:
	// theta_max = 2 * asin(L / (2R)), с защитой от аргумента вне [-1, 1].
};

} // namespace cable
