#pragma once

#include "solver/cable_params.h"
#include "solver/solver_math.h"

#include <vector>

namespace cable {

enum ParticleFlags : uint8_t {
	PARTICLE_PINNED = 1 << 0,
	PARTICLE_ASLEEP = 1 << 1,
	PARTICLE_IN_CONTACT = 1 << 2,
	PARTICLE_BAD_NORMAL = 1 << 3, // нормаль контакта вырождена, берём направление движения
};

// Якорь контакта живёт дольше одного подшага — без него статического трения нет.
// Точка касания хранится в локальном пространстве коллайдера, чтобы ехать вместе с ним.
struct ContactAnchor {
	uint64_t collider_key = 0; // 0 — контакта нет
	Vec3 anchor_local;
	Vec3 normal;
	float lambda_normal = 0.0f;
	uint8_t age = 0;

	void clear() {
		collider_key = 0;
		lambda_normal = 0.0f;
		age = 0;
	}
};

// Контакт внутренней точки сегмента с миром. Параметр задаёт точку
// lerp(position[i], position[i + 1], parameter), а lambda нужна velocity-pass.
struct SegmentContact {
	uint64_t collider_key = 0;
	float parameter = 0.0f;
	Vec3 normal;
	float lambda_normal = 0.0f;

	void clear() {
		collider_key = 0;
		parameter = 0.0f;
		normal = Vec3();
		lambda_normal = 0.0f;
	}
};

// SoA: от раскладки зависят хеш-сетка самопересечения, усыпление и возможный
// вынос горячего цикла в отдельный поток.
struct CableState {
	std::vector<Vec3> position;
	std::vector<Vec3> prev_position; // позиция на начало подшага
	std::vector<Vec3> frame_position; // позиция на начало кадра, для усыпления
	std::vector<Vec3> velocity;
	// Смещения от жёстких проекций с нулевой податливостью (LRA, предел изгиба,
	// коллизии, самопересечение). Они вычитаются при выводе скорости: иначе поправка
	// превращается в скорость как dx/h и при малом подшаге впрыскивает энергию,
	// от которой кабель кипит и отдаёт по всей длине.
	std::vector<Vec3> projection_delta;
	std::vector<float> inv_mass;
	std::vector<uint8_t> flags;
	std::vector<uint8_t> sleep_counter;
	std::vector<float> rest_length; // длина сегмента i..i+1, у последнего переменная
	std::vector<float> arc_from_start; // дуговая длина от точки A, для LRA и UV
	std::vector<float> lambda_stretch; // накопленная lambda сегмента, отсюда натяжение
	std::vector<float> lambda_bend; // накопленная lambda сустава i
	std::vector<ContactAnchor> contacts;
	std::vector<SegmentContact> segment_contacts;

	int count = 0;

	void allocate(int p_capacity);
	void reset_count() { count = 0; }

	int capacity() const { return static_cast<int>(position.size()); }
	int segment_count() const { return count > 0 ? count - 1 : 0; }

	bool is_pinned(int i) const { return (flags[i] & PARTICLE_PINNED) != 0; }
	bool is_asleep(int i) const { return (flags[i] & PARTICLE_ASLEEP) != 0; }

	// Спящая частица участвует в constraint как бесконечно тяжёлая: полный пропуск
	// constraint разрушал бы длину на границе спящего участка.
	float solve_inv_mass(int i) const { return is_asleep(i) ? 0.0f : inv_mass[i]; }

	// Сдвиг частицы жёсткой проекцией: позиция меняется, скорость — нет.
	void project(int i, const Vec3 &p_delta) {
		position[i] += p_delta;
		projection_delta[i] += p_delta;
	}

	void append(const Vec3 &p_position, float p_inv_mass);
	void wake(int i);
	void wake_window(int i);

	// Суммарная rest-длина размотанной части.
	float deployed_length() const;
	// Фактическая длина ломаной, для отладочного сравнения с cable_length.
	float polyline_length() const;

	Bounds compute_bounds(float p_radius) const;
	void recompute_arc_lengths();
	void distribute_masses(const CableParams &p_params);
};

} // namespace cable
