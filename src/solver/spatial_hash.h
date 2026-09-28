#pragma once

#include "solver/solver_math.h"

#include <vector>

namespace cable {

// Равномерная хеш-сетка по частицам. Коллизии хеша разрешаются сравнением
// координат ячейки, поэтому одна частица посещается запросом ровно один раз —
// иначе позиционная коррекция применялась бы дважды.
class SpatialHash {
public:
	void reserve(int p_max_entries);
	void build(const Vec3 *p_positions, const uint8_t *p_flags, int p_count, uint8_t p_skip_mask, float p_cell_size);

	template <typename Callback>
	void for_each_neighbor(const Vec3 &p_point, Callback &&p_callback) const {
		if (bucket_start.empty()) {
			return;
		}
		const int32_t cx = to_cell(p_point.x);
		const int32_t cy = to_cell(p_point.y);
		const int32_t cz = to_cell(p_point.z);

		for (int32_t dz = -1; dz <= 1; ++dz) {
			for (int32_t dy = -1; dy <= 1; ++dy) {
				for (int32_t dx = -1; dx <= 1; ++dx) {
					const Cell cell = { cx + dx, cy + dy, cz + dz };
					const uint32_t slot = hash_cell(cell);
					const uint32_t from = bucket_start[slot];
					const uint32_t to = bucket_start[slot + 1];
					for (uint32_t k = from; k < to; ++k) {
						const int32_t index = entries[k];
						if (entry_cell[index] == cell) {
							p_callback(static_cast<int>(index));
						}
					}
				}
			}
		}
	}

private:
	struct Cell {
		int32_t x = 0;
		int32_t y = 0;
		int32_t z = 0;

		bool operator==(const Cell &o) const { return x == o.x && y == o.y && z == o.z; }
	};

	int32_t to_cell(float v) const {
		return static_cast<int32_t>(std::floor(v * inv_cell_size));
	}

	uint32_t hash_cell(const Cell &c) const {
		const uint32_t h = static_cast<uint32_t>(c.x) * 73856093u ^
				static_cast<uint32_t>(c.y) * 19349663u ^
				static_cast<uint32_t>(c.z) * 83492791u;
		return h & table_mask;
	}

	float inv_cell_size = 1.0f;
	uint32_t table_mask = 0;
	std::vector<uint32_t> bucket_start; // размер table_size + 1
	std::vector<uint32_t> cursor;
	std::vector<int32_t> entries;
	std::vector<Cell> entry_cell; // по индексу частицы, для точного сравнения ячеек
};

} // namespace cable
