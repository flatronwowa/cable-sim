#include "solver/spatial_hash.h"

#include <algorithm>

namespace cable {

static uint32_t next_power_of_two(uint32_t v) {
	uint32_t p = 1;
	while (p < v) {
		p <<= 1;
	}
	return p;
}

void SpatialHash::reserve(int p_max_entries) {
	if (p_max_entries < 1) {
		p_max_entries = 1;
	}
	const uint32_t table_size = next_power_of_two(static_cast<uint32_t>(p_max_entries) * 2u);
	table_mask = table_size - 1u;
	bucket_start.assign(table_size + 1, 0u);
	cursor.assign(table_size, 0u);
	entries.assign(static_cast<size_t>(p_max_entries), 0);
	entry_cell.assign(static_cast<size_t>(p_max_entries), Cell());
}

void SpatialHash::build(const Vec3 *p_positions, const uint8_t *p_flags, int p_count, uint8_t p_skip_mask, float p_cell_size) {
	if (bucket_start.empty() || p_count > static_cast<int>(entries.size())) {
		reserve(p_count);
	}
	inv_cell_size = p_cell_size > EPS_LENGTH ? 1.0f / p_cell_size : 1.0f;

	const uint32_t table_size = static_cast<uint32_t>(cursor.size());
	std::fill(bucket_start.begin(), bucket_start.end(), 0u);

	// Подсчёт заполнения корзин: bucket_start[slot + 1] используется как счётчик,
	// чтобы затем префиксной суммой превратиться в границы.
	for (int i = 0; i < p_count; ++i) {
		if ((p_flags[i] & p_skip_mask) != 0) {
			continue;
		}
		const Cell cell = { to_cell(p_positions[i].x), to_cell(p_positions[i].y), to_cell(p_positions[i].z) };
		entry_cell[i] = cell;
		bucket_start[hash_cell(cell) + 1] += 1u;
	}

	for (uint32_t s = 0; s < table_size; ++s) {
		bucket_start[s + 1] += bucket_start[s];
		cursor[s] = bucket_start[s];
	}

	for (int i = 0; i < p_count; ++i) {
		if ((p_flags[i] & p_skip_mask) != 0) {
			continue;
		}
		entries[cursor[hash_cell(entry_cell[i])]++] = static_cast<int32_t>(i);
	}
}

} // namespace cable
