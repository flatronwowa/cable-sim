#pragma once

#include <cstddef>
#include <vector>

namespace cable::v2 {

class SparseSystem {
public:
	bool analyze_pattern(int p_size, int p_half_bandwidth);
	void clear_numeric();
	void add(int p_row, int p_column, double p_value);
	bool factorize(double p_diagonal_shift);
	bool solve(const std::vector<double> &p_rhs, std::vector<double> &r_solution);

	int size() const { return matrix_size; }
	double max_abs_diagonal() const;
	size_t capacity_growths() const { return growths; }
	size_t peak_capacity_bytes() const { return peak_bytes; }

private:
	size_t index(int p_row, int p_column) const;
	double get_factor(int p_row, int p_column) const;
	void update_peak_capacity();

	std::vector<double> coefficients;
	std::vector<double> factors;
	std::vector<double> diagonal;
	std::vector<double> workspace;
	int matrix_size = 0;
	int half_bandwidth = 0;
	size_t growths = 0;
	size_t peak_bytes = 0;
};

} // namespace cable::v2
