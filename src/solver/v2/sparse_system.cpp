#include "solver/v2/sparse_system.h"

#include <algorithm>
#include <cmath>

namespace cable::v2 {

namespace {

void resize_tracked(std::vector<double> &p_values, size_t p_size, size_t &r_growths) {
	const size_t old_capacity = p_values.capacity();
	p_values.resize(p_size);
	if (p_values.capacity() != old_capacity) {
		++r_growths;
	}
}

} // namespace

bool SparseSystem::analyze_pattern(int p_size, int p_half_bandwidth) {
	if (p_size < 0 || p_half_bandwidth < 0) {
		return false;
	}
	matrix_size = p_size;
	half_bandwidth = p_half_bandwidth;
	const size_t storage = static_cast<size_t>(matrix_size) * static_cast<size_t>(half_bandwidth + 1);
	resize_tracked(coefficients, storage, growths);
	resize_tracked(factors, storage, growths);
	resize_tracked(diagonal, static_cast<size_t>(matrix_size), growths);
	resize_tracked(workspace, static_cast<size_t>(matrix_size), growths);
	update_peak_capacity();
	return true;
}

void SparseSystem::clear_numeric() {
	std::fill(coefficients.begin(), coefficients.end(), 0.0);
}

size_t SparseSystem::index(int p_row, int p_column) const {
	return static_cast<size_t>(p_row) * static_cast<size_t>(half_bandwidth + 1) +
			static_cast<size_t>(p_row - p_column);
}

void SparseSystem::add(int p_row, int p_column, double p_value) {
	if (p_column > p_row) {
		std::swap(p_row, p_column);
	}
	if (p_row < 0 || p_row >= matrix_size || p_column < 0 ||
			p_row - p_column > half_bandwidth) {
		return;
	}
	coefficients[index(p_row, p_column)] += p_value;
}

double SparseSystem::get_factor(int p_row, int p_column) const {
	if (p_column > p_row || p_row - p_column > half_bandwidth) {
		return 0.0;
	}
	return factors[index(p_row, p_column)];
}

double SparseSystem::max_abs_diagonal() const {
	double maximum = 0.0;
	for (int row = 0; row < matrix_size; ++row) {
		maximum = std::max(maximum, std::fabs(coefficients[index(row, row)]));
	}
	return maximum;
}

bool SparseSystem::factorize(double p_diagonal_shift) {
	std::fill(factors.begin(), factors.end(), 0.0);
	for (int row = 0; row < matrix_size; ++row) {
		const int first_column = std::max(0, row - half_bandwidth);
		for (int column = first_column; column < row; ++column) {
			double value = coefficients[index(row, column)];
			const int first_shared = std::max(first_column, column - half_bandwidth);
			for (int k = first_shared; k < column; ++k) {
				value -= get_factor(row, k) * diagonal[k] * get_factor(column, k);
			}
			if (!std::isfinite(diagonal[column]) || diagonal[column] <= 0.0) {
				return false;
			}
			factors[index(row, column)] = value / diagonal[column];
		}

		double pivot = coefficients[index(row, row)] + p_diagonal_shift;
		for (int k = first_column; k < row; ++k) {
			const double lower = get_factor(row, k);
			pivot -= lower * lower * diagonal[k];
		}
		if (!std::isfinite(pivot) || pivot <= 1.0e-12) {
			return false;
		}
		diagonal[row] = pivot;
		factors[index(row, row)] = 1.0;
	}
	return true;
}

bool SparseSystem::solve(const std::vector<double> &p_rhs, std::vector<double> &r_solution) {
	if (static_cast<int>(p_rhs.size()) != matrix_size) {
		return false;
	}
	const size_t old_capacity = r_solution.capacity();
	r_solution.resize(static_cast<size_t>(matrix_size));
	if (r_solution.capacity() != old_capacity) {
		++growths;
	}

	for (int row = 0; row < matrix_size; ++row) {
		double value = p_rhs[row];
		const int first_column = std::max(0, row - half_bandwidth);
		for (int column = first_column; column < row; ++column) {
			value -= get_factor(row, column) * workspace[column];
		}
		workspace[row] = value;
	}
	for (int row = 0; row < matrix_size; ++row) {
		workspace[row] /= diagonal[row];
	}
	for (int row = matrix_size - 1; row >= 0; --row) {
		double value = workspace[row];
		const int last = std::min(matrix_size - 1, row + half_bandwidth);
		for (int lower_row = row + 1; lower_row <= last; ++lower_row) {
			value -= get_factor(lower_row, row) * r_solution[lower_row];
		}
		r_solution[row] = value;
	}
	update_peak_capacity();
	return true;
}

void SparseSystem::update_peak_capacity() {
	const size_t bytes = (coefficients.capacity() + factors.capacity() + diagonal.capacity() +
								 workspace.capacity()) *
			sizeof(double);
	if (bytes > peak_bytes) {
		peak_bytes = bytes;
	}
}

} // namespace cable::v2
