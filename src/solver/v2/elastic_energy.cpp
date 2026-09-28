#include "solver/v2/elastic_energy.h"

#include <cmath>

namespace cable::v2 {

namespace {

constexpr int LOCAL_DOF = 9;
constexpr double ENERGY_EPSILON = 1.0e-12;

struct Jet {
	double value = 0.0;
	double gradient[LOCAL_DOF] = {};
	double hessian[LOCAL_DOF * LOCAL_DOF] = {};

	Jet() = default;
	explicit Jet(double p_value) : value(p_value) {}

	static Jet variable(double p_value, int p_index) {
		Jet result(p_value);
		result.gradient[p_index] = 1.0;
		return result;
	}
};

Jet operator+(const Jet &p_a, const Jet &p_b) {
	Jet result(p_a.value + p_b.value);
	for (int i = 0; i < LOCAL_DOF; ++i) {
		result.gradient[i] = p_a.gradient[i] + p_b.gradient[i];
	}
	for (int i = 0; i < LOCAL_DOF * LOCAL_DOF; ++i) {
		result.hessian[i] = p_a.hessian[i] + p_b.hessian[i];
	}
	return result;
}

Jet operator-(const Jet &p_a, const Jet &p_b) {
	Jet result(p_a.value - p_b.value);
	for (int i = 0; i < LOCAL_DOF; ++i) {
		result.gradient[i] = p_a.gradient[i] - p_b.gradient[i];
	}
	for (int i = 0; i < LOCAL_DOF * LOCAL_DOF; ++i) {
		result.hessian[i] = p_a.hessian[i] - p_b.hessian[i];
	}
	return result;
}

Jet operator*(const Jet &p_a, const Jet &p_b) {
	Jet result(p_a.value * p_b.value);
	for (int row = 0; row < LOCAL_DOF; ++row) {
		result.gradient[row] =
				p_a.gradient[row] * p_b.value + p_a.value * p_b.gradient[row];
		for (int column = 0; column < LOCAL_DOF; ++column) {
			const int index = row * LOCAL_DOF + column;
			result.hessian[index] =
					p_a.hessian[index] * p_b.value +
					p_a.gradient[row] * p_b.gradient[column] +
					p_a.gradient[column] * p_b.gradient[row] +
					p_a.value * p_b.hessian[index];
		}
	}
	return result;
}

Jet operator*(double p_scale, const Jet &p_value) {
	Jet result(p_scale * p_value.value);
	for (int i = 0; i < LOCAL_DOF; ++i) {
		result.gradient[i] = p_scale * p_value.gradient[i];
	}
	for (int i = 0; i < LOCAL_DOF * LOCAL_DOF; ++i) {
		result.hessian[i] = p_scale * p_value.hessian[i];
	}
	return result;
}

Jet unary(const Jet &p_value, double p_result, double p_first, double p_second) {
	Jet result(p_result);
	for (int row = 0; row < LOCAL_DOF; ++row) {
		result.gradient[row] = p_first * p_value.gradient[row];
		for (int column = 0; column < LOCAL_DOF; ++column) {
			const int index = row * LOCAL_DOF + column;
			result.hessian[index] =
					p_first * p_value.hessian[index] +
					p_second * p_value.gradient[row] * p_value.gradient[column];
		}
	}
	return result;
}

Jet reciprocal(const Jet &p_value) {
	const double inverse = 1.0 / p_value.value;
	return unary(p_value, inverse, -inverse * inverse, 2.0 * inverse * inverse * inverse);
}

Jet operator/(const Jet &p_numerator, const Jet &p_denominator) {
	return p_numerator * reciprocal(p_denominator);
}

Jet jet_sqrt(const Jet &p_value) {
	const double root = std::sqrt(p_value.value);
	return unary(p_value, root, 0.5 / root, -0.25 / (p_value.value * root));
}

struct JetVec3 {
	Jet x;
	Jet y;
	Jet z;
};

JetVec3 make_point(const ImplicitVec3 &p_value, int p_first_dof) {
	return {
		Jet::variable(p_value.x, p_first_dof),
		Jet::variable(p_value.y, p_first_dof + 1),
		Jet::variable(p_value.z, p_first_dof + 2),
	};
}

JetVec3 operator-(const JetVec3 &p_a, const JetVec3 &p_b) {
	return { p_a.x - p_b.x, p_a.y - p_b.y, p_a.z - p_b.z };
}

Jet dot_jet(const JetVec3 &p_a, const JetVec3 &p_b) {
	return p_a.x * p_b.x + p_a.y * p_b.y + p_a.z * p_b.z;
}

LocalEnergy to_local_energy(const Jet &p_energy, int p_dof_count) {
	LocalEnergy result;
	result.energy = p_energy.value;
	result.dof_count = p_dof_count;
	for (int row = 0; row < p_dof_count; ++row) {
		result.gradient[row] = p_energy.gradient[row];
		for (int column = 0; column < p_dof_count; ++column) {
			result.hessian[row * LOCAL_DOF + column] =
					p_energy.hessian[row * LOCAL_DOF + column];
		}
	}
	return result;
}

} // namespace

LocalEnergy evaluate_stretch(const ImplicitVec3 &p_a, const ImplicitVec3 &p_b, double p_rest_length,
		double p_axial_stiffness) {
	LocalEnergy result;
	result.dof_count = 6;
	if (p_rest_length <= ENERGY_EPSILON || p_axial_stiffness < 0.0) {
		result.valid = false;
		return result;
	}

	const double edge[3] = {
		p_b.x - p_a.x,
		p_b.y - p_a.y,
		p_b.z - p_a.z,
	};
	const double edge_length =
			std::sqrt(edge[0] * edge[0] + edge[1] * edge[1] + edge[2] * edge[2]);
	if (edge_length <= ENERGY_EPSILON) {
		result.valid = false;
		return result;
	}

	const double stiffness = p_axial_stiffness / p_rest_length;
	const double extension = edge_length - p_rest_length;
	const double tangent_scale = extension / edge_length;
	result.energy = 0.5 * stiffness * extension * extension;
	for (int row = 0; row < 3; ++row) {
		const double direction_row = edge[row] / edge_length;
		const double gradient = stiffness * extension * direction_row;
		result.gradient[row] = -gradient;
		result.gradient[row + 3] = gradient;
		for (int column = 0; column < 3; ++column) {
			const double direction_column = edge[column] / edge_length;
			const double identity = row == column ? 1.0 : 0.0;
			const double block = stiffness *
					(direction_row * direction_column +
							tangent_scale * (identity - direction_row * direction_column));
			result.hessian[row * 9 + column] = block;
			result.hessian[row * 9 + column + 3] = -block;
			result.hessian[(row + 3) * 9 + column] = -block;
			result.hessian[(row + 3) * 9 + column + 3] = block;
		}
	}
	return result;
}

LocalEnergy evaluate_bending(const ImplicitVec3 &p_a, const ImplicitVec3 &p_b, const ImplicitVec3 &p_c,
		double p_rest_voronoi_length, double p_bending_stiffness) {
	if (p_rest_voronoi_length <= ENERGY_EPSILON || p_bending_stiffness < 0.0) {
		LocalEnergy result;
		result.valid = false;
		return result;
	}

	const JetVec3 a = make_point(p_a, 0);
	const JetVec3 b = make_point(p_b, 3);
	const JetVec3 c = make_point(p_c, 6);
	const JetVec3 edge_before = b - a;
	const JetVec3 edge_after = c - b;
	const Jet before_squared = dot_jet(edge_before, edge_before);
	const Jet after_squared = dot_jet(edge_after, edge_after);
	if (before_squared.value <= ENERGY_EPSILON || after_squared.value <= ENERGY_EPSILON) {
		LocalEnergy result;
		result.valid = false;
		return result;
	}

	const Jet before_length = jet_sqrt(before_squared);
	const Jet after_length = jet_sqrt(after_squared);
	const Jet length_product = before_length * after_length;
	const Jet cosine = dot_jet(edge_before, edge_after) / length_product;
	if (1.0 + cosine.value <= ENERGY_EPSILON) {
		LocalEnergy result;
		result.valid = false;
		return result;
	}

	// Isotropic DER identity: ||2(e0 x e1)/(||e0||||e1|| + e0.e1)||²
	// equals 4(1 - cos(theta))/(1 + cos(theta)).
	const Jet energy =
			(2.0 * p_bending_stiffness / p_rest_voronoi_length) *
			((Jet(1.0) - cosine) / (Jet(1.0) + cosine));
	return to_local_energy(energy, 9);
}

} // namespace cable::v2
