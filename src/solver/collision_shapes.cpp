#include "solver/collision_shapes.h"

namespace cable {

static inline float sign_or_positive(float v) {
	return v < 0.0f ? -1.0f : 1.0f;
}

static ShapeSample sample_sphere(float p_radius, const Vec3 &p_local) {
	ShapeSample s;
	const float len = length(p_local);
	if (len < EPS_NORMAL) {
		// Точка ровно в центре: нормали нет, вызывающий пометит частицу и возьмёт
		// направление движения на следующем подшаге.
		s.distance = -p_radius;
		return s;
	}
	s.distance = len - p_radius;
	s.gradient = p_local * (1.0f / len);
	s.valid = true;
	return s;
}

static ShapeSample sample_box(const Vec3 &p_half, const Vec3 &p_local) {
	ShapeSample s;
	const Vec3 q = vec_abs(p_local) - p_half;
	const Vec3 outside = vec_max(q, Vec3());
	const float max_q = maxf(q.x, maxf(q.y, q.z));

	if (max_q > 0.0f) {
		// Точное знаковое расстояние до бокса снаружи.
		s.distance = length(outside);
		s.gradient = safe_normalize(Vec3(
				outside.x * sign_or_positive(p_local.x),
				outside.y * sign_or_positive(p_local.y),
				outside.z * sign_or_positive(p_local.z)));
		s.valid = length_squared(s.gradient) > 0.0f;
		return s;
	}

	// Внутри: выход через наименее глубокую грань.
	int axis = 0;
	if (q.y > q.x) {
		axis = 1;
	}
	if (q.z > (axis == 0 ? q.x : q.y)) {
		axis = 2;
	}
	s.distance = max_q;
	Vec3 g;
	if (axis == 0) {
		g = Vec3(sign_or_positive(p_local.x), 0.0f, 0.0f);
	} else if (axis == 1) {
		g = Vec3(0.0f, sign_or_positive(p_local.y), 0.0f);
	} else {
		g = Vec3(0.0f, 0.0f, sign_or_positive(p_local.z));
	}
	s.gradient = g;
	s.valid = true;
	return s;
}

static ShapeSample sample_capsule(float p_radius, float p_height, const Vec3 &p_local) {
	// Godot задаёт height как полную высоту с полусферами, ось Y.
	const float half_core = maxf(p_height * 0.5f - p_radius, 0.0f);
	Vec3 q = p_local;
	q.y -= clampf(p_local.y, -half_core, half_core);
	return sample_sphere(p_radius, q);
}

static ShapeSample sample_cylinder(float p_radius, float p_height, const Vec3 &p_local) {
	ShapeSample s;
	const float half_height = p_height * 0.5f;
	const float radial_len = std::sqrt(p_local.x * p_local.x + p_local.z * p_local.z);
	const float d_radial = radial_len - p_radius;
	const float d_axial = std::fabs(p_local.y) - half_height;
	const Vec3 axial_dir = Vec3(0.0f, sign_or_positive(p_local.y), 0.0f);
	const Vec3 radial_dir = radial_len > EPS_NORMAL
			? Vec3(p_local.x / radial_len, 0.0f, p_local.z / radial_len)
			: Vec3();

	if (d_radial > 0.0f || d_axial > 0.0f) {
		const float ox = maxf(d_radial, 0.0f);
		const float oy = maxf(d_axial, 0.0f);
		s.distance = std::sqrt(ox * ox + oy * oy);
		s.gradient = safe_normalize(radial_dir * ox + axial_dir * oy);
		s.valid = length_squared(s.gradient) > 0.0f;
		return s;
	}

	if (d_radial > d_axial && length_squared(radial_dir) > 0.0f) {
		s.distance = d_radial;
		s.gradient = radial_dir;
	} else {
		s.distance = d_axial;
		s.gradient = axial_dir;
	}
	s.valid = true;
	return s;
}

// Максимум по опорным плоскостям. Точно внутри выпуклой оболочки; снаружи у рёбер
// и вершин расстояние занижено, поэтому контакт срабатывает чуть раньше, а
// направление выталкивания совпадает с нормалью ближайшей грани.
static ShapeSample sample_convex(const ConvexPlane *p_planes, uint32_t p_count, const Vec3 &p_local) {
	ShapeSample s;
	if (p_count == 0) {
		return s;
	}
	float best = -1.0e30f;
	Vec3 best_normal;
	for (uint32_t i = 0; i < p_count; ++i) {
		const float d = dot(p_planes[i].normal, p_local) - p_planes[i].d;
		if (d > best) {
			best = d;
			best_normal = p_planes[i].normal;
		}
	}
	s.distance = best;
	s.gradient = best_normal;
	s.valid = length_squared(best_normal) > 0.0f;
	return s;
}

ShapeSample sample_primitive(const ShapeData &p_shape, const ConvexPlane *p_planes, const Vec3 &p_point) {
	if (p_shape.type == ShapeType::PLANE) {
		ShapeSample s;
		s.distance = dot(p_shape.plane_normal, p_point) - p_shape.plane_d;
		s.gradient = p_shape.plane_normal;
		s.valid = true;
		return s;
	}

	const Vec3 local = xform_inv_point(p_shape.transform, p_point);
	ShapeSample s;
	switch (p_shape.type) {
		case ShapeType::SPHERE:
			s = sample_sphere(p_shape.radius, local);
			break;
		case ShapeType::BOX:
			s = sample_box(p_shape.half_extents, local);
			break;
		case ShapeType::CAPSULE:
			s = sample_capsule(p_shape.radius, p_shape.height, local);
			break;
		case ShapeType::CYLINDER:
			s = sample_cylinder(p_shape.radius, p_shape.height, local);
			break;
		case ShapeType::CONVEX:
			s = sample_convex(p_planes + p_shape.convex_plane_first, p_shape.convex_plane_count, local);
			break;
		default:
			return s;
	}
	if (s.valid) {
		s.gradient = xform_dir(p_shape.transform, s.gradient);
	}
	return s;
}

} // namespace cable
