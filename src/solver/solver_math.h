#pragma once

#include <cmath>
#include <cstdint>

namespace cable {

// Эпсилоны именованные и явные: деления без проверки знаменателя в солвере нет.
constexpr float EPS_LENGTH = 1.0e-6f; // вырожденный сегмент, distance constraint пропускается
constexpr float EPS_NORMAL = 1.0e-7f; // вырожденная нормаль контакта
constexpr float EPS_DENOM = 1.0e-12f; // знаменатель XPBD
constexpr float EPS_ANGLE = 1.0e-4f; // theta -> 0, C -> 0, bending пропускается

constexpr float PI_F = 3.14159265358979323846f;

inline float clampf(float v, float lo, float hi) {
	return v < lo ? lo : (v > hi ? hi : v);
}

inline float minf(float a, float b) {
	return a < b ? a : b;
}

inline float maxf(float a, float b) {
	return a > b ? a : b;
}

struct Vec3 {
	float x = 0.0f;
	float y = 0.0f;
	float z = 0.0f;

	Vec3() = default;
	Vec3(float p_x, float p_y, float p_z) : x(p_x), y(p_y), z(p_z) {}

	Vec3 operator+(const Vec3 &o) const { return Vec3(x + o.x, y + o.y, z + o.z); }
	Vec3 operator-(const Vec3 &o) const { return Vec3(x - o.x, y - o.y, z - o.z); }
	Vec3 operator-() const { return Vec3(-x, -y, -z); }
	Vec3 operator*(float s) const { return Vec3(x * s, y * s, z * s); }
	Vec3 &operator+=(const Vec3 &o) {
		x += o.x;
		y += o.y;
		z += o.z;
		return *this;
	}
	Vec3 &operator-=(const Vec3 &o) {
		x -= o.x;
		y -= o.y;
		z -= o.z;
		return *this;
	}
	Vec3 &operator*=(float s) {
		x *= s;
		y *= s;
		z *= s;
		return *this;
	}
};

inline float dot(const Vec3 &a, const Vec3 &b) {
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline Vec3 cross(const Vec3 &a, const Vec3 &b) {
	return Vec3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}

inline float length_squared(const Vec3 &v) {
	return dot(v, v);
}

inline float length(const Vec3 &v) {
	return std::sqrt(dot(v, v));
}

inline float distance(const Vec3 &a, const Vec3 &b) {
	return length(b - a);
}

// Возвращает нулевой вектор, если нормализовать нельзя: вызывающий обязан это проверить.
inline Vec3 safe_normalize(const Vec3 &v, float p_eps = EPS_LENGTH) {
	const float len = length(v);
	if (len < p_eps) {
		return Vec3();
	}
	const float inv = 1.0f / len;
	return Vec3(v.x * inv, v.y * inv, v.z * inv);
}

inline bool is_finite(const Vec3 &v) {
	return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

inline Vec3 lerp(const Vec3 &a, const Vec3 &b, float t) {
	return Vec3(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t);
}

inline Vec3 vec_min(const Vec3 &a, const Vec3 &b) {
	return Vec3(minf(a.x, b.x), minf(a.y, b.y), minf(a.z, b.z));
}

inline Vec3 vec_max(const Vec3 &a, const Vec3 &b) {
	return Vec3(maxf(a.x, b.x), maxf(a.y, b.y), maxf(a.z, b.z));
}

inline Vec3 vec_abs(const Vec3 &v) {
	return Vec3(std::fabs(v.x), std::fabs(v.y), std::fabs(v.z));
}

// Любой единичный вектор, ортогональный v. Используется для первого репера и для
// восстановления оси изгиба в вырожденных конфигурациях.
inline Vec3 any_perpendicular(const Vec3 &v) {
	const Vec3 axis = (std::fabs(v.x) < 0.9f) ? Vec3(1.0f, 0.0f, 0.0f) : Vec3(0.0f, 1.0f, 0.0f);
	const Vec3 p = safe_normalize(cross(v, axis));
	return length_squared(p) > 0.0f ? p : Vec3(0.0f, 0.0f, 1.0f);
}

// Матрица по столбцам: col[i] — образ i-го базисного вектора.
struct Mat3 {
	Vec3 col[3] = { Vec3(1.0f, 0.0f, 0.0f), Vec3(0.0f, 1.0f, 0.0f), Vec3(0.0f, 0.0f, 1.0f) };
};

inline Vec3 mat_xform(const Mat3 &m, const Vec3 &v) {
	return Vec3(
			m.col[0].x * v.x + m.col[1].x * v.y + m.col[2].x * v.z,
			m.col[0].y * v.x + m.col[1].y * v.y + m.col[2].y * v.z,
			m.col[0].z * v.x + m.col[1].z * v.y + m.col[2].z * v.z);
}

// Для ортонормированного базиса транспонирование и есть обращение.
inline Vec3 mat_xform_transposed(const Mat3 &m, const Vec3 &v) {
	return Vec3(dot(m.col[0], v), dot(m.col[1], v), dot(m.col[2], v));
}

struct Xform {
	Mat3 basis;
	Vec3 origin;
};

inline Vec3 xform_point(const Xform &t, const Vec3 &p) {
	return mat_xform(t.basis, p) + t.origin;
}

inline Vec3 xform_dir(const Xform &t, const Vec3 &d) {
	return mat_xform(t.basis, d);
}

// Обратное преобразование; корректно только для ортонормированного basis, что
// гарантируется на этапе построения снапшота.
inline Vec3 xform_inv_point(const Xform &t, const Vec3 &p) {
	return mat_xform_transposed(t.basis, p - t.origin);
}

inline Vec3 xform_inv_dir(const Xform &t, const Vec3 &d) {
	return mat_xform_transposed(t.basis, d);
}

struct Bounds {
	Vec3 min = Vec3(1e30f, 1e30f, 1e30f);
	Vec3 max = Vec3(-1e30f, -1e30f, -1e30f);

	void reset() {
		min = Vec3(1e30f, 1e30f, 1e30f);
		max = Vec3(-1e30f, -1e30f, -1e30f);
	}

	bool is_valid() const { return min.x <= max.x; }

	void expand(const Vec3 &p) {
		min = vec_min(min, p);
		max = vec_max(max, p);
	}

	void grow(float m) {
		min -= Vec3(m, m, m);
		max += Vec3(m, m, m);
	}

	bool overlaps(const Bounds &o) const {
		return min.x <= o.max.x && max.x >= o.min.x &&
				min.y <= o.max.y && max.y >= o.min.y &&
				min.z <= o.max.z && max.z >= o.min.z;
	}
};

// Параметр ближайшей к q точки отрезка [a, b], зажатый в [0, 1].
inline float closest_param_on_segment(const Vec3 &a, const Vec3 &b, const Vec3 &q) {
	const Vec3 ab = b - a;
	const float denom = length_squared(ab);
	if (denom < EPS_DENOM) {
		return 0.0f;
	}
	return clampf(dot(q - a, ab) / denom, 0.0f, 1.0f);
}

} // namespace cable
