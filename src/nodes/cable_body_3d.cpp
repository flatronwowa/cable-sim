#include "cable_body_3d.h"

#include <godot_cpp/core/class_db.hpp>

using namespace godot;

void CableBody3D::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_cable_length", "length"), &CableBody3D::set_cable_length);
	ClassDB::bind_method(D_METHOD("get_cable_length"), &CableBody3D::get_cable_length);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "cable_length", PROPERTY_HINT_RANGE, "0.01,100.0,0.01,or_greater"), "set_cable_length", "get_cable_length");
}

void CableBody3D::set_cable_length(double p_length) {
	cable_length = p_length;
}

double CableBody3D::get_cable_length() const {
	return cable_length;
}
