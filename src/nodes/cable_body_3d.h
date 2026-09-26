#pragma once

#include <godot_cpp/classes/node3d.hpp>

namespace godot {

class CableBody3D : public Node3D {
	GDCLASS(CableBody3D, Node3D)

protected:
	static void _bind_methods();

public:
	void set_cable_length(double p_length);
	double get_cable_length() const;

private:
	double cable_length = 1.0;
};

} // namespace godot
