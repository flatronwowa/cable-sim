extends SceneTree

var cable: CableBody3D
var anchor: Node3D
var player: Node3D
var frames := 0

func add_static(shape: Shape3D, xform: Transform3D) -> StaticBody3D:
	var b := StaticBody3D.new()
	var cs := CollisionShape3D.new()
	cs.shape = shape
	b.add_child(cs)
	b.transform = xform
	return b

func _initialize() -> void:
	var r := Node3D.new()
	root.add_child(r)
	var f := BoxShape3D.new()
	f.size = Vector3(60, 1, 60)
	r.add_child(add_static(f, Transform3D(Basis(), Vector3(0, -0.5, 0))))
	var crate := BoxShape3D.new()
	crate.size = Vector3(0.9, 0.9, 0.9)
	r.add_child(add_static(crate, Transform3D(Basis(), Vector3(4, 0.45, 0))))

	anchor = Node3D.new()
	anchor.position = Vector3(0, 1.8, 0)
	r.add_child(anchor)
	player = Node3D.new()
	player.position = Vector3(1.0, 1.0, 0)
	r.add_child(player)

	cable = CableBody3D.new()
	cable.cable_length = 16.0
	cable.cable_radius = 0.02
	cable.target_segment_length = 0.07
	cable.mass_per_meter = 0.25
	cable.bending_compliance = float(OS.get_environment("BC")) if OS.get_environment("BC") != "" else 0.001
	cable.minimum_bend_radius = float(OS.get_environment("MBR")) if OS.get_environment("MBR") != "" else 0.06
	if OS.get_environment("ITER") != "":
		cable.solver_iterations = int(OS.get_environment("ITER"))
	r.add_child(cable)
	cable.anchor_a = cable.get_path_to(anchor)
	cable.anchor_b = cable.get_path_to(player)
	cable.rebuild_cable()

func _process(_d: float) -> bool:
	return false

func _physics_process(_d: float) -> bool:
	frames += 1
	if frames > 20 and frames < 320:
		player.position += Vector3(0.03, 0, 0)

	if frames == 320 or frames == 560:
		var line := cable.get_centerline()
		var poly := 0.0
		var turn_sum := 0.0
		var turn_max := 0.0
		var bad := 0
		for i in range(line.size() - 1):
			poly += line[i].distance_to(line[i + 1])
			if not is_finite(line[i].x):
				bad += 1
			if i > 0:
				var a: Vector3 = (line[i] - line[i - 1]).normalized()
				var b: Vector3 = (line[i + 1] - line[i]).normalized()
				var ang := rad_to_deg(a.angle_to(b))
				turn_sum += ang
				turn_max = maxf(turn_max, ang)
		print("f=%d bc=%.6f mbr=%.3f n=%d dep=%.2f poly=%.2f err=%.2f%% turn_avg=%.1f turn_max=%.1f nan=%d tens=%.0f" % [
			frames, cable.bending_compliance, cable.minimum_bend_radius, line.size(),
			cable.get_deployed_length(), poly,
			100.0 * (poly - cable.get_deployed_length()) / maxf(cable.get_deployed_length(), 0.001),
			turn_sum / maxf(line.size() - 2, 1), turn_max, bad, cable.get_tension()])
	if frames >= 560:
		return true
	return false
