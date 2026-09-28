extends SceneTree

var cable: CableBody3D
var frames := 0

func _initialize() -> void:
	var r := Node3D.new()
	root.add_child(r)

	# Якорь в воздухе, кабель выходит горизонтально вдоль +X (-Z базиса повёрнут).
	var anchor := Node3D.new()
	anchor.position = Vector3(0, 3, 0)
	anchor.rotation = Vector3(0, -PI / 2.0, 0)
	r.add_child(anchor)

	cable = CableBody3D.new()
	cable.cable_length = 1.5
	cable.cable_radius = 0.02
	cable.target_segment_length = 0.05
	cable.mass_per_meter = 0.25
	cable.bending_compliance = float(OS.get_environment("BC")) if OS.get_environment("BC") != "" else 0.001
	cable.minimum_bend_radius = float(OS.get_environment("MBR")) if OS.get_environment("MBR") != "" else 0.15
	cable.anchor_stiffness_compliance = 0.0
	if OS.get_environment("ITER") != "":
		cable.solver_iterations = int(OS.get_environment("ITER"))
	r.add_child(cable)
	cable.anchor_a = cable.get_path_to(anchor)
	cable.rebuild_cable()

func _process(_d: float) -> bool:
	return false

func _physics_process(_d: float) -> bool:
	frames += 1
	if frames == 400:
		var line := cable.get_centerline()
		var tip: Vector3 = line[line.size() - 1]
		var base: Vector3 = line[0]
		var poly := 0.0
		var bad := 0
		for i in range(line.size() - 1):
			poly += line[i].distance_to(line[i + 1])
			if not is_finite(line[i].y):
				bad += 1
		print("bc=%.6f it=%d reach=%.3f drop=%.3f reach/L=%.0f%% poly=%.3f err=%.2f%% nan=%d tens=%.0f" % [
			cable.bending_compliance, cable.solver_iterations, absf(tip.x - base.x), base.y - tip.y,
			100.0 * absf(tip.x - base.x) / poly, poly,
			100.0 * (poly - cable.get_deployed_length()) / cable.get_deployed_length(),
			bad, cable.get_tension()])
		return true
	return false
