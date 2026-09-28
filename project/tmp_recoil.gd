extends SceneTree

# Замер отдачи: кабель укладывается на пол, затем рука смещается на 2 см,
# и по снимкам центральной линии считается профиль скорости по длине.

var demo
var cable: CableBody3D
var player: CharacterBody3D
var prev: PackedVector3Array

func _initialize() -> void:
	demo = load("res://tmp_demo.tscn").instantiate()
	root.add_child(demo)
	await process_frame
	cable = demo.cable
	player = demo.player
	demo.set_physics_process(false)

	cable.minimum_bend_radius = float(OS.get_environment("MBR")) if OS.has_environment("MBR") else 0.25
	cable.bending_compliance = float(OS.get_environment("BC")) if OS.has_environment("BC") else 0.0005
	if OS.has_environment("SELF"):
		cable.self_collision_enabled = OS.get_environment("SELF") == "1"
	if OS.has_environment("FRIC"):
		cable.static_friction = float(OS.get_environment("FRIC"))
		cable.dynamic_friction = float(OS.get_environment("FRIC"))
	if OS.has_environment("ANCH"):
		cable.anchor_stiffness_compliance = float(OS.get_environment("ANCH"))
	if OS.has_environment("SLEEP"):
		cable.sleep_enabled = OS.get_environment("SLEEP") == "1"
	if OS.has_environment("SUB"):
		cable.substeps_min = int(OS.get_environment("SUB"))
		cable.substeps_max = int(OS.get_environment("SUB"))
	if OS.has_environment("DAMP"):
		cable.damping = float(OS.get_environment("DAMP"))

	# Игрок отходит от столба, чтобы размотать кабель, затем всё успокаивается.
	await walk(Vector3(1, 0, 0), 3.0, 140)
	await settle(300)

	print("--- mbr=%.2f bc=%.5f, после укладки: n=%d, покой ниже ---" % [
		cable.minimum_bend_radius, cable.bending_compliance, cable.get_particle_count()])
	print("подшагов=", cable.get_substep_count())
	await report(4, Vector3.ZERO)
	print("--- рывок рукой на 2 см за один кадр ---")
	await report(8, Vector3(0.02, 0, 0))
	exit_angles()

	quit()

# Направление выхода задаётся базисом узла-якоря (кабель идёт вдоль -Z).
func exit_angles() -> void:
	var c := cable.get_centerline()
	var n := c.size()
	var post_axis: Vector3 = -cable.get_node(cable.anchor_a).global_transform.basis.z
	var hand_axis: Vector3 = -cable.get_node(cable.anchor_b).global_transform.basis.z
	var post_seg: Vector3 = (c[1] - c[0]).normalized()
	var hand_seg: Vector3 = (c[n - 2] - c[n - 1]).normalized()
	print("угол выхода: столб=%.1f град, рука=%.1f град" % [
		rad_to_deg(post_axis.angle_to(post_seg)), rad_to_deg(hand_axis.angle_to(hand_seg))])

func step() -> void:
	await process_frame
	await physics_frame

func walk(dir: Vector3, speed: float, frames: int) -> void:
	for i in frames:
		player.velocity = dir * speed
		player.velocity.y = -1.0
		player.move_and_slide()
		await step()

func settle(frames: int) -> void:
	for i in frames:
		player.velocity = Vector3.ZERO
		await step()

# Скорость частицы = смещение за кадр / dt; интересен профиль по длине, где
# индекс n-1 — рука игрока, индекс 0 — неподвижный столб.
func report(frames: int, nudge: Vector3) -> void:
	var dt := 1.0 / 60.0
	prev = cable.get_centerline()
	for f in frames:
		if f == 0 and nudge != Vector3.ZERO:
			player.global_position += nudge
		player.velocity = Vector3.ZERO
		await step()
		var now := cable.get_centerline()
		if now.size() != prev.size():
			prev = now
			continue
		var n := now.size()
		var vmax := 0.0
		var imax := 0
		var q := [0.0, 0.0, 0.0, 0.0] # четверти длины: 0 у столба, 3 у руки
		var sum := 0.0
		for i in n:
			var v: float = prev[i].distance_to(now[i]) / dt
			sum += v
			if v > vmax:
				vmax = v
				imax = i
			var qi: int = mini(i * 4 / n, 3)
			q[qi] = maxf(q[qi], v)
		print("f=%2d vmax=%5.2f @%3d/%d  по четвертям от столба к руке: %4.2f %4.2f %4.2f %4.2f  средняя=%4.2f" % [
			f, vmax, imax, n, q[0], q[1], q[2], q[3], sum / float(n)])
		prev = now
