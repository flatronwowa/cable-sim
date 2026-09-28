extends SceneTree

# Приоритет 1: кабель не проходит сквозь геометрию. Пол лежит верхом на y=0,
# поэтому минимальная высота центральной линии не должна опускаться ниже
# cable_radius + collision_margin.

func _initialize() -> void:
	var demo = load("res://tmp_demo.tscn").instantiate()
	root.add_child(demo)
	await process_frame
	var cable: CableBody3D = demo.cable
	var player: CharacterBody3D = demo.player
	demo.set_physics_process(false)

	var floor_limit: float = cable.cable_radius + cable.collision_margin
	var min_y := 1e9
	var worst_frame := 0

	# Обход препятствий: по лестнице блоков, мимо колонн, вокруг шара, обратно.
	var route := [Vector3(1, 0, 0), Vector3(0, 0, -1), Vector3(-1, 0, 0), Vector3(0, 0, 1)]
	var frame := 0
	for leg in route:
		for i in 150:
			player.velocity = leg * 5.0
			player.velocity.y = -2.0
			demo.try_step_up(leg * 5.0)
			player.move_and_slide()
			await process_frame
			await physics_frame
			frame += 1
			for p in cable.get_centerline():
				if p.y < min_y:
					min_y = p.y
					worst_frame = frame

	var line := cable.get_centerline()
	var poly := 0.0
	for i in range(1, line.size()):
		poly += line[i].distance_to(line[i - 1])

	print("порог по полу=%.4f, минимум центральной линии=%.4f (кадр %d) -> %s" % [
		floor_limit, min_y, worst_frame, "ок" if min_y >= floor_limit - 1e-4 else "ПРОНИКНОВЕНИЕ"])
	print("размотано=%.2f м, ломаная=%.2f м, ошибка длины=%.2f%%, частиц=%d, натяжение=%.1f Н" % [
		cable.get_deployed_length(), poly,
		100.0 * absf(poly - cable.get_deployed_length()) / maxf(cable.get_deployed_length(), 1e-6),
		cable.get_particle_count(), cable.get_tension()])
	quit()
