extends SceneTree

# Что именно происходит в месте скачка: положение участка до и после возмущения.

var demo
var cable: CableBody3D
var player: CharacterBody3D

func _initialize() -> void:
	demo = load("res://tmp_demo.tscn").instantiate()
	root.add_child(demo)
	await process_frame
	cable = demo.cable
	player = demo.player
	demo.set_physics_process(false)

	# Идём через лестницу блоков: кабель перекидывается через рёбра, а не лежит
	# ровно на полу. Именно в таком состоянии и возникал скачок.
	player.global_position = Vector3(1.2, 1.0, 0.0)
	for i in 200:
		player.velocity = Vector3(3, -1, 0)
		demo.try_step_up(Vector3(3, 0, 0))
		player.move_and_slide()
		await step()
	for i in 200:
		player.velocity = Vector3.ZERO
		await step()

	var before := cable.get_centerline()
	print("состояние покоя: частиц %d, размотано %.2f м" % [before.size(), cable.get_deployed_length()])
	print("профиль высоты кабеля (где перегибы через рёбра):")
	for i in range(0, before.size(), 6):
		if i < before.size():
			print("  %3d  (%6.2f, %6.2f, %6.2f)" % [i, before[i].x, before[i].y, before[i].z])

	player.global_position += Vector3(0.02, 0, 0)
	await step()
	var after := cable.get_centerline()
	if after.size() != before.size():
		print("число частиц изменилось: %d -> %d" % [before.size(), after.size()])
		quit()
		return

	print("\nсмещения за кадр рывка, только больше 2 мм:")
	for i in after.size():
		var d: float = before[i].distance_to(after[i])
		if d > 0.002:
			var mv: Vector3 = after[i] - before[i]
			print("  %3d  смещение %.4f м  направление (%5.2f, %5.2f, %5.2f)  y было %.3f стало %.3f" % [
				i, d, mv.normalized().x, mv.normalized().y, mv.normalized().z, before[i].y, after[i].y])
	quit()

func step() -> void:
	await process_frame
	await physics_frame
