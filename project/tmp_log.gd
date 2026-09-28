extends SceneTree

# Прогон настоящего контроллера стенда с записью лога: только так видно поведение,
# которое видит игрок, а не упрощённые условия синтетического замера.

var demo
var cable: CableBody3D

func _initialize() -> void:
	demo = load("res://tmp_demo.tscn").instantiate()
	root.add_child(demo)
	await process_frame
	cable = demo.cable
	if OS.has_environment("ITER"):
		cable.solver_iterations = int(OS.get_environment("ITER"))
	if OS.has_environment("SELF"):
		cable.self_collision_enabled = int(OS.get_environment("SELF")) != 0
	if OS.has_environment("SLEEP"):
		cable.sleep_enabled = int(OS.get_environment("SLEEP")) != 0
	if OS.has_environment("BEND"):
		if int(OS.get_environment("BEND")) == 0:
			cable.bending_compliance = 1000000.0
			cable.minimum_bend_radius = 0.0
			cable.anchor_stiffness_compliance = 1000000.0
	if OS.has_environment("ANCHOR"):
		cable.anchor_stiffness_compliance = float(OS.get_environment("ANCHOR"))
	if OS.has_environment("RADIUS") and int(OS.get_environment("RADIUS")) == 0:
		cable.minimum_bend_radius = 0.0
	if OS.has_environment("ELASTIC"):
		cable.bending_compliance = float(OS.get_environment("ELASTIC"))
	if OS.has_environment("DAMPING"):
		cable.damping = float(OS.get_environment("DAMPING"))

	var phases: Array = []
	# Третий элемент — вращение камеры за кадр, рад: осмотр по сторонам без ходьбы.
	# Стоим на месте: кабель висит от столба к руке, никакого ввода нет.
	phases.append(["стоим_смирно", Vector3.ZERO, 120, 0.0])
	phases.append(["осмотр", Vector3.ZERO, 120, 0.015])
	phases.append(["_x1", Vector3.ZERO, 1, 0.0])
	phases.append(["_x2", Vector3.ZERO, 1, 0.0])
	phases.append(["_x3", Vector3.ZERO, 1, 0.0])
	# Отходим по прямой, кабель волочится по полу.
	phases.append(["ход_прямо", Vector3(1, 0, 0), 150])
	phases.append(["покой_после_хода", Vector3.ZERO, 300])
	# Ведём кабель мимо ящика и колонн: он цепляется за предметы.
	phases.append(["_пропуск", Vector3.ZERO, 1])
	phases.append(["ход_к_колоннам", Vector3(0.3, 0, -1), 150])
	phases.append(["ход_вокруг", Vector3(1, 0, 0.4), 150])
	phases.append(["покой_после_зацепа", Vector3.ZERO, 400])

	demo.start_log("user://cable_log.csv")
	var marks: Array = []
	for phase in phases:
		marks.append([phase[0], demo.log_frame + 1])
		demo.scripted_input = phase[1]
		var spin: float = phase[3] if phase.size() > 3 else 0.0
		for i in int(phase[2]):
			if spin != 0.0:
				demo.yaw_node.rotate_y(spin * (1.0 if (i / 60) % 2 == 0 else -1.0))
			await process_frame
			await physics_frame

	demo.log_file.flush()
	print("лог: ", ProjectSettings.globalize_path("user://cable_log.csv"))
	for m in marks:
		print("фаза %s с кадра %d" % [m[0], m[1]])
	quit()
