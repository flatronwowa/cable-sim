extends SceneTree

func _initialize() -> void:
	var demo = load("res://tmp_demo.tscn").instantiate()
	root.add_child(demo)
	await process_frame
	await process_frame

	print("mouse_mode=", Input.mouse_mode)
	print("yaw до=", demo.yaw_node.rotation.y, " pitch до=", demo.pitch_node.rotation.x)

	var ev := InputEventMouseMotion.new()
	ev.relative = Vector2(100, 40)
	Input.parse_input_event(ev)
	await process_frame
	print("через parse_input_event: yaw=", demo.yaw_node.rotation.y, " pitch=", demo.pitch_node.rotation.x)

	demo._input(ev)
	print("прямым вызовом: yaw=", demo.yaw_node.rotation.y, " pitch=", demo.pitch_node.rotation.x)
	quit()
