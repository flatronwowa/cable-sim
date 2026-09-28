extends SceneTree

var demo: Node3D
var frames := 0
var start := Vector3.ZERO

func _initialize() -> void:
	demo = load("res://tmp_demo.tscn").instantiate()
	root.add_child(demo)

func _process(_d: float) -> bool:
	return false

func _physics_process(delta: float) -> bool:
	frames += 1
	var p: CharacterBody3D = demo.player
	var c: CableBody3D = demo.cable
	if frames == 10:
		p.global_position = Vector3(1.2, 1.0, 0.0)
		start = p.global_position

	# Имитируем удержание W: та же логика, что в демо, но без Input.
	if frames > 10:
		var dir := Vector3(1, 0, 0)
		dir.y = 0.0
		dir = dir.normalized()
		p.velocity.x = dir.x * 3.0
		p.velocity.z = dir.z * 3.0
		p.velocity.y -= 18.0 * delta
		var from := p.global_position
		var wanted := from + p.velocity * delta
		var allowed: Vector3 = c.constrain_motion(from, wanted)
		var blocked := (wanted - allowed).length() > 1e-5
		p.velocity = (allowed - from) / delta
		demo.try_step_up(Vector3(p.velocity.x, 0.0, p.velocity.z))
		p.move_and_slide()
		if frames % 30 == 0:
			print("f=%4d pos=(%.2f,%.2f,%.2f) floor=%s blocked=%s rem=%.1f n=%d" % [
				frames, p.global_position.x, p.global_position.y, p.global_position.z,
				str(p.is_on_floor()), str(blocked),
				c.get_remaining_length(), c.get_particle_count()])
	if frames >= 600:
		return true
	return false
