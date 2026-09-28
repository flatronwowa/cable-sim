extends Node3D

const SPEED := 3.2
const RUN_SPEED := 6.5
const FLY_SPEED := 7.0
const MOVE_ACCEL := 18.0
const GRAVITY := 18.0
const JUMP := 4.5
const STEP_HEIGHT := 0.45
const MOUSE_SENS := 0.0025

var player: CharacterBody3D
var yaw_node: Node3D
var pitch_node: Node3D
var hand: Node3D
var cable: CableBody3D
var hud: Label
var toast: Label
var toast_time := 0.0
var mouse_captured := true
var flying := false
var scripted_input := Vector3.ZERO
var log_file: FileAccess
var log_frame := 0
var log_prev: PackedVector3Array

func _ready() -> void:
	build_environment()
	build_level()
	build_player()
	build_cable()
	build_hud()
	capture_mouse(true)
	if OS.has_environment("CABLE_LOG"):
		start_log("user://cable_manual_log.csv")

# --- сборка сцены ---

func build_environment() -> void:
	var env := Environment.new()
	env.background_mode = Environment.BG_SKY
	var sky := Sky.new()
	sky.sky_material = ProceduralSkyMaterial.new()
	env.sky = sky
	env.ambient_light_source = Environment.AMBIENT_SOURCE_SKY
	env.ambient_light_energy = 0.7
	var we := WorldEnvironment.new()
	we.environment = env
	add_child(we)

	var sun := DirectionalLight3D.new()
	sun.rotation_degrees = Vector3(-50, -35, 0)
	sun.shadow_enabled = true
	add_child(sun)

func solid(color: Color) -> StandardMaterial3D:
	var m := StandardMaterial3D.new()
	m.albedo_color = color
	return m

func add_body(shape: Shape3D, mesh: Mesh, xform: Transform3D, color: Color) -> StaticBody3D:
	var body := StaticBody3D.new()
	var cs := CollisionShape3D.new()
	cs.shape = shape
	body.add_child(cs)
	var mi := MeshInstance3D.new()
	mi.mesh = mesh
	mi.material_override = solid(color)
	body.add_child(mi)
	body.transform = xform
	add_child(body)
	return body

func add_box(size: Vector3, xform: Transform3D, color: Color) -> StaticBody3D:
	var shape := BoxShape3D.new()
	shape.size = size
	var mesh := BoxMesh.new()
	mesh.size = size
	return add_body(shape, mesh, xform, color)

func add_cylinder(radius: float, height: float, pos: Vector3, color: Color) -> StaticBody3D:
	var shape := CylinderShape3D.new()
	shape.radius = radius
	shape.height = height
	var mesh := CylinderMesh.new()
	mesh.top_radius = radius
	mesh.bottom_radius = radius
	mesh.height = height
	return add_body(shape, mesh, Transform3D(Basis(), pos), color)

func build_level() -> void:
	var grey := Color(0.36, 0.38, 0.41)
	var wood := Color(0.6, 0.45, 0.25)
	var stone := Color(0.5, 0.5, 0.55)

	add_box(Vector3(80, 1, 80), Transform3D(Basis(), Vector3(0, -0.5, 0)), grey)

	# Столб с неподвижным концом кабеля.
	add_box(Vector3(0.3, 2.2, 0.3), Transform3D(Basis(), Vector3(0, 1.1, 0)), wood)

	# Низкие блоки: на них можно просто зайти шагом.
	add_box(Vector3(1.4, 0.4, 1.4), Transform3D(Basis(), Vector3(3.0, 0.2, 0)), stone)
	add_box(Vector3(1.4, 0.8, 1.4), Transform3D(Basis(), Vector3(4.6, 0.4, 0)), stone)
	add_box(Vector3(1.4, 1.2, 1.4), Transform3D(Basis(), Vector3(6.2, 0.6, 0)), stone)

	# Пандус на высокую платформу: кабель тащится вверх по наклонной.
	add_box(Vector3(2.0, 0.3, 6.0), Transform3D(Basis(Vector3.RIGHT, deg_to_rad(-13.0)),
		Vector3(-4.0, 0.7, -3.0)), Color(0.42, 0.5, 0.42))
	add_box(Vector3(5.0, 0.3, 4.0), Transform3D(Basis(), Vector3(-4.0, 1.5, -7.5)), stone)

	# Стол: кабель перекидывается через край и свисает.
	add_box(Vector3(2.4, 0.2, 1.6), Transform3D(Basis(), Vector3(2.0, 1.1, 5.0)), wood)
	for dx in [-1.0, 1.0]:
		for dz in [-0.6, 0.6]:
			add_box(Vector3(0.15, 1.0, 0.15), Transform3D(Basis(),
				Vector3(2.0 + dx, 0.5, 5.0 + dz)), wood)

	# Ящик с острым ребром.
	add_box(Vector3(0.9, 0.9, 0.9), Transform3D(Basis(), Vector3(-2.5, 0.45, 3.0)), wood)

	# Колонны для петель.
	add_cylinder(0.4, 4.0, Vector3(8.5, 2.0, 3.0), stone)
	add_cylinder(0.4, 4.0, Vector3(11.0, 2.0, 0.5), stone)
	add_cylinder(0.25, 4.0, Vector3(9.5, 2.0, -3.0), stone)

	# Шар и капсула.
	var sph := SphereShape3D.new()
	sph.radius = 0.7
	var sm := SphereMesh.new()
	sm.radius = 0.7
	sm.height = 1.4
	add_body(sph, sm, Transform3D(Basis(), Vector3(5.5, 0.7, -3.5)), Color(0.35, 0.45, 0.6))

	var cap := CapsuleShape3D.new()
	cap.radius = 0.35
	cap.height = 1.8
	var capm := CapsuleMesh.new()
	capm.radius = 0.35
	capm.height = 1.8
	add_body(cap, capm, Transform3D(Basis(Vector3.RIGHT, deg_to_rad(90.0)),
		Vector3(0.5, 0.35, -4.5)), Color(0.6, 0.55, 0.35))

	# Бортик: волочить кабель поперёк.
	add_box(Vector3(7.0, 0.35, 0.3), Transform3D(Basis(), Vector3(7.0, 0.175, 7.0)),
		Color(0.55, 0.3, 0.3))

func build_player() -> void:
	player = CharacterBody3D.new()
	player.position = Vector3(1.2, 0.85, 1.2)
	var cs := CollisionShape3D.new()
	var cap := CapsuleShape3D.new()
	cap.radius = 0.35
	cap.height = 1.7
	cs.shape = cap
	player.add_child(cs)
	add_child(player)

	yaw_node = Node3D.new()
	yaw_node.position = Vector3(0, 0.7, 0)
	player.add_child(yaw_node)

	pitch_node = Node3D.new()
	yaw_node.add_child(pitch_node)
	pitch_node.add_child(Camera3D.new())

	# Катушка закреплена на корпусе, а не на yaw камеры: осмотр по сторонам не
	# перемещает разъём по окружности и не хлещет кабелем.
	hand = Node3D.new()
	hand.position = Vector3(0.25, 0.8, 0.0)
	player.add_child(hand)

func build_cable() -> void:
	var anchor := Node3D.new()
	anchor.position = Vector3(0, 2.0, 0.2)
	add_child(anchor)

	cable = CableBody3D.new()
	cable.cable_length = 40.0
	cable.cable_radius = 0.02
	cable.target_segment_length = 0.07
	cable.mass_per_meter = 0.25
	cable.bending_compliance = 1.0
	cable.damping = 80.0
	cable.minimum_bend_radius = 0.0
	cable.cable_material = solid(Color(0.9, 0.5, 0.1))
	add_child(cable)

	cable.anchor_a = cable.get_path_to(anchor)
	cable.anchor_b = cable.get_path_to(hand)
	cable.add_collision_exception(player)
	cable.rebuild_cable()
	cable.taut_limit_reached.connect(_on_taut_limit)

func build_hud() -> void:
	var layer := CanvasLayer.new()
	add_child(layer)

	hud = Label.new()
	hud.position = Vector2(16, 12)
	hud.add_theme_color_override("font_color", Color.WHITE)
	hud.add_theme_color_override("font_outline_color", Color.BLACK)
	hud.add_theme_constant_override("outline_size", 6)
	layer.add_child(hud)

	toast = Label.new()
	toast.position = Vector2(16, 300)
	toast.add_theme_color_override("font_color", Color(1, 0.5, 0.3))
	toast.add_theme_color_override("font_outline_color", Color.BLACK)
	toast.add_theme_constant_override("outline_size", 6)
	layer.add_child(toast)

# --- ввод ---

func _input(event: InputEvent) -> void:
	if event is InputEventMouseMotion:
		# Мышь вращает камеру и при отпущенном курсоре, если зажата кнопка: так
		# камера работает, даже когда захват курсора не встал.
		if mouse_captured or (event.button_mask & MOUSE_BUTTON_MASK_LEFT) != 0:
			yaw_node.rotate_y(-event.relative.x * MOUSE_SENS)
			pitch_node.rotate_x(-event.relative.y * MOUSE_SENS)
			pitch_node.rotation.x = clampf(pitch_node.rotation.x, -1.4, 1.4)
		return
	if event is InputEventMouseButton and event.pressed:
		capture_mouse(true) # клик по окну возвращает захват
		return
	if not (event is InputEventKey and event.pressed and not event.echo):
		return

	match event.keycode:
		KEY_ESCAPE:
			capture_mouse(not mouse_captured)
		KEY_F1:
			cable.debug_draw_enabled = not cable.debug_draw_enabled
		KEY_F2:
			cable.self_collision_enabled = not cable.self_collision_enabled
		KEY_F3:
			cable.sleep_enabled = not cable.sleep_enabled
		KEY_F4:
			flying = not flying
			player.velocity = Vector3.ZERO
		KEY_1:
			cable.minimum_bend_radius = maxf(cable.minimum_bend_radius - 0.05, 0.0)
		KEY_2:
			cable.minimum_bend_radius += 0.05
		KEY_5:
			cable.damping = maxf(cable.damping - 10.0, 0.0)
		KEY_6:
			cable.damping += 10.0
		KEY_3:
			cable.bending_compliance = minf(cable.bending_compliance * 2.0, 1000.0)
		KEY_4:
			cable.bending_compliance = maxf(cable.bending_compliance * 0.5, 1e-6)
		KEY_R:
			player.position = Vector3(1.2, 0.85, 1.2)
			player.velocity = Vector3.ZERO
			cable.rebuild_cable()
		KEY_F10:
			get_tree().quit()

# --- журнал поведения ---

# Скорость частиц восстанавливается по смещению центральной линии за кадр: это
# именно то, что видно глазом, в отличие от внутренних скоростей солвера.
func start_log(p_path: String) -> void:
	log_file = FileAccess.open(p_path, FileAccess.WRITE)
	log_file.store_line("frame,player_x,player_y,player_z,player_speed,hand_speed," +
			"v_max,v_max_at,v_q0,v_q1,v_q2,v_q3,v_mean,tension,strain_max," +
			"substeps,iterations,contacts,segment_contacts,self_contacts,awake,particles,deployed,poly," +
			"kinetic,bending,solver_stretch,projection,phys_ms")
	log_frame = 0
	log_prev = cable.get_centerline()

func write_log(p_delta: float, p_hand_speed: float) -> void:
	var now := cable.get_centerline()
	var n := now.size()
	log_frame += 1
	if log_prev.size() != n or n < 8:
		log_prev = now
		return

	var q := [0.0, 0.0, 0.0, 0.0]
	var v_max := 0.0
	var v_at := 0
	var v_sum := 0.0
	var strain_max := 0.0
	var rest: float = cable.get_deployed_length() / float(n - 1)
	var poly := 0.0
	for i in n:
		var v: float = log_prev[i].distance_to(now[i]) / p_delta
		v_sum += v
		if v > v_max:
			v_max = v
			v_at = i
		var qi: int = mini(i * 4 / n, 3)
		q[qi] = maxf(q[qi], v)
		if i > 0:
			var seg: float = now[i].distance_to(now[i - 1])
			poly += seg
			strain_max = maxf(strain_max, seg / rest - 1.0)

	log_file.store_line("%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.2f,%.4f,%d,%d,%d,%d,%d,%d,%d,%.3f,%.3f,%.6f,%.6f,%.6f,%.6f,%.3f" % [
		log_frame, player.global_position.x, player.global_position.y, player.global_position.z,
		Vector3(player.velocity.x, 0, player.velocity.z).length(), p_hand_speed,
		v_max, v_at, q[0], q[1], q[2], q[3], v_sum / float(n),
		cable.get_tension(), strain_max, cable.get_substep_count(), cable.get_solver_iteration_count(),
		cable.get_contact_count(), cable.get_world_segment_contact_count(),
		cable.get_self_contact_count(), cable.get_awake_particle_count(),
		n, cable.get_deployed_length(), poly, cable.get_kinetic_energy(), cable.get_bending_energy(),
		cable.get_max_stretch_error(), cable.get_max_projection(),
		Performance.get_monitor(Performance.TIME_PHYSICS_PROCESS) * 1000.0])
	if log_frame % 60 == 0:
		log_file.flush()
	log_prev = now

func capture_mouse(p_captured: bool) -> void:
	mouse_captured = p_captured
	Input.mouse_mode = Input.MOUSE_MODE_CAPTURED if p_captured else Input.MOUSE_MODE_VISIBLE

# Окно, поднятое поверх уже запущенного Godot, получает фокус позже _ready, и
# захват курсора, выставленный там, до него не доходит.
func _notification(what: int) -> void:
	if what == NOTIFICATION_APPLICATION_FOCUS_IN and mouse_captured:
		Input.mouse_mode = Input.MOUSE_MODE_CAPTURED

func move_input() -> Vector3:
	if scripted_input != Vector3.ZERO:
		return scripted_input # headless-прогон крутит ровно тот же контроллер
	var dir := Vector3.ZERO
	var basis: Basis = yaw_node.global_transform.basis
	if Input.is_key_pressed(KEY_W):
		dir -= basis.z
	if Input.is_key_pressed(KEY_S):
		dir += basis.z
	if Input.is_key_pressed(KEY_A):
		dir -= basis.x
	if Input.is_key_pressed(KEY_D):
		dir += basis.x
	return dir

# --- передвижение ---

func _physics_process(delta: float) -> void:
	if flying:
		fly(delta)
		return

	var dir := move_input()
	dir.y = 0.0
	dir = dir.normalized()

	var speed := RUN_SPEED if Input.is_key_pressed(KEY_SHIFT) else SPEED
	player.velocity.x = move_toward(player.velocity.x, dir.x * speed, MOVE_ACCEL * delta)
	player.velocity.z = move_toward(player.velocity.z, dir.z * speed, MOVE_ACCEL * delta)
	if player.is_on_floor() and Input.is_key_pressed(KEY_SPACE):
		player.velocity.y = JUMP
	else:
		player.velocity.y -= GRAVITY * delta

	apply_cable_limit(delta)
	# Шаг вверх решается до переноса: move_and_slide гасит скорость при упоре в стену,
	# и после него уже не видно, куда игрок хотел идти.
	try_step_up(Vector3(player.velocity.x, 0.0, player.velocity.z))

	var hand_before := hand.global_position
	player.move_and_slide()
	if log_file != null:
		write_log(delta, hand_before.distance_to(hand.global_position) / delta)

func fly(delta: float) -> void:
	var dir := move_input()
	if Input.is_key_pressed(KEY_SPACE):
		dir += Vector3.UP
	if Input.is_key_pressed(KEY_CTRL):
		dir += Vector3.DOWN
	dir = dir.normalized()
	var speed := FLY_SPEED * (2.0 if Input.is_key_pressed(KEY_SHIFT) else 1.0)
	player.velocity = dir * speed
	apply_cable_limit(delta)
	player.global_position += player.velocity * delta

# Кабель ограничивает перемещение до фактического переноса, как move_and_slide.
func apply_cable_limit(delta: float) -> void:
	var from := player.global_position
	var wanted := from + player.velocity * delta
	var allowed: Vector3 = cable.constrain_motion(from, wanted)
	player.velocity = (allowed - from) / delta

# CharacterBody3D сам не переступает уступы: если на уровне ног путь закрыт, а на
# высоте шага свободен, игрок поднимается.
func try_step_up(p_wish: Vector3) -> void:
	if not player.is_on_floor() or p_wish.length() < 0.5:
		return
	var motion := p_wish.normalized() * 0.4
	if not player.test_move(player.global_transform, motion):
		return
	# Минимальная достаточная высота, иначе игрок подскакивает выше уступа.
	for step in [0.15, 0.3, STEP_HEIGHT]:
		var raised := player.global_transform
		raised.origin += Vector3.UP * step
		if not player.test_move(raised, motion):
			player.global_position = raised.origin
			return

func _on_taut_limit() -> void:
	toast_time = 2.5

func _process(delta: float) -> void:
	toast_time = maxf(toast_time - delta, 0.0)
	toast.text = "Кабель размотан до конца" if toast_time > 0.0 else ""

	hud.text = "\n".join([
		"FPS %d   физика %.2f мс" % [Engine.get_frames_per_second(),
			Performance.get_monitor(Performance.TIME_PHYSICS_PROCESS) * 1000.0],
		"осталось %.1f м / размотано %.1f м" % [cable.get_remaining_length(), cable.get_deployed_length()],
		"частиц %d, активных %d, контактов %d, подшагов %d" % [cable.get_particle_count(),
			cable.get_awake_particle_count(), cable.get_contact_count(), cable.get_substep_count()],
		"натяжение %.1f Н" % cable.get_tension(),
		"внутреннее демпфирование %.0f  [5] / [6]" % cable.damping,
		"",
		"мин. радиус изгиба %.2f м (%.1f диаметра)  [1] / [2]" % [cable.minimum_bend_radius,
			cable.minimum_bend_radius / (cable.cable_radius * 2.0)],
		"податливость изгиба %.6f  [3] мягче / [4] жёстче" % cable.bending_compliance,
		"",
		"WASD + мышь, Shift бег, Space прыжок, уступы переступаются",
		"F4 свободный полёт (%s), в полёте Space/Ctrl вверх-вниз" % ("вкл" if flying else "выкл"),
		"F1 отладка (%s)  F2 самопересечение (%s)  F3 усыпление (%s)" % [
			"вкл" if cable.debug_draw_enabled else "выкл",
			"вкл" if cable.self_collision_enabled else "выкл",
			"вкл" if cable.sleep_enabled else "выкл"],
		"R сброс, Esc мышь, F10 выход",
	])
