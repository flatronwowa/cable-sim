extends SceneTree

const WARMUP_FRAMES := 30
const SAMPLE_FRAMES := 240

func percentile(values: Array[float], fraction: float) -> float:
	values.sort()
	var index := mini(int(ceil(fraction * values.size())) - 1, values.size() - 1)
	return values[maxi(index, 0)]

func report(name: String, values: Array[float], suffix: String = " us") -> void:
	print("%s p50=%.3f%s p95=%.3f%s p99=%.3f%s" % [
		name,
		percentile(values.duplicate(), 0.50), suffix,
		percentile(values.duplicate(), 0.95), suffix,
		percentile(values.duplicate(), 0.99), suffix,
	])

func report_scientific(name: String, values: Array[float]) -> void:
	print("%s p50=%.15f p95=%.15f p99=%.15f" % [
		name,
		percentile(values.duplicate(), 0.50),
		percentile(values.duplicate(), 0.95),
		percentile(values.duplicate(), 0.99),
	])

func _initialize() -> void:
	var demo = load("res://tmp_demo.tscn").instantiate()
	root.add_child(demo)
	await process_frame

	var cable: CableBody3D = demo.cable
	demo.set_physics_process(false)
	cable.cable_length = 40.04
	cable.target_segment_length = 0.07
	demo.player.global_position = Vector3(39.79, 1.2, 0.2)
	cable.gravity_scale = 1.0
	cable.axial_stiffness = 100000.0
	cable.v2_bending_stiffness = 0.01
	cable.solver_backend = CableBody3D.IMPLICIT_V2
	cable.rebuild_cable()

	await physics_frame
	if cable.get_particle_count() != 573:
		push_error("V2 benchmark requires 573 nodes, got %d" % cable.get_particle_count())
		quit(1)
		return

	for frame in WARMUP_FRAMES:
		await physics_frame
	var growths_after_warmup := cable.get_v2_capacity_growths()

	var assembly: Array[float] = []
	var inertia_assembly: Array[float] = []
	var stretch_assembly: Array[float] = []
	var bending_assembly: Array[float] = []
	var line_search_evaluation: Array[float] = []
	var symbolic: Array[float] = []
	var factorization: Array[float] = []
	var solve: Array[float] = []
	var iterations: Array[float] = []
	var line_search_iterations: Array[float] = []
	var residual: Array[float] = []
	var relative_step: Array[float] = []
	var stretch: Array[float] = []
	var rejected := 0

	for frame in SAMPLE_FRAMES:
		await physics_frame
		assembly.push_back(cable.get_v2_assembly_us())
		inertia_assembly.push_back(cable.get_v2_inertia_assembly_us())
		stretch_assembly.push_back(cable.get_v2_stretch_assembly_us())
		bending_assembly.push_back(cable.get_v2_bending_assembly_us())
		line_search_evaluation.push_back(cable.get_v2_line_search_evaluation_us())
		symbolic.push_back(cable.get_v2_symbolic_analysis_us())
		factorization.push_back(cable.get_v2_numeric_factorization_us())
		solve.push_back(cable.get_v2_linear_solve_us())
		iterations.push_back(cable.get_v2_newton_iterations())
		line_search_iterations.push_back(cable.get_v2_line_search_iterations())
		residual.push_back(cable.get_v2_residual())
		relative_step.push_back(cable.get_v2_relative_step())
		stretch.push_back(cable.get_max_stretch_error())
		if not cable.was_v2_step_accepted():
			rejected += 1

	print("V2 contact-free benchmark: nodes=573 samples=%d rejected=%d" % [SAMPLE_FRAMES, rejected])
	report("assembly", assembly)
	report("inertia_assembly", inertia_assembly)
	report("stretch_assembly", stretch_assembly)
	report("bending_assembly", bending_assembly)
	report("line_search_evaluation", line_search_evaluation)
	report("symbolic", symbolic)
	report("numeric_factorization", factorization)
	report("linear_solve", solve)
	report("newton_iterations", iterations, "")
	report("line_search_iterations", line_search_iterations, "")
	report_scientific("residual", residual)
	report_scientific("relative_step", relative_step)
	report_scientific("max_stretch_error", stretch)
	print("capacity_growths_after_warmup=%d runtime_capacity_growths=%d peak_capacity_bytes=%d" % [
		growths_after_warmup,
		cable.get_v2_capacity_growths() - growths_after_warmup,
		cable.get_v2_peak_capacity_bytes(),
	])
	quit(0 if rejected == 0 else 1)
