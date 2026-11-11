extends Node3D

const DroneScript := preload("res://drone.gd")
const ReceiverScript := preload("res://udp_receiver.gd")

const UDP_PORT := 9999
const CMD_PORT := 9998
const GRAVITY := 9.81
const MASS := 0.45
const SMOOTH := 30.0
const TRAIL_MAX := 800
const TRAIL_MIN_STEP := 0.02

enum CamMode { ORBIT, FOLLOW, CHASE }
const CAM_MODE_NAMES := ["ORBIT", "FOLLOW", "CHASE"]

var receiver: Node
var cmd_peer := PacketPeerUDP.new()
var drone: Node3D
var camera: Camera3D
var target_marker: Node3D
var center_marker: Node3D
var obstacle_root: Node3D
var hud_label: Label
var banner: Label
var hud_panel: PanelContainer
var lines_mesh: ImmediateMesh
var trail_mesh: ImmediateMesh
var ref_mesh: ImmediateMesh
var trail_mat: StandardMaterial3D
var lines_mat: StandardMaterial3D

var state: Dictionary = {}
var has_state := false
var run_id := -1
var goal_pos := Vector3.ZERO
var goal_rot := Quaternion.IDENTITY
var target_pos := Vector3.ZERO
var raw_target_pos := Vector3.ZERO
var loiter_center := Vector3(0, 2.0, 0)
var loiter_radius := 3.0
var click_mode := "loiter"
var selected_alt := 2.0
var _click_start := Vector2.ZERO
var avoiding := false
var collided := false
var hit_idx := -1
var obstacles_key := ""
var obstacle_nodes: Array[MeshInstance3D] = []
var obstacle_mats: Array[StandardMaterial3D] = []
var trail := PackedVector3Array()
var ref_trail := PackedVector3Array()
var show_trail := true
var wind := Vector3.ZERO
var dust: CPUParticles3D

var cam_mode: int = CamMode.ORBIT
var cam_yaw := deg_to_rad(45.0)
var cam_pitch := deg_to_rad(25.0)
var cam_dist := 10.0
var cam_focus := Vector3(0, 1.5, 0)
var chase_heading := Vector3(1, 0, 0)
var shake := 0.0
var replay: Array = []
var replay_t := 0.0
var replay_idx := -1


static func s2g(v) -> Vector3:
	return Vector3(float(v[0]), float(v[2]), -float(v[1]))


static func rot_to_basis(r) -> Basis:
	var x := Vector3(float(r[0]), float(r[6]), -float(r[3]))
	var y := Vector3(float(r[2]), float(r[8]), -float(r[5]))
	var z := Vector3(-float(r[1]), -float(r[7]), float(r[4]))
	return Basis(x, y, z).orthonormalized()


func _ready() -> void:
	_build_environment()
	_build_ground()
	_build_axes()

	drone = DroneScript.new()
	drone.name = "Drone"
	add_child(drone)

	_build_target()
	obstacle_root = Node3D.new()
	obstacle_root.name = "Obstacles"
	add_child(obstacle_root)
	_build_lines()

	camera = Camera3D.new()
	camera.fov = 60.0
	camera.near = 0.02
	add_child(camera)
	camera.make_current()

	_build_hud()
	_build_dust()

	cmd_peer.set_dest_address("127.0.0.1", CMD_PORT)

	receiver = ReceiverScript.new()
	receiver.name = "UdpReceiver"
	receiver.port = UDP_PORT
	receiver.state_received.connect(_on_state)
	add_child(receiver)

	for a in OS.get_cmdline_user_args():
		if a.begins_with("--replay="):
			_load_replay(a.substr(9))


func _load_replay(path: String) -> void:
	var f := FileAccess.open(path, FileAccess.READ)
	if f == null:
		return
	while not f.eof_reached():
		var line := f.get_line()
		if line.length() > 2:
			var d = JSON.parse_string(line)
			if d is Dictionary:
				replay.append(d)
	cam_mode = CamMode.FOLLOW
	cam_dist = 6.5
	cam_pitch = deg_to_rad(18.0)
	hud_label.add_theme_font_size_override("font_size", 11)


func _build_environment() -> void:
	var sky_mat := ProceduralSkyMaterial.new()
	sky_mat.sky_top_color = Color(0.22, 0.38, 0.62)
	sky_mat.sky_horizon_color = Color(0.68, 0.74, 0.82)
	sky_mat.ground_horizon_color = Color(0.45, 0.47, 0.5)
	sky_mat.ground_bottom_color = Color(0.15, 0.16, 0.18)
	var sky := Sky.new()
	sky.sky_material = sky_mat

	var env := Environment.new()
	env.background_mode = Environment.BG_SKY
	env.sky = sky
	env.ambient_light_source = Environment.AMBIENT_SOURCE_SKY
	env.ambient_light_energy = 0.6
	env.tonemap_mode = Environment.TONE_MAPPER_FILMIC
	env.glow_enabled = true
	env.glow_intensity = 0.6
	env.ssao_enabled = true
	env.fog_enabled = true
	env.fog_light_color = Color(0.68, 0.74, 0.82)
	env.fog_density = 0.008

	var we := WorldEnvironment.new()
	we.environment = env
	add_child(we)

	var sun := DirectionalLight3D.new()
	sun.rotation_degrees = Vector3(-55, -35, 0)
	sun.light_energy = 1.2
	sun.shadow_enabled = true
	sun.directional_shadow_max_distance = 40.0
	add_child(sun)


func _build_ground() -> void:
	var shader := Shader.new()
	shader.code = """
shader_type spatial;
uniform vec3 base_color = vec3(0.20, 0.22, 0.24);
uniform vec3 line_color = vec3(0.55, 0.60, 0.65);
uniform float fade_dist = 30.0;
varying vec3 wpos;
void vertex() { wpos = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz; }
float grid(vec2 c) {
	vec2 g = abs(fract(c - 0.5) - 0.5) / fwidth(c);
	return 1.0 - min(min(g.x, g.y), 1.0);
}
void fragment() {
	vec2 c = wpos.xz;
	float l = max(grid(c) * 0.35, grid(c / 5.0));
	float fade = 1.0 - smoothstep(fade_dist * 0.4, fade_dist, length(c));
	ALBEDO = mix(base_color, line_color, l * fade);
	ROUGHNESS = 0.95;
}
"""
	var mat := ShaderMaterial.new()
	mat.shader = shader
	var plane := PlaneMesh.new()
	plane.size = Vector2(80, 80)
	var ground := MeshInstance3D.new()
	ground.mesh = plane
	ground.material_override = mat
	add_child(ground)


func _unshaded(c: Color) -> StandardMaterial3D:
	var m := StandardMaterial3D.new()
	m.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	m.albedo_color = c
	return m


func _build_axes() -> void:
	var specs := [
		[Vector3(0.75, 0.005, 0), Vector3(1.5, 0.01, 0.02), Color(0.9, 0.2, 0.2)],
		[Vector3(0, 0.005, -0.75), Vector3(0.02, 0.01, 1.5), Color(0.2, 0.85, 0.3)],
		[Vector3(0, 0.75, 0), Vector3(0.02, 1.5, 0.02), Color(0.25, 0.45, 1.0)],
	]
	for s in specs:
		var mi := MeshInstance3D.new()
		var bm := BoxMesh.new()
		bm.size = s[1]
		mi.mesh = bm
		mi.position = s[0]
		mi.material_override = _unshaded(s[2])
		mi.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
		add_child(mi)


func _build_target() -> void:
	target_marker = Node3D.new()
	target_marker.name = "Target"
	add_child(target_marker)
	var m := StandardMaterial3D.new()
	m.albedo_color = Color(0.2, 1.0, 0.4)
	m.emission_enabled = true
	m.emission = Color(0.1, 1.0, 0.3)
	m.emission_energy_multiplier = 1.5
	var mi := MeshInstance3D.new()
	var bm := BoxMesh.new()
	bm.size = Vector3(0.08, 0.08, 0.08)
	mi.mesh = bm
	mi.material_override = m
	mi.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	mi.name = "Cube"
	target_marker.add_child(mi)
	target_marker.visible = false

	center_marker = Node3D.new()
	center_marker.name = "LoiterCenter"
	add_child(center_marker)
	var cm := StandardMaterial3D.new()
	cm.albedo_color = Color(1.0, 0.55, 0.1)
	cm.emission_enabled = true
	cm.emission = Color(1.0, 0.45, 0.05)
	cm.emission_energy_multiplier = 2.0
	var cmi := MeshInstance3D.new()
	var csm := SphereMesh.new()
	csm.radius = 0.11
	csm.height = 0.22
	cmi.mesh = csm
	cmi.material_override = cm
	cmi.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	center_marker.add_child(cmi)
	center_marker.visible = false


func _line_mat() -> StandardMaterial3D:
	var m := StandardMaterial3D.new()
	m.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	m.vertex_color_use_as_albedo = true
	m.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	return m


func _build_lines() -> void:
	trail_mat = _line_mat()
	lines_mat = _line_mat()
	for pair in [["Trail", "trail_mesh"], ["RefPath", "ref_mesh"], ["Guides", "lines_mesh"]]:
		var im := ImmediateMesh.new()
		var mi := MeshInstance3D.new()
		mi.name = pair[0]
		mi.mesh = im
		mi.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
		add_child(mi)
		set(pair[1], im)


func _build_hud() -> void:
	var layer := CanvasLayer.new()
	add_child(layer)

	hud_panel = PanelContainer.new()
	var sb := StyleBoxFlat.new()
	sb.bg_color = Color(0.05, 0.06, 0.08, 0.72)
	sb.set_corner_radius_all(6)
	sb.set_content_margin_all(10)
	hud_panel.add_theme_stylebox_override("panel", sb)
	hud_panel.position = Vector2(12, 12)
	layer.add_child(hud_panel)

	hud_label = Label.new()
	hud_label.add_theme_font_size_override("font_size", 14)
	var mono := SystemFont.new()
	mono.font_names = PackedStringArray(["DejaVu Sans Mono", "Consolas", "Menlo", "monospace"])
	hud_label.add_theme_font_override("font", mono)
	hud_panel.add_child(hud_label)

	banner = Label.new()
	banner.text = "COLLISION"
	banner.add_theme_font_size_override("font_size", 48)
	banner.add_theme_color_override("font_color", Color(1, 0.25, 0.2))
	banner.add_theme_color_override("font_outline_color", Color.BLACK)
	banner.add_theme_constant_override("outline_size", 8)
	banner.set_anchors_preset(Control.PRESET_CENTER_TOP)
	banner.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
	banner.position.y = 40
	banner.grow_horizontal = Control.GROW_DIRECTION_BOTH
	banner.visible = false
	layer.add_child(banner)


func _build_dust() -> void:
	dust = CPUParticles3D.new()
	var bm := BoxMesh.new()
	bm.size = Vector3(0.015, 0.015, 0.015)
	var m := _unshaded(Color(1, 1, 1, 0.45))
	m.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	bm.material = m
	dust.mesh = bm
	dust.amount = 400
	dust.lifetime = 4.0
	dust.preprocess = 4.0
	dust.emission_shape = CPUParticles3D.EMISSION_SHAPE_BOX
	dust.emission_box_extents = Vector3(10, 2.5, 10)
	dust.position = Vector3(0, 2.5, 0)
	dust.gravity = Vector3.ZERO
	dust.spread = 8.0
	dust.local_coords = false
	dust.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	dust.emitting = false
	add_child(dust)


func _update_dust() -> void:
	var speed := wind.length()
	dust.emitting = speed > 0.2
	if speed > 0.2:
		dust.direction = wind / speed
		dust.initial_velocity_min = speed * 0.9
		dust.initial_velocity_max = speed * 1.1
		dust.position = Vector3(0, 2.5, 0) - Vector3(wind.x, 0, wind.z).normalized() * 2.0


func _on_state(s: Dictionary) -> void:
	if not (s.has("pos") and s.has("rot")):
		return
	var new_run := int(s.get("run", 0))
	var first := not has_state or new_run != run_id
	state = s
	has_state = true
	run_id = new_run

	goal_pos = s2g(s["pos"])
	goal_rot = rot_to_basis(s["rot"]).get_rotation_quaternion()
	if s.has("target"):
		raw_target_pos = s2g(s["target"])
		target_pos = s2g(s.get("cmd_target", s["target"]))
		avoiding = target_pos.distance_to(raw_target_pos) > 0.01
	if s.has("loiter_center"):
		loiter_center = s2g(s["loiter_center"])
		center_marker.visible = true
		center_marker.position = loiter_center
	if s.has("loiter_radius"):
		loiter_radius = float(s["loiter_radius"])
	if s.has("surfaces"):
		var sf := PackedFloat32Array()
		for v in s["surfaces"]:
			sf.append(float(v))
		drone.surfaces = sf

	if first:
		drone.position = goal_pos
		drone.quaternion = goal_rot
		trail.clear()
		ref_trail.clear()
		_set_collision(false, -1)

	if s.has("obstacles"):
		var key := str(s["obstacles"])
		if key != obstacles_key:
			obstacles_key = key
			_rebuild_obstacles(s["obstacles"], float(s.get("drone_radius", 0.15)))
	if s.has("wind"):
		wind = s2g(s["wind"])
	if s.has("motors"):
		var hm := float(s.get("hover_motor", MASS * GRAVITY / 4.0))
		var mr := PackedFloat32Array()
		for f in s["motors"]:
			mr.append(float(f) / hm)
		drone.motor_ratios = mr
	if s.has("drone_radius"):
		drone.set_collision_radius(float(s["drone_radius"]))

	_set_collision(bool(s.get("collision", false)), int(s.get("hit", -1)))


func _rebuild_obstacles(list: Array, drone_r: float) -> void:
	for c in obstacle_root.get_children():
		c.queue_free()
	obstacle_nodes.clear()
	obstacle_mats.clear()
	for o in list:
		var r := float(o[3])
		var center := s2g(o)
		var m := StandardMaterial3D.new()
		m.albedo_color = Color(0.85, 0.18, 0.15)
		m.roughness = 0.35
		m.emission_enabled = true
		m.emission = Color(0.4, 0.02, 0.0)
		var mi := MeshInstance3D.new()
		var sm := SphereMesh.new()
		sm.radius = r
		sm.height = r * 2.0
		mi.mesh = sm
		mi.material_override = m
		mi.position = center
		obstacle_root.add_child(mi)
		obstacle_nodes.append(mi)
		obstacle_mats.append(m)

		var shell := MeshInstance3D.new()
		var ss := SphereMesh.new()
		ss.radius = r + drone_r
		ss.height = (r + drone_r) * 2.0
		shell.mesh = ss
		var sm2 := StandardMaterial3D.new()
		sm2.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
		sm2.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
		sm2.albedo_color = Color(1, 0.3, 0.2, 0.08)
		sm2.cull_mode = BaseMaterial3D.CULL_DISABLED
		shell.material_override = sm2
		shell.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
		shell.position = center
		obstacle_root.add_child(shell)

		var pole := MeshInstance3D.new()
		var pb := BoxMesh.new()
		pb.size = Vector3(0.03, max(center.y - r, 0.0), 0.03)
		pole.mesh = pb
		pole.position = Vector3(center.x, (center.y - r) * 0.5, center.z)
		pole.material_override = _unshaded(Color(0.35, 0.35, 0.38))
		obstacle_root.add_child(pole)


func _set_collision(v: bool, idx: int) -> void:
	if v and not collided:
		shake = 0.35
		_spawn_burst(goal_pos)
	collided = v
	hit_idx = idx
	drone.collided = v
	banner.visible = v


func _spawn_burst(at: Vector3) -> void:
	var p := CPUParticles3D.new()
	var bm := BoxMesh.new()
	bm.size = Vector3(0.025, 0.025, 0.025)
	var m := StandardMaterial3D.new()
	m.albedo_color = Color(1, 0.55, 0.1)
	m.emission_enabled = true
	m.emission = Color(1, 0.4, 0.05)
	m.emission_energy_multiplier = 3.0
	bm.material = m
	p.mesh = bm
	p.amount = 60
	p.one_shot = true
	p.explosiveness = 1.0
	p.lifetime = 1.2
	p.direction = Vector3.UP
	p.spread = 180.0
	p.initial_velocity_min = 1.5
	p.initial_velocity_max = 4.0
	p.gravity = Vector3(0, -GRAVITY, 0)
	p.scale_amount_min = 0.5
	p.scale_amount_max = 1.5
	p.position = at
	add_child(p)
	p.emitting = true
	get_tree().create_timer(2.0).timeout.connect(p.queue_free)


func _process(delta: float) -> void:
	if replay.size() > 0:
		replay_t += delta
		var idx := mini(int(replay_t * 50.0), replay.size() - 1)
		if idx != replay_idx:
			replay_idx = idx
			_on_state(replay[idx])
		cam_yaw += delta * 0.10
	if has_state:
		var a := 1.0 - exp(-delta * SMOOTH)
		drone.position = drone.position.lerp(goal_pos, a)
		drone.quaternion = drone.quaternion.slerp(goal_rot, a)
		var hover := MASS * GRAVITY
		drone.thrust_ratio = float(state.get("thrust", 0.0)) / hover

		target_marker.visible = true
		target_marker.position = target_marker.position.lerp(target_pos, a)
		var cube := target_marker.get_node("Cube") as Node3D
		cube.rotate_y(delta * 2.0)
		cube.rotate_x(delta * 1.3)

		_update_trails()

	for i in obstacle_mats.size():
		var m := obstacle_mats[i]
		if collided and i == hit_idx:
			var k := 0.5 + 0.5 * sin(Time.get_ticks_msec() * 0.012)
			m.emission = Color(1.0, 0.25, 0.05) * (0.5 + 1.5 * k)
		else:
			m.emission = Color(0.4, 0.02, 0.0)

	_draw_guides()
	_update_dust()
	_update_camera(delta)
	_update_hud()


func _update_trails() -> void:
	var p := drone.position
	if trail.is_empty() or trail[trail.size() - 1].distance_to(p) > TRAIL_MIN_STEP:
		trail.append(p)
		if trail.size() > TRAIL_MAX:
			trail = trail.slice(trail.size() - TRAIL_MAX)
	if ref_trail.is_empty() or ref_trail[ref_trail.size() - 1].distance_to(raw_target_pos) > TRAIL_MIN_STEP * 2.0:
		ref_trail.append(raw_target_pos)
		if ref_trail.size() > TRAIL_MAX:
			ref_trail = ref_trail.slice(ref_trail.size() - TRAIL_MAX)

	trail_mesh.clear_surfaces()
	ref_mesh.clear_surfaces()
	if not show_trail:
		return
	_strip(trail_mesh, trail, Color(0.25, 0.65, 1.0), true)
	_strip(ref_mesh, ref_trail, Color(0.8, 0.8, 0.8, 0.35), false)


func _strip(im: ImmediateMesh, pts: PackedVector3Array, c: Color, fade: bool) -> void:
	var n := pts.size()
	if n < 2:
		return
	im.surface_begin(Mesh.PRIMITIVE_LINE_STRIP, trail_mat)
	for i in n:
		var alpha: float = c.a * (float(i) / float(n - 1) if fade else 1.0)
		im.surface_set_color(Color(c.r, c.g, c.b, alpha))
		im.surface_add_vertex(pts[i])
	im.surface_end()


func _draw_guides() -> void:
	lines_mesh.clear_surfaces()
	if not has_state:
		return
	var d := drone.position
	var t := target_marker.position
	lines_mesh.surface_begin(Mesh.PRIMITIVE_LINES, lines_mat)
	lines_mesh.surface_set_color(Color(1, 1, 1, 0.35))
	lines_mesh.surface_add_vertex(d)
	lines_mesh.surface_add_vertex(Vector3(d.x, 0.0, d.z))
	lines_mesh.surface_set_color(Color(0.2, 1.0, 0.4, 0.5))
	lines_mesh.surface_add_vertex(d)
	lines_mesh.surface_add_vertex(t)
	lines_mesh.surface_set_color(Color(0.2, 1.0, 0.4, 0.2))
	lines_mesh.surface_add_vertex(t)
	lines_mesh.surface_add_vertex(Vector3(t.x, 0.0, t.z))
	if center_marker.visible:
		var c := loiter_center
		lines_mesh.surface_set_color(Color(1.0, 0.55, 0.1, 0.65))
		lines_mesh.surface_add_vertex(c)
		lines_mesh.surface_add_vertex(Vector3(c.x, 0.0, c.z))
		var segs := 64
		for i in segs:
			var a0 := TAU * float(i) / float(segs)
			var a1 := TAU * float(i + 1) / float(segs)
			var p0 := c + Vector3(cos(a0) * loiter_radius, 0.0, -sin(a0) * loiter_radius)
			var p1 := c + Vector3(cos(a1) * loiter_radius, 0.0, -sin(a1) * loiter_radius)
			lines_mesh.surface_set_color(Color(1.0, 0.65, 0.15, 0.45))
			lines_mesh.surface_add_vertex(p0)
			lines_mesh.surface_add_vertex(p1)
	if avoiding:
		lines_mesh.surface_set_color(Color(1.0, 0.5, 0.1, 0.8))
		lines_mesh.surface_add_vertex(raw_target_pos)
		lines_mesh.surface_add_vertex(t)
	if state.has("vel"):
		lines_mesh.surface_set_color(Color(1.0, 0.8, 0.2, 0.9))
		lines_mesh.surface_add_vertex(d)
		lines_mesh.surface_add_vertex(d + s2g(state["vel"]) * 0.3)
	if wind.length() > 0.05:
		var base := d + Vector3.UP * 0.45
		var tip := base + wind * 0.25
		var side := wind.cross(Vector3.UP).normalized() * 0.06
		var back := -wind.normalized() * 0.12
		lines_mesh.surface_set_color(Color(0.4, 0.9, 1.0, 0.9))
		for seg in [[base, tip], [tip, tip + back + side], [tip, tip + back - side]]:
			lines_mesh.surface_add_vertex(seg[0])
			lines_mesh.surface_add_vertex(seg[1])
	lines_mesh.surface_end()


func _unhandled_input(event: InputEvent) -> void:
	if event is InputEventMouseMotion:
		var mm := event as InputEventMouseMotion
		if mm.button_mask & (MOUSE_BUTTON_MASK_LEFT | MOUSE_BUTTON_MASK_RIGHT):
			cam_yaw -= mm.relative.x * 0.006
			cam_pitch = clampf(cam_pitch + mm.relative.y * 0.006, deg_to_rad(-10), deg_to_rad(89))
		elif mm.button_mask & MOUSE_BUTTON_MASK_MIDDLE and cam_mode == CamMode.ORBIT:
			var right := camera.global_transform.basis.x
			var fwd := Vector3(-sin(cam_yaw), 0, -cos(cam_yaw))
			cam_focus += (-right * mm.relative.x + fwd * mm.relative.y) * cam_dist * 0.0015
	elif event is InputEventMouseButton:
		var mb := event as InputEventMouseButton
		if mb.pressed:
			if mb.button_index == MOUSE_BUTTON_WHEEL_UP:
				cam_dist = maxf(cam_dist * 0.9, 0.4)
			elif mb.button_index == MOUSE_BUTTON_WHEEL_DOWN:
				cam_dist = minf(cam_dist * 1.1, 60.0)
			elif mb.button_index == MOUSE_BUTTON_RIGHT or (mb.button_index == MOUSE_BUTTON_LEFT and mb.shift_pressed):
				_click_start = mb.position
		else:
			if (mb.button_index == MOUSE_BUTTON_RIGHT or (mb.button_index == MOUSE_BUTTON_LEFT and mb.shift_pressed)) and mb.position.distance_to(_click_start) < 6.0:
				_select_ground_point(mb.position)
	elif event is InputEventKey and event.pressed and not event.echo:
		match (event as InputEventKey).keycode:
			KEY_C:
				cam_mode = (cam_mode + 1) % 3
				if cam_mode == CamMode.FOLLOW:
					cam_dist = 2.5
				elif cam_mode == CamMode.ORBIT:
					cam_dist = 10.0
			KEY_R:
				cam_yaw = deg_to_rad(45.0)
				cam_pitch = deg_to_rad(25.0)
				cam_dist = 10.0 if cam_mode == CamMode.ORBIT else 2.5
				cam_focus = Vector3(0, 1.5, 0)
			KEY_T:
				show_trail = not show_trail
			KEY_B:
				drone.toggle_collision_sphere()
			KEY_H:
				hud_panel.visible = not hud_panel.visible
			KEY_M:
				click_mode = "fly_to" if click_mode == "loiter" else "loiter"
			KEY_0:
				_send_cmd({"cmd": "clear_point"})


func _send_cmd(payload: Dictionary) -> void:
	cmd_peer.put_packet(JSON.stringify(payload).to_utf8_buffer())


func _select_ground_point(screen_pos: Vector2) -> void:
	var orig := camera.project_ray_origin(screen_pos)
	var dir := camera.project_ray_normal(screen_pos)
	if absf(dir.y) < 1e-4:
		return
	var t_hit := -orig.y / dir.y
	if t_hit <= 0.0:
		return
	var hit_g := orig + dir * t_hit
	var sim_x := clampf(hit_g.x, -25.0, 25.0)
	var sim_y := clampf(-hit_g.z, -25.0, 25.0)
	_send_cmd({
		"cmd": "select_point",
		"pos": [sim_x, sim_y, selected_alt],
		"radius": loiter_radius,
		"mode": click_mode,
	})


func _update_camera(delta: float) -> void:
	var d := drone.position
	if cam_mode == CamMode.CHASE and has_state:
		var v := s2g(state.get("vel", [0, 0, 0]))
		var hv := Vector3(v.x, 0, v.z)
		if hv.length() > 0.3:
			chase_heading = chase_heading.lerp(hv.normalized(), 1.0 - exp(-delta * 2.0)).normalized()
		var want := d - chase_heading * 1.6 + Vector3.UP * 0.6
		camera.position = camera.position.lerp(want, 1.0 - exp(-delta * 5.0))
		camera.look_at(d + chase_heading * 0.5, Vector3.UP)
	else:
		var focus := cam_focus
		if cam_mode == CamMode.FOLLOW:
			focus = d
		var off := Vector3(cos(cam_pitch) * sin(cam_yaw), sin(cam_pitch), cos(cam_pitch) * cos(cam_yaw))
		camera.position = focus + off * cam_dist
		camera.look_at(focus, Vector3.UP)

	if shake > 0.0:
		camera.position += Vector3(randf_range(-1, 1), randf_range(-1, 1), randf_range(-1, 1)) * shake * 0.15
		shake = maxf(shake - delta, 0.0)


func _update_hud() -> void:
	var since: float = receiver.seconds_since_packet()
	var link: String
	if not receiver.listening:
		link = "ERROR: port %d unavailable" % UDP_PORT
	elif since == INF:
		link = "waiting for python on udp:%d ..." % UDP_PORT
	elif since > 1.0:
		link = "idle (%.0fs since last packet)" % since
	else:
		link = "LIVE  %.0f pkt/s" % receiver.packet_rate

	var lines := PackedStringArray()
	lines.append("FIXED-WING UAV SIM  ·  link: %s" % link)
	if has_state:
		var p: Array = state["pos"]
		var v: Array = state.get("vel", [0, 0, 0])
		var att: Array = state.get("att", [0, 0, 0])
		var lc: Array = state.get("loiter_center", [0, 0, 2.0])
		var speed := Vector3(float(v[0]), float(v[1]), float(v[2])).length()
		var airspeed := float(state.get("airspeed", speed))
		var d_center := Vector2(float(p[0]) - float(lc[0]), float(p[1]) - float(lc[1])).length()
		lines.append("t        %7.2f s     seq %d" % [float(state.get("t", 0.0)), int(state.get("seq", 0))])
		lines.append("pos      %6.2f %6.2f %6.2f m" % [float(p[0]), float(p[1]), float(p[2])])
		lines.append("point    %6.2f %6.2f %6.2f m  (dist %4.1fm)" % [float(lc[0]), float(lc[1]), float(lc[2]), d_center])
		lines.append("airspeed %6.2f m/s   alt %5.2f m" % [airspeed, float(p[2])])
		lines.append("bank/p/y %6.1f %6.1f %6.1f deg" % [rad_to_deg(float(att[0])), -rad_to_deg(float(att[1])), rad_to_deg(float(att[2]))])
		var wv: Array = state.get("wind", [0, 0, 0])
		var wsp := Vector3(float(wv[0]), float(wv[1]), float(wv[2])).length()
		var wdir := rad_to_deg(atan2(float(wv[1]), float(wv[0])))
		lines.append("wind     %6.2f m/s  toward %4.0f deg" % [wsp, wdir])
		if state.has("surfaces"):
			var sf: Array = state["surfaces"]
			lines.append("surfaces THR %3.0f%%  AIL %+4.0f%%  ELEV %+4.0f%%  RUD %+4.0f%%" % [
				100.0 * float(sf[0]), 100.0 * float(sf[1]),
				100.0 * float(sf[2]), 100.0 * float(sf[3])])
		var phase_str := str(state.get("phase", "flying"))
		var status_str: String
		if collided:
			status_str = "COLLISION (obstacle %d)" % hit_idx
		elif avoiding:
			status_str = "%s · AVOIDING" % phase_str
		else:
			status_str = phase_str
		lines.append("status   %s" % status_str)
	lines.append("click-mode: %s [M]  ·  Right-Click/Shift-Click ground to select point  ·  [0] auto" % click_mode.to_upper())
	lines.append("cam %s  ·  [C]am [R]eset [T]rail [B] hit-sphere [H]ud" % CAM_MODE_NAMES[cam_mode])
	hud_label.text = "\n".join(lines)
