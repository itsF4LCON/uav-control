extends Node3D

const WINGSPAN := 0.56
const FUSELAGE_LEN := 0.36
const PROP_LEN := 0.115
const MAX_PROP_SPEED := 85.0
const VTAIL_ANGLE := deg_to_rad(42.0)

var thrust_ratio: float = 0.5
var surfaces: PackedFloat32Array = PackedFloat32Array([0.5, 0.0, 0.0, 0.0])
var motor_ratios: PackedFloat32Array = PackedFloat32Array()
var collided: bool = false : set = set_collided

var _prop_pivot: Node3D
var _prop_disc: MeshInstance3D
var _aileron_l: Node3D
var _aileron_r: Node3D
var _vtail_l_hinge: Node3D
var _vtail_r_hinge: Node3D
var _vtail_flap_l: Node3D
var _vtail_flap_r: Node3D
var _ventral_rudder: Node3D
var _disc_mat: StandardMaterial3D
var _body_mat: StandardMaterial3D
var _wing_mat: StandardMaterial3D
var _body_color := Color(0.76, 0.79, 0.82)
var _wing_color := Color(0.74, 0.77, 0.80)
var _flash_t: float = 0.0
var _hit_sphere: MeshInstance3D


func _ready() -> void:
	_build()


func _mat(color: Color, emission := Color.BLACK, metallic := 0.15, rough := 0.55) -> StandardMaterial3D:
	var m := StandardMaterial3D.new()
	m.albedo_color = color
	m.metallic = metallic
	m.roughness = rough
	if emission != Color.BLACK:
		m.emission_enabled = true
		m.emission = emission
		m.emission_energy_multiplier = 2.0
	return m


func _box(parent: Node3D, size: Vector3, pos: Vector3, mat: Material, rot := Vector3.ZERO) -> MeshInstance3D:
	var mi := MeshInstance3D.new()
	var bm := BoxMesh.new()
	bm.size = size
	mi.mesh = bm
	mi.material_override = mat
	mi.position = pos
	mi.rotation = rot
	parent.add_child(mi)
	return mi


func _cyl(parent: Node3D, r_top: float, r_bot: float, h: float, pos: Vector3, mat: Material, rot := Vector3.ZERO) -> MeshInstance3D:
	var mi := MeshInstance3D.new()
	var cm := CylinderMesh.new()
	cm.top_radius = r_top
	cm.bottom_radius = r_bot
	cm.height = h
	cm.radial_segments = 20
	mi.mesh = cm
	mi.material_override = mat
	mi.position = pos
	mi.rotation = rot
	parent.add_child(mi)
	return mi


func _sphere(parent: Node3D, radius: float, pos: Vector3, mat: Material, scale_v := Vector3.ONE) -> MeshInstance3D:
	var mi := MeshInstance3D.new()
	var sm := SphereMesh.new()
	sm.radius = radius
	sm.height = radius * 2.0
	mi.mesh = sm
	mi.material_override = mat
	mi.position = pos
	mi.scale = scale_v
	parent.add_child(mi)
	return mi


func _build() -> void:
	_body_mat = _mat(_body_color, Color.BLACK, 0.18, 0.52)
	_wing_mat = _mat(_wing_color, Color.BLACK, 0.18, 0.50)
	var belly_mat := _mat(Color(0.20, 0.22, 0.26), Color.BLACK, 0.25, 0.55)
	var intake_mat := _mat(Color(0.07, 0.08, 0.10), Color.BLACK, 0.10, 0.75)
	var lens_mat := _mat(Color(0.08, 0.20, 0.32), Color(0.04, 0.15, 0.25), 0.85, 0.12)
	var spinner_mat := _mat(Color(0.84, 0.87, 0.90), Color.BLACK, 0.80, 0.22)
	var flap_mat := _mat(Color(0.62, 0.66, 0.70), Color.BLACK, 0.20, 0.50)
	var prop_mat := _mat(Color(0.14, 0.15, 0.17), Color.BLACK, 0.30, 0.55)
	var tip_mat := _mat(Color(0.95, 0.68, 0.12), Color(0.6, 0.4, 0.05), 0.1, 0.4)
	var led_left := _mat(Color(1.0, 0.15, 0.15), Color(1.0, 0.1, 0.1))
	var led_right := _mat(Color(0.15, 1.0, 0.35), Color(0.1, 1.0, 0.3))

	_disc_mat = StandardMaterial3D.new()
	_disc_mat.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	_disc_mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	_disc_mat.albedo_color = Color(0.82, 0.86, 0.90, 0.0)
	_disc_mat.cull_mode = BaseMaterial3D.CULL_DISABLED

	_cyl(self, 0.017, 0.019, 0.28, Vector3(-0.01, 0.002, 0.0), _body_mat, Vector3(0, 0, PI * 0.5))
	_box(self, Vector3(0.30, 0.010, 0.030), Vector3(0.0, -0.010, 0.0), belly_mat)

	_sphere(self, 0.025, Vector3(0.105, 0.009, 0.0), _body_mat, Vector3(2.4, 0.95, 0.90))
	_sphere(self, 0.018, Vector3(0.148, -0.001, 0.0), _body_mat, Vector3(1.6, 0.70, 0.85))
	_box(self, Vector3(0.065, 0.008, 0.026), Vector3(0.135, -0.008, 0.0), belly_mat, Vector3(0, 0, deg_to_rad(-6)))
	_cyl(self, 0.0015, 0.002, 0.032, Vector3(0.182, -0.002, 0.0), spinner_mat, Vector3(0, 0, PI * 0.5))

	_cyl(self, 0.009, 0.010, 0.012, Vector3(0.115, -0.016, 0.0), belly_mat)
	_sphere(self, 0.012, Vector3(0.115, -0.024, 0.0), belly_mat)
	_sphere(self, 0.007, Vector3(0.123, -0.025, 0.0), lens_mat)

	_sphere(self, 0.008, Vector3(0.035, 0.020, 0.0), _body_mat, Vector3(1.3, 0.65, 0.9))
	_box(self, Vector3(0.012, 0.016, 0.003), Vector3(0.012, 0.024, 0.0), _body_mat, Vector3(0, 0, deg_to_rad(-18)))
	_sphere(self, 0.020, Vector3(-0.125, 0.008, 0.0), _body_mat, Vector3(2.1, 0.90, 0.88))
	_box(self, Vector3(0.024, 0.010, 0.016), Vector3(-0.105, 0.021, 0.0), intake_mat, Vector3(0, 0, deg_to_rad(12)))
	_cyl(self, 0.004, 0.016, 0.028, Vector3(-0.168, 0.004, 0.0), spinner_mat, Vector3(0, 0, PI * 0.5))

	_box(self, Vector3(0.062, 0.009, 0.14), Vector3(-0.005, -0.003, 0.0), _wing_mat)
	_box(self, Vector3(0.048, 0.007, WINGSPAN * 0.48), Vector3(-0.010, 0.003, -WINGSPAN * 0.26), _wing_mat, Vector3(deg_to_rad(-2.5), deg_to_rad(3.5), 0))
	_box(self, Vector3(0.048, 0.007, WINGSPAN * 0.48), Vector3(-0.010, 0.003,  WINGSPAN * 0.26), _wing_mat, Vector3(deg_to_rad( 2.5), deg_to_rad(-3.5), 0))
	_box(self, Vector3(0.038, 0.008, 0.016), Vector3(-0.016, 0.009, -WINGSPAN * 0.49), belly_mat)
	_box(self, Vector3(0.038, 0.008, 0.016), Vector3(-0.016, 0.009,  WINGSPAN * 0.49), belly_mat)
	_box(self, Vector3(0.014, 0.008, 0.008), Vector3(-0.008, 0.010, -WINGSPAN * 0.505), led_left)
	_box(self, Vector3(0.014, 0.008, 0.008), Vector3(-0.008, 0.010,  WINGSPAN * 0.505), led_right)

	for side in [-1.0, 1.0]:
		var z_pod: float = side * 0.075
		_box(self, Vector3(0.035, 0.012, 0.005), Vector3(0.0, -0.011, z_pod), belly_mat)
		_cyl(self, 0.006, 0.006, 0.065, Vector3(0.005, -0.018, z_pod), _body_mat, Vector3(0, 0, PI * 0.5))
		_sphere(self, 0.0065, Vector3(0.038, -0.018, z_pod), belly_mat)

	_aileron_l = Node3D.new()
	_aileron_l.position = Vector3(-0.034, 0.005, -WINGSPAN * 0.33)
	add_child(_aileron_l)
	_box(_aileron_l, Vector3(0.016, 0.005, WINGSPAN * 0.26), Vector3(-0.008, 0, 0), flap_mat)

	_aileron_r = Node3D.new()
	_aileron_r.position = Vector3(-0.034, 0.005, WINGSPAN * 0.33)
	add_child(_aileron_r)
	_box(_aileron_r, Vector3(0.016, 0.005, WINGSPAN * 0.26), Vector3(-0.008, 0, 0), flap_mat)

	var vtail_span := 0.105
	_vtail_l_hinge = Node3D.new()
	_vtail_l_hinge.position = Vector3(-0.138, 0.012, -0.006)
	_vtail_l_hinge.rotation.x = -VTAIL_ANGLE
	add_child(_vtail_l_hinge)
	_box(_vtail_l_hinge, Vector3(0.040, vtail_span, 0.006), Vector3(0.0, vtail_span * 0.5, 0.0), _wing_mat)
	_box(_vtail_l_hinge, Vector3(0.036, vtail_span * 0.65, 0.0065), Vector3(0.002, vtail_span * 0.35, 0.001), belly_mat)
	_box(_vtail_l_hinge, Vector3(0.034, 0.006, 0.007), Vector3(0.0, vtail_span, 0.0), belly_mat)
	_vtail_flap_l = Node3D.new()
	_vtail_flap_l.position = Vector3(-0.020, vtail_span * 0.52, 0.0)
	_vtail_l_hinge.add_child(_vtail_flap_l)
	_box(_vtail_flap_l, Vector3(0.014, vtail_span * 0.88, 0.005), Vector3(-0.007, 0.0, 0.0), flap_mat)

	_vtail_r_hinge = Node3D.new()
	_vtail_r_hinge.position = Vector3(-0.138, 0.012, 0.006)
	_vtail_r_hinge.rotation.x = VTAIL_ANGLE
	add_child(_vtail_r_hinge)
	_box(_vtail_r_hinge, Vector3(0.040, vtail_span, 0.006), Vector3(0.0, vtail_span * 0.5, 0.0), _wing_mat)
	_box(_vtail_r_hinge, Vector3(0.036, vtail_span * 0.65, 0.0065), Vector3(0.002, vtail_span * 0.35, -0.001), belly_mat)
	_box(_vtail_r_hinge, Vector3(0.034, 0.006, 0.007), Vector3(0.0, vtail_span, 0.0), belly_mat)
	_vtail_flap_r = Node3D.new()
	_vtail_flap_r.position = Vector3(-0.020, vtail_span * 0.52, 0.0)
	_vtail_r_hinge.add_child(_vtail_flap_r)
	_box(_vtail_flap_r, Vector3(0.014, vtail_span * 0.88, 0.005), Vector3(-0.007, 0.0, 0.0), flap_mat)

	_box(self, Vector3(0.042, 0.046, 0.006), Vector3(-0.138, -0.028, 0.0), _wing_mat, Vector3(0, 0, deg_to_rad(-12)))
	_ventral_rudder = Node3D.new()
	_ventral_rudder.position = Vector3(-0.159, -0.028, 0.0)
	add_child(_ventral_rudder)
	_box(_ventral_rudder, Vector3(0.014, 0.040, 0.005), Vector3(-0.007, 0.0, 0.0), flap_mat)

	_prop_pivot = Node3D.new()
	_prop_pivot.position = Vector3(-0.178, 0.004, 0.0)
	add_child(_prop_pivot)
	_box(_prop_pivot, Vector3(0.005, PROP_LEN, 0.012), Vector3.ZERO, prop_mat)
	_box(_prop_pivot, Vector3(0.005, 0.012, PROP_LEN), Vector3.ZERO, prop_mat)
	_box(_prop_pivot, Vector3(0.0055, 0.014, 0.013), Vector3(0,  PROP_LEN * 0.46, 0), tip_mat)
	_box(_prop_pivot, Vector3(0.0055, 0.014, 0.013), Vector3(0, -PROP_LEN * 0.46, 0), tip_mat)
	_box(_prop_pivot, Vector3(0.0055, 0.013, 0.014), Vector3(0, 0,  PROP_LEN * 0.46), tip_mat)
	_box(_prop_pivot, Vector3(0.0055, 0.013, 0.014), Vector3(0, 0, -PROP_LEN * 0.46), tip_mat)

	_prop_disc = MeshInstance3D.new()
	var cyl := CylinderMesh.new()
	cyl.top_radius = PROP_LEN * 0.5
	cyl.bottom_radius = PROP_LEN * 0.5
	cyl.height = 0.002
	cyl.radial_segments = 24
	_prop_disc.mesh = cyl
	_prop_disc.material_override = _disc_mat
	_prop_disc.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	_prop_disc.position = _prop_pivot.position
	_prop_disc.rotation.z = PI * 0.5
	add_child(_prop_disc)

	_hit_sphere = MeshInstance3D.new()
	var sm := SphereMesh.new()
	sm.radius = 0.15
	sm.height = 0.30
	_hit_sphere.mesh = sm
	var hm := StandardMaterial3D.new()
	hm.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	hm.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	hm.albedo_color = Color(0.3, 0.8, 1.0, 0.15)
	hm.cull_mode = BaseMaterial3D.CULL_DISABLED
	_hit_sphere.material_override = hm
	_hit_sphere.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	_hit_sphere.visible = false
	add_child(_hit_sphere)


func set_collision_radius(r: float) -> void:
	var sm := _hit_sphere.mesh as SphereMesh
	sm.radius = r
	sm.height = r * 2.0


func toggle_collision_sphere() -> void:
	_hit_sphere.visible = not _hit_sphere.visible


func set_collided(v: bool) -> void:
	if v and not collided:
		_flash_t = 0.0
	collided = v
	if not v and _body_mat:
		_body_mat.albedo_color = _body_color
		_body_mat.emission_enabled = false


func _process(delta: float) -> void:
	var throttle := clampf(surfaces[0] if surfaces.size() >= 1 else thrust_ratio, 0.0, 1.5)
	var ail := clampf(surfaces[1] if surfaces.size() >= 2 else 0.0, -1.0, 1.0)
	var elev := clampf(surfaces[2] if surfaces.size() >= 3 else 0.0, -1.0, 1.0)
	var rud := clampf(surfaces[3] if surfaces.size() >= 4 else 0.0, -1.0, 1.0)

	var target_speed := 0.0 if collided else sqrt(throttle) * MAX_PROP_SPEED
	var cur: float = _prop_pivot.get_meta("speed", 0.0)
	cur = lerpf(cur, target_speed, 1.0 - exp(-delta * 6.0))
	_prop_pivot.set_meta("speed", cur)
	_prop_pivot.rotate_x(cur * delta)
	_disc_mat.albedo_color.a = clampf(cur / MAX_PROP_SPEED, 0.0, 1.0) * 0.35

	var max_defl := deg_to_rad(25.0)
	_aileron_l.rotation.z =  ail * max_defl
	_aileron_r.rotation.z = -ail * max_defl
	_vtail_flap_l.rotation.y = clampf(elev + rud, -1.2, 1.2) * max_defl
	_vtail_flap_r.rotation.y = clampf(-elev + rud, -1.2, 1.2) * max_defl
	_ventral_rudder.rotation.y = rud * max_defl

	if collided:
		_flash_t += delta
		var k := 0.5 + 0.5 * sin(_flash_t * 12.0)
		_body_mat.albedo_color = _body_color.lerp(Color(1, 0.1, 0.05), k)
		_body_mat.emission_enabled = true
		_body_mat.emission = Color(1, 0.1, 0.0) * k
