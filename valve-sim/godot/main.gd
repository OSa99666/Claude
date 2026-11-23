extends Node3D
## Valve simulation: user interface + 3D view.
##
## All logic runs in the Python simulator (valvesim).  This program
##  * receives the state as JSON over UDP (default port 9872) and draws it,
##  * sends the parameter/command values from the panel back to the simulator
##    (UDP, port 9873, to the address the state packets come from).
##
## Valve: butterfly valve in a pipe along X.  pos 0 = Low (passive/closed, disc across the pipe),
## pos 1 = High (active/open, disc along the pipe).
##
## Run:  godot --path godot -- [--port 9872] [--cmd-port 9873]

var port := 9872
var cmd_port := 9873
var udp := PacketPeerUDP.new()
var cmd := PacketPeerUDP.new()
var sim_host := ""

var disc := Node3D.new()           # pivot: rotates about the vertical stem
var lever := Node3D.new()
var down_water := MeshInstance3D.new()
var down_water_mat := StandardMaterial3D.new()
var cam := Camera3D.new()
# animated fluid: small spheres drifting along the pipe, speed follows the valve opening
const N_PART := 90
const PIPE_HALF := 2.2
var flow_mm := MultiMesh.new()
var part_x := PackedFloat32Array()
var part_y := PackedFloat32Array()
var part_z := PackedFloat32Array()

var hud := Label.new()
var panel := PanelContainer.new()
var controls := {}
var to_low_seq := 0
var to_high_seq := 0
var ui_ready := false
var send_timer := 0.0
var msg := {}
var last_rx_ms := -100000

var yaw := 0.6
var pitch := 0.35
var dist := 4.2


func _ready() -> void:
	var args := OS.get_cmdline_user_args()
	for i in args.size():
		if args[i] == "--port" and i + 1 < args.size():
			port = int(args[i + 1])
		elif args[i] == "--cmd-port" and i + 1 < args.size():
			cmd_port = int(args[i + 1])
	var err := udp.bind(port)
	if err != OK:
		push_error("cannot bind UDP port %d (error %d)" % [port, err])

	var env := Environment.new()
	env.background_mode = Environment.BG_COLOR
	env.background_color = Color(0.12, 0.13, 0.16)
	env.ambient_light_source = Environment.AMBIENT_SOURCE_COLOR
	env.ambient_light_color = Color(0.55, 0.57, 0.62)
	var we := WorldEnvironment.new()
	we.environment = env
	add_child(we)

	var sun := DirectionalLight3D.new()
	sun.rotation_degrees = Vector3(-55, 35, 0)
	add_child(sun)

	_build_valve()
	add_child(cam)
	cam.current = true
	_update_camera()

	var layer := CanvasLayer.new()
	hud.position = Vector2(12, 8)
	hud.add_theme_font_size_override("font_size", 16)
	layer.add_child(hud)
	_build_panel(layer)
	add_child(layer)


# ------------------------------------------------------------------ 3D scene
func _mat(c: Color, alpha := 1.0, metal := 0.0) -> StandardMaterial3D:
	var m := StandardMaterial3D.new()
	m.albedo_color = Color(c.r, c.g, c.b, alpha)
	m.metallic = metal
	m.roughness = 0.5
	if alpha < 1.0:
		m.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	return m


func _cyl(r: float, h: float, pos: Vector3, mat: Material, rot := Vector3.ZERO) -> MeshInstance3D:
	var mi := MeshInstance3D.new()
	var m := CylinderMesh.new()
	m.top_radius = r
	m.bottom_radius = r
	m.height = h
	m.radial_segments = 48
	mi.mesh = m
	mi.material_override = mat
	mi.position = pos
	mi.rotation_degrees = rot
	add_child(mi)
	return mi


func _build_valve() -> void:
	var steel := _mat(Color(0.62, 0.65, 0.7), 1.0, 0.6)
	var glass := _mat(Color(0.75, 0.85, 0.95), 0.25)
	# pipe along X (cylinders are along Y, rotate 90 deg about Z); left, right, transparent middle
	_cyl(0.30, 1.6, Vector3(-1.4, 0, 0), steel, Vector3(0, 0, 90))
	_cyl(0.30, 1.6, Vector3(1.4, 0, 0), steel, Vector3(0, 0, 90))
	_cyl(0.30, 1.2, Vector3(0, 0, 0), glass, Vector3(0, 0, 90))
	for x in [-0.6, 0.6]:
		_cyl(0.38, 0.08, Vector3(x, 0, 0), steel, Vector3(0, 0, 90))   # flanges
	# upstream water (always) and downstream water (follows the opening)
	_cyl(0.26, 1.6, Vector3(-1.4, 0, 0), _mat(Color(0.2, 0.5, 0.95)), Vector3(0, 0, 90))
	down_water.mesh = CylinderMesh.new()
	(down_water.mesh as CylinderMesh).top_radius = 0.26
	(down_water.mesh as CylinderMesh).bottom_radius = 0.26
	(down_water.mesh as CylinderMesh).height = 1.6
	down_water_mat.albedo_color = Color(0.2, 0.5, 0.95)
	down_water.material_override = down_water_mat
	down_water.position = Vector3(1.4, 0, 0)
	down_water.rotation_degrees = Vector3(0, 0, 90)
	add_child(down_water)
	# flow particles
	flow_mm.transform_format = MultiMesh.TRANSFORM_3D
	var sm := SphereMesh.new()
	sm.radius = 0.035
	sm.height = 0.07
	sm.radial_segments = 8
	sm.rings = 4
	flow_mm.mesh = sm
	flow_mm.instance_count = N_PART
	var rng := RandomNumberGenerator.new()
	rng.seed = 7
	for i in N_PART:
		part_x.append(-PIPE_HALF + 2.0 * PIPE_HALF * float(i) / N_PART)
		var r := 0.2 * sqrt(rng.randf())
		var a := rng.randf() * TAU
		part_y.append(r * sin(a))
		part_z.append(r * cos(a))
	var fi := MultiMeshInstance3D.new()
	fi.multimesh = flow_mm
	fi.material_override = _mat(Color(0.75, 0.9, 1.0))
	add_child(fi)
	# stem + actuator housing
	_cyl(0.05, 0.9, Vector3(0, 0.55, 0), steel)
	_cyl(0.16, 0.3, Vector3(0, 0.95, 0), _mat(Color(0.25, 0.3, 0.4), 1.0, 0.3))
	# disc (flat cylinder); rotates about the vertical stem
	var dm := CylinderMesh.new()
	dm.top_radius = 0.28
	dm.bottom_radius = 0.28
	dm.height = 0.03
	var dmi := MeshInstance3D.new()
	dmi.mesh = dm
	dmi.material_override = _mat(Color(0.95, 0.75, 0.2), 1.0, 0.4)
	dmi.rotation_degrees = Vector3(0, 0, 90)   # disc plane perpendicular to the flow (X) = closed
	disc.add_child(dmi)
	add_child(disc)
	# indicator lever on top of the actuator
	lever.position = Vector3(0, 1.13, 0)
	add_child(lever)
	var bar := MeshInstance3D.new()
	var bm := BoxMesh.new()
	bm.size = Vector3(0.08, 0.05, 0.7)
	bar.mesh = bm
	bar.material_override = _mat(Color(0.9, 0.2, 0.2))
	lever.add_child(bar)


# ---------------------------------------------------------------------- panel
func _build_panel(layer: CanvasLayer) -> void:
	panel.anchor_left = 1.0
	panel.anchor_right = 1.0
	panel.offset_left = -400
	panel.offset_right = -10
	panel.offset_top = 10
	panel.grow_horizontal = Control.GROW_DIRECTION_BEGIN
	panel.visible = false
	layer.add_child(panel)
	var box := VBoxContainer.new()
	panel.add_child(box)

	_heading(box, "Transit time [s]  (full stroke)")
	_spin(box, "To High  (PTDTH)", 0.05, 120.0, 0.1, "t_to_high")
	_spin(box, "To Low  (PTDTL)", 0.05, 120.0, 0.1, "t_to_low")
	_heading(box, "Action delay before moving [s]")
	_spin(box, "Leaving Low  (PADFL)", 0.0, 30.0, 0.1, "ad_from_low")
	_spin(box, "Leaving High  (PADFH)", 0.0, 30.0, 0.1, "ad_from_high")
	_check(box, "Travel to end (stroke completes if command removed)", "travel_to_end")
	_heading(box, "Safeguarding (forces that state, ignores commands)")
	_check(box, "Safeguard High  (FSH)", "safeguard_high")
	_check(box, "Safeguard Low  (FSL)", "safeguard_low")
	_heading(box, "Force feedbacks (fault injection)")
	_check(box, "Force V_XGH / V_XGL true  (FH)", "force_true")
	_check(box, "Force V_XGH / V_XGL false  (FL)", "force_false")
	_heading(box, "Set valve position directly")
	var row := HBoxContainer.new()
	var b1 := Button.new()
	b1.text = "Set to Low"
	b1.pressed.connect(func() -> void:
		to_low_seq += 1
		_send_ui())
	var b2 := Button.new()
	b2.text = "Set to High"
	b2.pressed.connect(func() -> void:
		to_high_seq += 1
		_send_ui())
	row.add_child(b1)
	row.add_child(b2)
	box.add_child(row)


func _heading(parent: Control, text: String) -> void:
	var l := Label.new()
	l.text = text
	l.add_theme_color_override("font_color", Color(0.75, 0.85, 1.0))
	parent.add_child(l)


func _spin(parent: Control, text: String, lo: float, hi: float, step: float, key: String) -> void:
	var row := HBoxContainer.new()
	var l := Label.new()
	l.text = text
	l.custom_minimum_size = Vector2(230, 0)
	var sb := SpinBox.new()
	sb.min_value = lo
	sb.max_value = hi
	sb.step = step
	sb.value_changed.connect(func(_v: float) -> void: _send_ui())
	row.add_child(l)
	row.add_child(sb)
	parent.add_child(row)
	controls[key] = sb


func _check(parent: Control, text: String, key: String) -> void:
	var cb := CheckBox.new()
	cb.text = text
	cb.toggled.connect(func(_on: bool) -> void: _send_ui())
	parent.add_child(cb)
	controls[key] = cb


func _fill_ui(p: Dictionary) -> void:
	for k in p:
		if controls.has(k):
			var c = controls[k]
			if c is SpinBox:
				c.set_value_no_signal(float(p[k]))
			elif c is CheckBox:
				c.set_pressed_no_signal(bool(p[k]))
	panel.visible = true
	ui_ready = true


func _send_ui() -> void:
	if not ui_ready or sim_host == "":
		return
	var d := {"to_low_seq": to_low_seq, "to_high_seq": to_high_seq}
	for k in controls:
		var c = controls[k]
		d[k] = c.value if c is SpinBox else c.button_pressed
	cmd.put_packet(JSON.stringify(d).to_utf8_buffer())


func _lamp(on: bool) -> String:
	return "[###]" if on else "[   ]"


# -------------------------------------------------------------------- update
func _process(delta: float) -> void:
	while udp.get_available_packet_count() > 0:
		var pkt := udp.get_packet()
		var ip := udp.get_packet_ip()
		var parsed = JSON.parse_string(pkt.get_string_from_utf8())
		if parsed is Dictionary:
			msg = parsed
			last_rx_ms = Time.get_ticks_msec()
			if ip != sim_host:
				sim_host = ip
				cmd.close()
				cmd.set_dest_address(sim_host, cmd_port)
			if not ui_ready and msg.has("params"):
				_fill_ui(msg["params"])

	var live := Time.get_ticks_msec() - last_rx_ms < 1000
	if msg.is_empty() or not live:
		hud.text = "Waiting for simulator on UDP port %d ..." % port
		panel.visible = false
		ui_ready = false
		return

	send_timer += delta
	if send_timer > 0.2:
		send_timer = 0.0
		_send_ui()

	var pos := float(msg["pos"])
	# disc: across the pipe at Low (0 deg), turned 90 deg along the pipe at High
	disc.rotation_degrees = Vector3(0, 90.0 * pos, 0)
	lever.rotation_degrees = Vector3(0, 90.0 * pos, 0)   # same direction as the disc: bar parallel to the disc plane
	down_water_mat.albedo_color = Color(0.2, 0.5, 0.95, 1.0).lerp(Color(0.25, 0.27, 0.3), 1.0 - clampf((pos - 0.1) / 0.9, 0.0, 1.0))

	# flow: ~ opening of the butterfly (sin of the disc angle), particles stop when closed
	var flow := sin(pos * PI / 2.0)
	for i in N_PART:
		part_x[i] += flow * 1.6 * delta
		if part_x[i] > PIPE_HALF:
			part_x[i] -= 2.0 * PIPE_HALF
		# downstream of the disc only while there is flow; no fluid left behind a closed valve
		var vis := 1.0 if (part_x[i] < 0.0 or flow > 0.03) else 0.0
		flow_mm.set_instance_transform(i, Transform3D(Basis().scaled(Vector3.ONE * vis), Vector3(part_x[i], part_y[i], part_z[i])))

	var xgh := bool(msg["xgh"])
	var xgl := bool(msg["xgl"])
	var state := "HIGH (open)" if pos >= 1.0 else ("LOW (closed)" if pos <= 0.0 else "travelling")
	if bool(msg.get("waiting", false)):
		state = "action delay ..."
	var forced := ""
	var p: Dictionary = msg.get("params", {})
	if bool(p.get("force_false", false)):
		forced = "   FEEDBACKS FORCED FALSE"
	elif bool(p.get("force_true", false)):
		forced = "   FEEDBACKS FORCED TRUE"
	hud.text = "valve  %3d %%   [%s]%s\nOPC UA clients: %d%s\n\nV_YH  %s  command High      V_XGH %s  is High\nV_YL  %s  command Low       V_XGL %s  is Low" % [
		roundi(pos * 100.0), state, forced,
		int(msg.get("clients", 0)), "" if msg.get("plc", false) else "   (no PLC - commands ignored)",
		_lamp(bool(msg["yh"])), _lamp(xgh), _lamp(bool(msg["yl"])), _lamp(xgl)]


func _unhandled_input(ev: InputEvent) -> void:
	if ev is InputEventMouseMotion and (ev.button_mask & MOUSE_BUTTON_MASK_LEFT):
		yaw -= ev.relative.x * 0.005
		pitch = clampf(pitch + ev.relative.y * 0.005, -0.2, 1.5)
		_update_camera()
	elif ev is InputEventMouseButton and ev.pressed:
		if ev.button_index == MOUSE_BUTTON_WHEEL_UP:
			dist = maxf(1.0, dist * 0.92)
			_update_camera()
		elif ev.button_index == MOUSE_BUTTON_WHEEL_DOWN:
			dist = minf(20.0, dist * 1.08)
			_update_camera()


func _update_camera() -> void:
	var p := Vector3(sin(yaw) * cos(pitch), sin(pitch), cos(yaw) * cos(pitch)) * dist
	cam.look_at_from_position(p, Vector3(0, 0.3, 0), Vector3.UP)
