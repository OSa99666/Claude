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
var leds: Array[MeshInstance3D] = []     # V_YH, V_YL (command row), V_XGH, V_XGL (feedback row)
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

# dimmed palette shared with the transport simulation (same ARGB item colours / metal)
const METAL := Color(0.6, 0.63, 0.68)
const WATER := Color8(0x00, 0x00, 0x65)         # dark blue
const DISC := Color8(0x65, 0x65, 0x00)          # dark yellow
const LEVER := Color8(0x65, 0x00, 0x00)         # dark red
const PARTICLE := Color(0.55, 0.65, 0.85)
const LED_HIGH := Color8(0x00, 0x65, 0x00)      # dark green: High / active / open
const LED_LOW := Color8(0x65, 0x65, 0x00)       # dark yellow: Low / passive / closed

var yaw := 0.6
var pitch := 0.35
var dist := 4.2
var target := Vector3(0, 0.3, 0)


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
	_save_home()

	var layer := CanvasLayer.new()
	hud.position = Vector2(12, 8)
	hud.add_theme_font_size_override("font_size", 16)
	layer.add_child(hud)
	_add_help(layer)
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
	var steel := _mat(METAL, 1.0, 0.5)
	var glass := _mat(Color(0.75, 0.85, 0.95), 0.25)
	# pipe along X (cylinders are along Y, rotate 90 deg about Z); left, right, transparent middle
	_cyl(0.30, 1.6, Vector3(-1.4, 0, 0), steel, Vector3(0, 0, 90))
	_cyl(0.30, 1.6, Vector3(1.4, 0, 0), steel, Vector3(0, 0, 90))
	_cyl(0.30, 1.2, Vector3(0, 0, 0), glass, Vector3(0, 0, 90))
	for x in [-0.6, 0.6]:
		_cyl(0.38, 0.08, Vector3(x, 0, 0), steel, Vector3(0, 0, 90))   # flanges
	# upstream water (always) and downstream water (follows the opening)
	_cyl(0.26, 1.6, Vector3(-1.4, 0, 0), _mat(WATER), Vector3(0, 0, 90))
	down_water.mesh = CylinderMesh.new()
	(down_water.mesh as CylinderMesh).top_radius = 0.26
	(down_water.mesh as CylinderMesh).bottom_radius = 0.26
	(down_water.mesh as CylinderMesh).height = 1.6
	down_water_mat.albedo_color = WATER
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
	fi.material_override = _mat(PARTICLE)
	add_child(fi)
	# stem + actuator housing
	_cyl(0.05, 0.9, Vector3(0, 0.55, 0), steel)
	_cyl(0.16, 0.3, Vector3(0, 0.95, 0), _mat(Color(0.25, 0.3, 0.4), 1.0, 0.3))
	# four indicator LEDs on the front of the actuator: command row (V_YH, V_YL) above, feedback row (V_XGH, V_XGL) below
	for row in 2:
		var y := 1.03 - 0.11 * row
		for col in 2:
			var x := -0.06 + 0.12 * col
			var led := MeshInstance3D.new()
			var led_mesh := SphereMesh.new()
			led_mesh.radius = 0.022
			led_mesh.height = 0.044
			led.mesh = led_mesh
			led.material_override = _mat(METAL, 0.5)
			led.position = Vector3(x, y, sqrt(0.16 * 0.16 - x * x) + 0.008)
			add_child(led)
			leds.append(led)
		var lab := Label3D.new()
		lab.text = "V_YH / V_YL" if row == 0 else "V_XGH / V_XGL"
		lab.font_size = 28
		lab.pixel_size = 0.002
		lab.billboard = BaseMaterial3D.BILLBOARD_ENABLED
		lab.modulate = Color(0.6, 0.65, 0.75)
		lab.horizontal_alignment = HORIZONTAL_ALIGNMENT_LEFT
		lab.position = Vector3(0.2, y, 0.1)
		add_child(lab)
	# disc (flat cylinder); rotates about the vertical stem
	var dm := CylinderMesh.new()
	dm.top_radius = 0.28
	dm.bottom_radius = 0.28
	dm.height = 0.03
	var dmi := MeshInstance3D.new()
	dmi.mesh = dm
	dmi.material_override = _mat(DISC, 1.0, 0.4)
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
	bar.material_override = _mat(LEVER)
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


## LED: coloured (and glowing) when on, metal when off
func _led(m: MeshInstance3D, on: bool, colour: Color) -> void:
	var mat := m.material_override as StandardMaterial3D
	mat.albedo_color = colour if on else METAL
	mat.emission_enabled = on
	mat.emission = colour
	mat.emission_energy_multiplier = 1.5


func _lamp(on: bool) -> String:
	return "[###]" if on else "[   ]"


# -------------------------------------------------------------------- update
func _process(delta: float) -> void:
	_camera_keys(delta)
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
	down_water_mat.albedo_color = WATER.lerp(Color(0.25, 0.27, 0.3), 1.0 - clampf((pos - 0.1) / 0.9, 0.0, 1.0))

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
	_led(leds[0], bool(msg["yh"]), LED_HIGH)
	_led(leds[1], bool(msg["yl"]), LED_LOW)
	_led(leds[2], xgh, LED_HIGH)
	_led(leds[3], xgl, LED_LOW)
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
	if _camera_event(ev):
		return
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
	var p := target + Vector3(sin(yaw) * cos(pitch), sin(pitch), cos(yaw) * cos(pitch)) * dist
	cam.look_at_from_position(p, target, Vector3.UP)


# ------------------------------------------------------- camera: keyboard + middle mouse
## Arrow keys pan the view (the scene moves in the arrow's direction), Shift+arrows orbit, +/- (or PageUp/Down)
## zoom, Home resets the view; mouse: left drag orbits, middle drag pans, wheel zooms.
const CAM_PITCH_MIN := -0.2
const CAM_DIST_MIN := 1.0
const CAM_DIST_MAX := 20.0
var keys_armed := {}                # keys whose press reached us (not consumed by a focused UI control)
var home := {}


func _save_home() -> void:
	home = {"yaw": yaw, "pitch": pitch, "dist": dist, "target": target}


func _key_down(k: int) -> bool:
	return bool(keys_armed.get(k, false)) and Input.is_key_pressed(k as Key)


func _camera_keys(delta: float) -> void:
	var dx := 0.0
	var dy := 0.0
	if _key_down(KEY_LEFT):
		dx -= 1.0
	if _key_down(KEY_RIGHT):
		dx += 1.0
	if _key_down(KEY_UP):
		dy -= 1.0
	if _key_down(KEY_DOWN):
		dy += 1.0
	var zoom := 0.0
	if _key_down(KEY_EQUAL) or _key_down(KEY_KP_ADD) or _key_down(KEY_PLUS) or _key_down(KEY_PAGEUP):
		zoom -= 1.0
	if _key_down(KEY_MINUS) or _key_down(KEY_KP_SUBTRACT) or _key_down(KEY_PAGEDOWN):
		zoom += 1.0
	if dx == 0.0 and dy == 0.0 and zoom == 0.0:
		return
	if Input.is_key_pressed(KEY_SHIFT):
		yaw -= dx * 1.5 * delta
		pitch = clampf(pitch + dy * 1.5 * delta, CAM_PITCH_MIN, 1.5)
	else:
		var right := cam.global_transform.basis.x
		var up := cam.global_transform.basis.y
		target += (up * dy - right * dx) * dist * 0.5 * delta
	dist = clampf(dist * exp(zoom * 1.2 * delta), CAM_DIST_MIN, CAM_DIST_MAX)
	_update_camera()


func _pan_drag(rel: Vector2) -> void:
	var k := dist * 0.0015
	target += (cam.global_transform.basis.y * rel.y - cam.global_transform.basis.x * rel.x) * k
	_update_camera()


func _camera_event(ev: InputEvent) -> bool:
	if ev is InputEventKey:
		if ev.pressed:
			keys_armed[ev.keycode] = true
			if ev.keycode == KEY_HOME and not ev.echo and not home.is_empty():
				yaw = home["yaw"]
				pitch = home["pitch"]
				dist = home["dist"]
				target = home["target"]
				_update_camera()
		return true
	if ev is InputEventMouseMotion and (ev.button_mask & MOUSE_BUTTON_MASK_MIDDLE):
		_pan_drag(ev.relative)
		return true
	return false


func _add_help(layer: CanvasLayer) -> void:
	var help := Label.new()
	help.text = "mouse: drag orbit, middle-drag pan, wheel zoom   |   keys: arrows pan, Shift+arrows orbit, +/- zoom, Home reset"
	help.add_theme_font_size_override("font_size", 12)
	help.add_theme_color_override("font_color", Color(0.6, 0.65, 0.75))
	help.anchor_top = 1.0
	help.anchor_bottom = 1.0
	help.offset_left = 12
	help.offset_top = -24
	layer.add_child(help)
