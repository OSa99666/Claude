extends Node3D
## Rolling-table user interface + 3D view.
##
## All physics runs in the Python simulator.  This program
##  * receives the state as JSON over UDP (default port 9870) and draws it,
##  * sends the parameter/command values from the panel back to the simulator
##    (UDP, port 9871, to the address the state packets come from).
##
## Simulator frame -> Godot frame:  (x, y, z) -> (x, z, -y)
## Tilt convention: tilt_x > 0 => surface slopes down towards +X (sim frame),
##                  tilt_y > 0 => surface slopes down towards +Y.
##
## Run:  godot --path godot -- [--port 9870] [--cmd-port 9871]

var port := 9870
var cmd_port := 9871
var udp := PacketPeerUDP.new()      # receives state
var cmd := PacketPeerUDP.new()      # sends UI values
var sim_host := ""

var table := Node3D.new()
var ball := MeshInstance3D.new()
var walls: Array[MeshInstance3D] = []
var cam := Camera3D.new()
var hud := Label.new()
var panel := PanelContainer.new()
var controls := {}                  # key -> SpinBox / CheckBox
var center_seq := 0
var ui_ready := false
var send_timer := 0.0

var built_L := -1.0
var built_R := -1.0
var built_wh := -1.0
var built_wt := -1.0
var msg := {}
var last_rx_ms := -100000

# orbit camera
var yaw := 0.7
var pitch := 0.65
var dist := 4.8
var target := Vector3.ZERO


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
	sun.shadow_enabled = true
	add_child(sun)

	add_child(table)
	add_child(cam)
	cam.current = true

	var layer := CanvasLayer.new()
	hud.position = Vector2(12, 8)
	hud.add_theme_font_size_override("font_size", 16)
	layer.add_child(hud)
	_add_help(layer)
	_build_panel(layer)
	add_child(layer)

	_build(2.0, 0.06, 0.12, 0.003)


# ------------------------------------------------------------------ UI panel
func _build_panel(layer: CanvasLayer) -> void:
	panel.anchor_left = 1.0
	panel.anchor_right = 1.0
	panel.anchor_top = 0.0
	panel.anchor_bottom = 0.0
	panel.offset_left = -400
	panel.offset_right = -10
	panel.offset_top = 10
	panel.grow_horizontal = Control.GROW_DIRECTION_BEGIN
	panel.visible = false
	layer.add_child(panel)

	var box := VBoxContainer.new()
	panel.add_child(box)

	_heading(box, "Dynamics")
	_spin(box, "Elasticity (<0: no walls)", -1.0, 1.0, 0.05, "elasticity")
	_spin(box, "Static friction", 0.0, 1.5, 0.01, "mu_s")
	_spin(box, "Dynamic friction", 0.0, 1.5, 0.01, "mu_k")
	_heading(box, "Velocity range [m/s]  (raw scaling)")
	_spin(box, "Min velocity", -5.0, 0.0, 0.05, "vel_min")
	_spin(box, "Max velocity", 0.0, 5.0, 0.05, "vel_max")
	_check(box, "Limit ball velocity to range", "limit_velocity")
	_heading(box, "Forced velocity [m/s]")
	_spin(box, "Velocity setpoint X", -3.0, 3.0, 0.05, "vel_sp_x")
	_check(box, "Set velocity X  (RX11)", "hold_vx")
	_spin(box, "Velocity setpoint Y", -3.0, 3.0, 0.05, "vel_sp_y")
	_check(box, "Set velocity Y  (RX21)", "hold_vy")
	_heading(box, "Forced position [m]")
	_spin(box, "Distance setpoint X", -1.5, 1.5, 0.05, "dist_sp_x")
	_check(box, "Set distance X  (RX12)", "hold_x")
	_spin(box, "Distance setpoint Y", -1.5, 1.5, 0.05, "dist_sp_y")
	_check(box, "Set distance Y  (RX22)", "hold_y")
	var b := Button.new()
	b.text = "Center ball  (RXBL)"
	b.pressed.connect(_on_center)
	box.add_child(b)


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
	# initial values come from the simulator, without triggering a send
	for k in p:
		if controls.has(k):
			var c = controls[k]
			if c is SpinBox:
				c.set_value_no_signal(float(p[k]))
			elif c is CheckBox:
				c.set_pressed_no_signal(bool(p[k]))
	panel.visible = true
	ui_ready = true


func _on_center() -> void:
	center_seq += 1
	_send_ui()


func _send_ui() -> void:
	if not ui_ready or sim_host == "":
		return
	var d := {"center_seq": center_seq}
	for k in controls:
		var c = controls[k]
		d[k] = c.value if c is SpinBox else c.button_pressed
	cmd.put_packet(JSON.stringify(d).to_utf8_buffer())


# --------------------------------------------------------------- scene build
func _make_ball_material() -> StandardMaterial3D:
	# checker pattern so rolling and spin are visible
	var img := Image.create(64, 32, false, Image.FORMAT_RGB8)
	for yy in 32:
		for xx in 64:
			var on := (floori(xx / 8.0) + floori(yy / 8.0)) % 2 == 0
			img.set_pixel(xx, yy, Color(0.9, 0.25, 0.2) if on else Color(0.95, 0.95, 0.95))
	var m := StandardMaterial3D.new()
	m.albedo_texture = ImageTexture.create_from_image(img)
	m.roughness = 0.35
	return m


func _box(size: Vector3, pos: Vector3, color: Color) -> MeshInstance3D:
	var mi := MeshInstance3D.new()
	var bm := BoxMesh.new()
	bm.size = size
	mi.mesh = bm
	mi.position = pos
	var mat := StandardMaterial3D.new()
	mat.albedo_color = color
	mat.roughness = 0.8
	mi.material_override = mat
	return mi


func _build(L: float, R: float, wh: float, t: float) -> void:
	for c in table.get_children():
		if c != ball:
			c.queue_free()
	walls.clear()
	built_L = L
	built_R = R
	built_wh = wh
	built_wt = t                       # wall thickness (config: [table] wall_thickness_m)
	var wc := Color(0.35, 0.37, 0.42)
	table.add_child(_box(Vector3(L + 2 * t, 0.04, L + 2 * t), Vector3(0, -0.02, 0), Color(0.78, 0.74, 0.62)))
	walls.append(_box(Vector3(t, wh, L + 2 * t), Vector3(L / 2 + t / 2, wh / 2, 0), wc))
	walls.append(_box(Vector3(t, wh, L + 2 * t), Vector3(-L / 2 - t / 2, wh / 2, 0), wc))
	walls.append(_box(Vector3(L, wh, t), Vector3(0, wh / 2, L / 2 + t / 2), wc))
	walls.append(_box(Vector3(L, wh, t), Vector3(0, wh / 2, -L / 2 - t / 2), wc))
	for w in walls:
		table.add_child(w)
	# centre cross
	table.add_child(_box(Vector3(0.2, 0.004, 0.01), Vector3(0, 0.002, 0), Color(0.2, 0.2, 0.2)))
	table.add_child(_box(Vector3(0.01, 0.004, 0.2), Vector3(0, 0.002, 0), Color(0.2, 0.2, 0.2)))

	var sm := SphereMesh.new()
	sm.radius = R
	sm.height = 2.0 * R
	ball.mesh = sm
	ball.material_override = _make_ball_material()
	if ball.get_parent() == null:
		table.add_child(ball)
	ball.position = Vector3(0, R, 0)
	dist = 2.4 * L
	_update_camera()
	_save_home()


# ------------------------------------------------------------------- per frame
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
		ui_ready = false      # re-read parameters if the simulator restarted
		return

	send_timer += delta
	if send_timer > 0.2:      # periodic resend, so lost packets or a restarted simulator recover
		send_timer = 0.0
		_send_ui()

	var L := float(msg.get("L", 2.0))
	var R := float(msg.get("R", 0.06))
	var wh := float(msg.get("wh", 0.12))
	var wt := float(msg.get("wt", 0.003))
	if absf(L - built_L) > 1e-6 or absf(R - built_R) > 1e-6 or absf(wh - built_wh) > 1e-6 or absf(wt - built_wt) > 1e-6:
		_build(L, R, wh, wt)
	var walls_on := bool(msg.get("walls", true))
	for w in walls:
		w.visible = walls_on

	# table orientation from the two slopes
	var tx := tan(float(msg["tx"]))
	var ty := tan(float(msg["ty"]))
	var xa := Vector3(1, -tx, 0).normalized()
	var za := Vector3(0, ty, 1).normalized()
	var ya := za.cross(xa).normalized()
	table.basis = Basis(xa, ya, za).orthonormalized()

	# ball (child of the table, table-local coordinates; z < 0 = falling off)
	ball.position = Vector3(float(msg["x"]), R + float(msg.get("z", 0.0)), -float(msg["y"]))
	var q: Array = msg["q"]   # w, x, y, z in the simulator frame
	ball.quaternion = Quaternion(float(q[1]), float(q[3]), -float(q[2]), float(q[0])).normalized()

	var raw: Dictionary = msg.get("raw", {})
	var tr: Array = msg.get("tilt_raw", [0, 0])
	var state := "BALL OFF TABLE" if msg.get("off", false) else ("slipping" if msg.get("slip", false) else "rolling")
	hud.text = "ball  x %+6.3f m   y %+6.3f m   [%s]\ntilt  X %+5.1f deg   Y %+5.1f deg   walls: %s\nOPC UA clients: %d%s\n\nF_YR10 %6d   F_YR20 %6d\nF_XG10 %6d   F_XG11 %6d   F_XG12 %6d\nF_XG20 %6d   F_XG21 %6d   F_XG22 %6d" % [
		float(msg["x"]), float(msg["y"]), state,
		rad_to_deg(float(msg["tx"])), rad_to_deg(float(msg["ty"])),
		"on" if walls_on else "REMOVED",
		int(msg.get("clients", 0)), "" if msg.get("plc", false) else "   (no PLC - table levelled)",
		int(tr[0]), int(tr[1]),
		int(raw.get("F_XG10", 0)), int(raw.get("F_XG11", 0)), int(raw.get("F_XG12", 0)),
		int(raw.get("F_XG20", 0)), int(raw.get("F_XG21", 0)), int(raw.get("F_XG22", 0))]


func _unhandled_input(ev: InputEvent) -> void:
	if _camera_event(ev):
		return
	if ev is InputEventMouseMotion and (ev.button_mask & MOUSE_BUTTON_MASK_LEFT):
		yaw -= ev.relative.x * 0.005
		pitch = clampf(pitch + ev.relative.y * 0.005, 0.1, 1.5)
		_update_camera()
	elif ev is InputEventMouseButton and ev.pressed:
		if ev.button_index == MOUSE_BUTTON_WHEEL_UP:
			dist = maxf(0.5, dist * 0.92)
			_update_camera()
		elif ev.button_index == MOUSE_BUTTON_WHEEL_DOWN:
			dist = minf(30.0, dist * 1.08)
			_update_camera()


func _update_camera() -> void:
	var p := target + Vector3(sin(yaw) * cos(pitch), sin(pitch), cos(yaw) * cos(pitch)) * dist
	cam.look_at_from_position(p, target, Vector3.UP)


# ------------------------------------------------------- camera: keyboard + middle mouse
## Arrow keys pan the view (the scene moves in the arrow's direction), Shift+arrows orbit, +/- (or PageUp/Down)
## zoom, Home resets the view; mouse: left drag orbits, middle drag pans, wheel zooms.
const CAM_PITCH_MIN := 0.1
const CAM_DIST_MIN := 0.5
const CAM_DIST_MAX := 30.0
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
