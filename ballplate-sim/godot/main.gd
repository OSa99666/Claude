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
# servo mechanism below the plate: base, central pivot, two servos with horn + connecting rod to the plate
var mech := Node3D.new()                # static parts (rebuilt with the table)
var lbl_x := Label3D.new()              # servo angle read-outs
var lbl_y := Label3D.new()
var seqs := {}                          # one-shot button counters (keys end with _seq)
# markers on the plate (children of the table, so they tilt with it)
var tgt_ring := MeshInstance3D.new()
var tgt_dot := MeshInstance3D.new()
var tgt_zone_built := -1.0
var patch_disc := MeshInstance3D.new()
var patch_r_built := -1.0
var trail_mesh := MeshInstance3D.new()
var trail_pts: Array[Vector3] = []
var trail_timer := 0.0
var mech_geo := {}                      # D (attachment distance), a (horn length), sy (shaft height), rod (rod length)
var horn_x := Node3D.new()              # rotating horn of servo X (tilt about the Z axis)
var horn_y := Node3D.new()              # rotating horn of servo Y (tilt about the X axis)
var rod_x := MeshInstance3D.new()
var rod_y := MeshInstance3D.new()
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
# diagnostics (F3): frame rate and state-packet timing as seen by Godot
var diag := false
var diag_last_pkt_ms := 0
var diag_gap_max := 0
var diag_pkts := 0
var diag_time := 0.0
var diag_text := ""
var diag_frame_max := 0.0
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
	add_child(mech)
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

	panel.anchor_bottom = 1.0
	panel.offset_bottom = -34
	var tabs := TabContainer.new()
	panel.add_child(tabs)
	var box := _tab(tabs, "Dynamics")

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

	var imp := _tab(tabs, "Imperfections")
	_heading(imp, "Sensor feedback (F_XG words)")
	_spin(imp, "Noise distance [mm, std dev]", 0.0, 50.0, 0.1, "imp_noise_pos")
	_spin(imp, "Noise velocity [mm/s]", 0.0, 100.0, 0.5, "imp_noise_vel")
	_spin(imp, "Noise acceleration [m/s2]", 0.0, 5.0, 0.01, "imp_noise_acc")
	_spin(imp, "Delay [ms]", 0.0, 500.0, 5.0, "imp_delay_ms")
	_spin(imp, "Quantisation [bits] (16 = off)", 1.0, 16.0, 1.0, "imp_quant_bits")
	_heading(imp, "Tilt actuator (F_YR words)")
	_spin(imp, "Dead time [ms]", 0.0, 500.0, 5.0, "imp_dead_ms")
	_spin(imp, "Backlash [deg]", 0.0, 5.0, 0.05, "imp_backlash")
	_spin(imp, "Calibration offset X [deg]", -5.0, 5.0, 0.05, "imp_off_x")
	_spin(imp, "Calibration offset Y [deg]", -5.0, 5.0, 0.05, "imp_off_y")

	var flt := _tab(tabs, "Faults")
	_heading(flt, "Random faults (0 = off)")
	_spin(flt, "Pushes per minute", 0.0, 60.0, 0.5, "f_kick_per_min")
	_spin(flt, "Push size [m/s]", 0.0, 2.0, 0.05, "f_kick_v")
	_spin(flt, "Feedback freezes per minute", 0.0, 60.0, 0.5, "f_freeze_per_min")
	_spin(flt, "Freeze duration [s]", 0.0, 30.0, 0.5, "f_freeze_s")
	_spin(flt, "Actuator sticks per minute", 0.0, 60.0, 0.5, "f_stick_per_min")
	_spin(flt, "Stick duration [s]", 0.0, 30.0, 0.5, "f_stick_s")
	_spin(flt, "Seed (0 = random)", 0.0, 99999.0, 1.0, "f_seed")
	_button(flt, "Restart random generators / apply seed", "f_restart_seq")
	_heading(flt, "Trigger now")
	_button(flt, "Push the ball (random direction)", "kick_seq")
	_button(flt, "Freeze the feedback words", "freeze_seq")
	_button(flt, "Stick the tilt actuator", "stick_seq")
	_heading(flt, "Sticky patch on the plate")
	_check(flt, "Sticky patch", "patch_on")
	_spin(flt, "Patch centre X [m]", -1.0, 1.0, 0.05, "patch_x")
	_spin(flt, "Patch centre Y [m]", -1.0, 1.0, 0.05, "patch_y")
	_spin(flt, "Patch radius [m]", 0.05, 1.0, 0.05, "patch_r")
	_spin(flt, "Extra rolling drag [g]", 0.0, 1.0, 0.01, "patch_drag")

	var scn := _tab(tabs, "Scenario")
	_heading(scn, "Release the ball")
	_spin(scn, "Position X [m]", -0.9, 0.9, 0.05, "rel_x")
	_spin(scn, "Position Y [m]", -0.9, 0.9, 0.05, "rel_y")
	_spin(scn, "Velocity X [m/s]", -1.0, 1.0, 0.05, "rel_vx")
	_spin(scn, "Velocity Y [m/s]", -1.0, 1.0, 0.05, "rel_vy")
	_button(scn, "Release ball here", "release_seq")
	_button(scn, "Random release (seeded)", "release_random_seq")
	_heading(scn, "Target for the measurement")
	_spin(scn, "Target X [m]", -0.9, 0.9, 0.05, "tgt_x")
	_spin(scn, "Target Y [m]", -0.9, 0.9, 0.05, "tgt_y")
	_spin(scn, "Zone radius [m]", 0.01, 0.5, 0.01, "tgt_r")
	_button(scn, "Start measurement now", "metrics_start_seq")
	var note := Label.new()
	note.text = "A measurement starts on release, when the target\nchanges, or with the button.  Settled = inside the\nzone for 1 s.  The target is only used for the\nmeasurement, the PLC has its own setpoint."
	note.add_theme_color_override("font_color", Color(0.6, 0.65, 0.75))
	scn.add_child(note)

	var vw := _tab(tabs, "View")
	_heading(vw, "View aids")
	_check(vw, "Show ball trail", "show_trail")
	(controls["show_trail"] as CheckBox).set_pressed_no_signal(true)
	var bt := Button.new()
	bt.text = "Clear trail"
	bt.pressed.connect(func() -> void: trail_pts.clear())
	vw.add_child(bt)


func _tab(tabs: TabContainer, title: String) -> VBoxContainer:
	var sc := ScrollContainer.new()
	sc.name = title
	sc.custom_minimum_size = Vector2(390, 300)
	sc.size_flags_vertical = Control.SIZE_EXPAND_FILL
	var box := VBoxContainer.new()
	sc.add_child(box)
	tabs.add_child(sc)
	return box


func _button(parent: Control, text: String, key: String) -> void:
	var b := Button.new()
	b.text = text
	seqs[key] = 0
	b.pressed.connect(func() -> void:
		seqs[key] += 1
		_send_ui())
	parent.add_child(b)


func _heading(parent: Control, text: String) -> void:
	var l := Label.new()
	l.text = text
	l.add_theme_color_override("font_color", Color(0.75, 0.85, 1.0))
	parent.add_child(l)


func _spin(parent: Control, text: String, lo: float, hi: float, step: float, key: String) -> void:
	var row := HBoxContainer.new()
	var l := Label.new()
	l.text = text
	l.custom_minimum_size = Vector2(210, 0)
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
	for k in seqs:
		d[k] = seqs[k]
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
	_build_mechanism(L)
	_build_markers()
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
	target = Vector3(0, -0.12 * L, 0)         # a little lower, so the mechanism below the plate is in view
	_update_camera()
	_save_home()


# ------------------------------------------------------- markers on the plate
func _build_markers() -> void:
	var gold := Color8(0x65, 0x65, 0x00)
	tgt_ring = MeshInstance3D.new()
	var tm := TorusMesh.new()
	tm.inner_radius = 0.044
	tm.outer_radius = 0.056
	tgt_ring.mesh = tm
	tgt_ring.material_override = _flat_mat(gold)
	table.add_child(tgt_ring)
	tgt_dot = MeshInstance3D.new()
	var dm := CylinderMesh.new()
	dm.top_radius = 0.012
	dm.bottom_radius = 0.012
	dm.height = 0.004
	tgt_dot.mesh = dm
	tgt_dot.material_override = _flat_mat(gold)
	table.add_child(tgt_dot)
	tgt_zone_built = -1.0
	patch_disc = MeshInstance3D.new()
	var pm := CylinderMesh.new()
	pm.top_radius = 0.25
	pm.bottom_radius = 0.25
	pm.height = 0.003
	patch_disc.mesh = pm
	var pmat := StandardMaterial3D.new()
	pmat.albedo_color = Color(0.45, 0.3, 0.15, 0.55)
	pmat.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	patch_disc.material_override = pmat
	patch_disc.visible = false
	table.add_child(patch_disc)
	patch_r_built = -1.0
	trail_mesh = MeshInstance3D.new()
	trail_mesh.mesh = ImmediateMesh.new()
	var trmat := StandardMaterial3D.new()
	trmat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	trmat.albedo_color = Color(0.9, 0.9, 0.5)
	trail_mesh.material_override = trmat
	table.add_child(trail_mesh)
	trail_pts.clear()


func _flat_mat(c: Color) -> StandardMaterial3D:
	var m := StandardMaterial3D.new()
	m.albedo_color = c
	m.roughness = 0.6
	return m


func _update_markers(delta: float) -> void:
	var met: Dictionary = msg.get("metrics", {})
	if not met.is_empty():
		var tg: Array = met["target"]
		var zone := float(met["zone"])
		var pos := Vector3(float(tg[0]), 0.004, -float(tg[1]))
		tgt_ring.position = pos
		tgt_dot.position = pos
		if absf(zone - tgt_zone_built) > 1e-6:
			var tm := tgt_ring.mesh as TorusMesh
			tm.inner_radius = maxf(zone - 0.006, 0.001)
			tm.outer_radius = zone + 0.006
			tgt_zone_built = zone
		var err := float(met["err"])
		var col := Color8(0x00, 0x65, 0x00) if (err >= 0.0 and err <= zone) else Color8(0x65, 0x65, 0x00)
		(tgt_ring.material_override as StandardMaterial3D).albedo_color = col
		(tgt_dot.material_override as StandardMaterial3D).albedo_color = col
	var pt: Dictionary = msg.get("patch", {})
	if not pt.is_empty():
		patch_disc.visible = bool(pt["on"])
		patch_disc.position = Vector3(float(pt["x"]), 0.0025, -float(pt["y"]))
		if absf(float(pt["r"]) - patch_r_built) > 1e-6:
			var pm := patch_disc.mesh as CylinderMesh
			pm.top_radius = float(pt["r"])
			pm.bottom_radius = float(pt["r"])
			patch_r_built = float(pt["r"])
	# ball trail: table-local points every 50 ms; a jump (release / centre) clears it
	var show := bool((controls["show_trail"] as CheckBox).button_pressed)
	var im := trail_mesh.mesh as ImmediateMesh
	im.clear_surfaces()
	if not show:
		trail_pts.clear()
		return
	trail_timer += delta
	if trail_timer >= 0.05 and not bool(msg.get("off", false)):
		trail_timer = 0.0
		var p := Vector3(float(msg["x"]), 0.006, -float(msg["y"]))
		if not trail_pts.is_empty() and p.distance_to(trail_pts[trail_pts.size() - 1]) > 0.3:
			trail_pts.clear()
		trail_pts.append(p)
		if trail_pts.size() > 120:
			trail_pts.remove_at(0)
	if trail_pts.size() > 1:
		im.surface_begin(Mesh.PRIMITIVE_LINE_STRIP)
		for tp in trail_pts:
			im.surface_add_vertex(tp)
		im.surface_end()


# ------------------------------------------------------------ servo mechanism
const MECH_BASE := Color(0.22, 0.23, 0.26)
const MECH_SERVO := Color(0.25, 0.3, 0.4)
const MECH_METAL := Color(0.6, 0.63, 0.68)
const MECH_ROD := Color(0.8, 0.8, 0.85)
const MECH_HORN := Color8(0x65, 0x65, 0x00)        # dark yellow, as the other simulations
const MECH_JOINT := Color8(0x65, 0x00, 0x00)       # dark red


func _cyl_node(radius: float, height: float, pos: Vector3, color: Color, rot_deg := Vector3.ZERO) -> MeshInstance3D:
	var mi := MeshInstance3D.new()
	var cm := CylinderMesh.new()
	cm.top_radius = radius
	cm.bottom_radius = radius
	cm.height = height
	mi.mesh = cm
	var mat := StandardMaterial3D.new()
	mat.albedo_color = color
	mat.roughness = 0.6
	mat.metallic = 0.3
	mi.material_override = mat
	mi.position = pos
	mi.rotation_degrees = rot_deg
	return mi


func _ball_node(radius: float, pos: Vector3, color: Color) -> MeshInstance3D:
	var mi := MeshInstance3D.new()
	var sm := SphereMesh.new()
	sm.radius = radius
	sm.height = 2.0 * radius
	mi.mesh = sm
	var mat := StandardMaterial3D.new()
	mat.albedo_color = color
	mat.roughness = 0.5
	mi.material_override = mat
	mi.position = pos
	return mi


## Base with two servos, a central pivot post with a ball joint under the plate centre, and for each axis a servo horn
## and a connecting rod to the underside of the plate.  The horn angles follow from the plate tilt (two-bar linkage),
## so the servos visibly tilt the plate.  Dimensions scale with the plate size L.
func _build_mechanism(L: float) -> void:
	for c in mech.get_children():
		c.queue_free()
	horn_x = Node3D.new()
	horn_y = Node3D.new()
	rod_x = MeshInstance3D.new()
	rod_y = MeshInstance3D.new()
	lbl_x = _servo_label(Vector3(0.35 * L - 0.125 * L, -0.275 * L + 0.22, 0.2))
	lbl_y = _servo_label(Vector3(0.2, -0.275 * L + 0.22 + 0.07, 0.35 * L - 0.125 * L))      # one font height higher than the X label
	var d := 0.35 * L                      # distance of the rod attachments from the plate centre
	var a := 0.125 * L                     # horn length
	var base_top := -0.45 * L              # top of the base plate below the plate
	var sy := -0.275 * L                   # height of the servo shafts
	var attach_y := -0.04                  # underside of the plate
	mech_geo = {"D": d, "a": a, "sy": sy, "rod": attach_y - sy}
	# base plate and central pivot post with a ball joint just under the plate centre
	mech.add_child(_box(Vector3(0.9 * L, 0.06, 0.9 * L), Vector3(0, base_top - 0.03, 0), MECH_BASE))
	mech.add_child(_cyl_node(0.06, base_top * -1.0 - 0.05, Vector3(0, (base_top - 0.05) / 2.0, 0), MECH_METAL))
	mech.add_child(_ball_node(0.05, Vector3(0, -0.045, 0), MECH_METAL))
	# servo X (shaft along Z, at +x side), horn swings in the x-y plane
	var colh := sy - 0.085 - base_top       # column from the base up to the servo case
	mech.add_child(_box(Vector3(0.2, 0.17, 0.14), Vector3(d - a, sy, -0.07), MECH_SERVO))
	mech.add_child(_box(Vector3(0.12, colh, 0.1), Vector3(d - a, base_top + colh / 2.0, -0.07), MECH_BASE))
	horn_x.position = Vector3(d - a, sy, 0)
	mech.add_child(horn_x)
	horn_x.add_child(_cyl_node(0.04, 0.045, Vector3.ZERO, MECH_METAL, Vector3(90, 0, 0)))
	horn_x.add_child(_box(Vector3(a + 0.04, 0.035, 0.035), Vector3(a / 2.0, 0, 0), MECH_HORN))
	horn_x.add_child(_ball_node(0.026, Vector3(a, 0, 0), MECH_JOINT))
	# servo Y (shaft along X, at +z side), horn swings in the z-y plane
	mech.add_child(_box(Vector3(0.14, 0.17, 0.2), Vector3(-0.07, sy, d - a), MECH_SERVO))
	mech.add_child(_box(Vector3(0.1, colh, 0.12), Vector3(-0.07, base_top + colh / 2.0, d - a), MECH_BASE))
	horn_y.position = Vector3(0, sy, d - a)
	mech.add_child(horn_y)
	horn_y.add_child(_cyl_node(0.04, 0.045, Vector3.ZERO, MECH_METAL, Vector3(0, 0, 90)))
	horn_y.add_child(_box(Vector3(0.035, 0.035, a + 0.04), Vector3(0, 0, a / 2.0), MECH_HORN))
	horn_y.add_child(_ball_node(0.026, Vector3(0, 0, a), MECH_JOINT))
	# connecting rods (placed every frame) and the attachment joints on the underside of the plate (they tilt with it)
	for rod in [rod_x, rod_y]:
		var cm := CylinderMesh.new()
		cm.top_radius = 0.012
		cm.bottom_radius = 0.012
		cm.height = 0.5
		rod.mesh = cm
		var mat := StandardMaterial3D.new()
		mat.albedo_color = MECH_ROD
		mat.metallic = 0.6
		mat.roughness = 0.4
		rod.material_override = mat
		mech.add_child(rod)
	table.add_child(_ball_node(0.026, Vector3(d, attach_y, 0), MECH_JOINT))
	table.add_child(_ball_node(0.026, Vector3(0, attach_y, d), MECH_JOINT))


func _servo_label(pos: Vector3) -> Label3D:
	var lab := Label3D.new()
	lab.font_size = 26
	lab.pixel_size = 0.0022
	lab.billboard = BaseMaterial3D.BILLBOARD_ENABLED
	lab.modulate = Color(0.75, 0.8, 0.9)
	lab.position = pos
	mech.add_child(lab)
	return lab


## Horn angle (from the +u axis towards +v) so that a horn of length a on a shaft at s reaches a rod of length rod_len
## ending at point p (two-bar linkage, elbow on the outer side).
func _horn_angle(s: Vector2, p: Vector2, a: float, rod_len: float) -> float:
	var dd := maxf((p - s).length(), 0.001)
	var cb := clampf((a * a + dd * dd - rod_len * rod_len) / (2.0 * a * dd), -1.0, 1.0)
	return atan2(p.y - s.y, p.x - s.x) - acos(cb)


func _place_rod(rod: MeshInstance3D, p0: Vector3, p1: Vector3) -> void:
	var dv := p1 - p0
	var l := maxf(dv.length(), 0.001)
	(rod.mesh as CylinderMesh).height = l
	rod.position = (p0 + p1) / 2.0
	rod.basis = Basis(Quaternion(Vector3.UP, dv / l))


func _update_mechanism() -> void:
	if mech_geo.is_empty():
		return
	var d: float = mech_geo["D"]
	var a: float = mech_geo["a"]
	var sy: float = mech_geo["sy"]
	var rl: float = mech_geo["rod"]
	var attach_y := -0.04
	# servo X: attachment at the +x side of the plate, horn in the x-y plane
	var ax := table.basis * Vector3(d, attach_y, 0)
	var phi := _horn_angle(Vector2(d - a, sy), Vector2(ax.x, ax.y), a, rl)
	horn_x.rotation = Vector3(0, 0, phi)
	_place_rod(rod_x, Vector3(d - a + a * cos(phi), sy + a * sin(phi), 0), ax)
	lbl_x.text = "servo X %+.1f deg" % rad_to_deg(phi)
	# servo Y: attachment at the +z side of the plate (sim -y), horn in the z-y plane (u = z, v = y)
	var ay := table.basis * Vector3(0, attach_y, d)
	phi = _horn_angle(Vector2(d - a, sy), Vector2(ay.z, ay.y), a, rl)
	horn_y.rotation = Vector3(-phi, 0, 0)
	_place_rod(rod_y, Vector3(0, sy + a * sin(phi), d - a + a * cos(phi)), ay)
	lbl_y.text = "servo Y %+.1f deg" % rad_to_deg(phi)


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
			if diag_last_pkt_ms > 0:
				diag_gap_max = maxi(diag_gap_max, last_rx_ms - diag_last_pkt_ms)
			diag_last_pkt_ms = last_rx_ms
			diag_pkts += 1
			if ip != sim_host:
				sim_host = ip
				cmd.close()
				cmd.set_dest_address(sim_host, cmd_port)
			if not ui_ready and msg.has("params"):
				_fill_ui(msg["params"])

	diag_time += delta
	if diag_time >= 2.0:
		diag_text = "F3 diagnostics (last %.1f s): %d fps, %.0f state packets/s, longest gap between packets %d ms, frame time max %.0f ms" % [
				diag_time, int(Engine.get_frames_per_second()), diag_pkts / diag_time, diag_gap_max, diag_frame_max * 1000.0]
		diag_time = 0.0
		diag_pkts = 0
		diag_gap_max = 0
		diag_frame_max = 0.0
	diag_frame_max = maxf(diag_frame_max, delta)

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
	_update_mechanism()

	# ball (child of the table, table-local coordinates; z < 0 = falling off)
	ball.position = Vector3(float(msg["x"]), R + float(msg.get("z", 0.0)), -float(msg["y"]))
	var q: Array = msg["q"]   # w, x, y, z in the simulator frame
	ball.quaternion = Quaternion(float(q[1]), float(q[3]), -float(q[2]), float(q[0])).normalized()
	_update_markers(delta)

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
	hud.text += _hud_test_aids()
	if diag:
		hud.text += "\n\n" + diag_text


## Control-quality measurement and active faults.
func _hud_test_aids() -> String:
	var t := ""
	var met: Dictionary = msg.get("metrics", {})
	if not met.is_empty():
		var tg: Array = met["target"]
		var err := float(met["err"])
		t += "\n\ntarget (%+.2f, %+.2f) m, zone %.2f m   error %s" % [float(tg[0]), float(tg[1]), float(met["zone"]),
				"(off table)" if err < 0.0 else "%.3f m" % err]
		if bool(met["running"]):
			var st := "settled at %.1f s" % float(met["settle_time"]) if bool(met["settled"]) else "not settled"
			t += "\nrun %.1f s   %s   overshoot %.1f %% (%.3f m)   in zone %.0f %%   RMS %.3f m" % [
					float(met["t"]), st, float(met["overshoot_pct"]), float(met["overshoot_m"]),
					float(met["in_zone_pct"]), float(met["rms"])]
		else:
			t += "\n(no measurement: release the ball, move the target or press Start measurement)"
	var fl: Dictionary = msg.get("faults", {})
	var lines: Array = []
	if not fl.is_empty():
		if float(fl["freeze"]) > 0.0:
			lines.append("feedback frozen %.1f s" % float(fl["freeze"]))
		if float(fl["stick"]) > 0.0:
			lines.append("tilt actuator stuck %.1f s" % float(fl["stick"]))
	if not lines.is_empty():
		t += "\n\nFAULTS: " + "; ".join(PackedStringArray(lines))
	return t


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
			if ev.keycode == KEY_F3 and not ev.echo:
				diag = not diag
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
	help.text = "mouse: drag orbit, middle-drag pan, wheel zoom   |   keys: arrows pan, Shift+arrows orbit, +/- zoom, Home reset, F3 diagnostics"
	help.add_theme_font_size_override("font_size", 12)
	help.add_theme_color_override("font_color", Color(0.6, 0.65, 0.75))
	help.anchor_top = 1.0
	help.anchor_bottom = 1.0
	help.offset_left = 12
	help.offset_top = -24
	layer.add_child(help)
