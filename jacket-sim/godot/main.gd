extends Node3D
## Jacketed water tank (MPC test plant): 3D view + user interface.
##
## All logic runs in the Python simulator (jacketsim).  This program
##  * receives the state as JSON over UDP (default port 9880) and draws it,
##  * sends the parameter/command values from the panel back to the simulator
##    (UDP, port 9881, to the address the state packets come from).
##
## Scene: a tank with a heating jacket (coils glow with the heater power, the jacket shell shows the wall temperature, the water
## colour the water temperature), inlet pipe + valve (inflow, control value M_YR02) on the left and an outlet (level disturbance d2)
## on the right.  A scope at the bottom shows temperature, level, heater and inflow over the last 5 minutes with UI-only setpoint lines.
##
## Run:  godot --path godot -- [--port 9880] [--cmd-port 9881]

var port := 9880
var cmd_port := 9881
var udp := PacketPeerUDP.new()
var cmd := PacketPeerUDP.new()
var sim_host := ""

const BASE_Y := 0.45                # bottom of the tank
const TANK_R := 0.9
const TANK_H := 2.0                 # 1 m of level = 2 units, so the whole tank is 1 m high
const JACKET_R := 1.1
const JACKET_H := 1.5
const IN_Y := 3.25
const OUT_Y := 0.85
const IN_END_X := -0.45
const PIPE_X0 := -3.6
const PIPE_X1 := 3.6

var liquid := MeshInstance3D.new()
var liquid_mesh := CylinderMesh.new()
var liquid_mat := StandardMaterial3D.new()
var jacket_mat := StandardMaterial3D.new()
var coil_mat := StandardMaterial3D.new()
var fall := MeshInstance3D.new()            # stream falling from the inlet pipe into the tank
var in_disc := Node3D.new()
var in_lever := Node3D.new()
var out_disc := Node3D.new()
var out_lever := Node3D.new()
var temp_label: Label3D
var heater_label: Label3D
var streams: Array[Dictionary] = []
var cam := Camera3D.new()

var hud := Label.new()
var panel := PanelContainer.new()
var scope := Control.new()
var controls := {}
var reset_seq := 0
var set_state_seq := 0
var pulse_seq := 0
var ui_ready := false
var send_timer := 0.0
var msg := {}
var last_rx_ms := -100000

# scope history (simulation time), at most 10 points per second
const SCOPE_WINDOW := 300.0
var h_t := PackedFloat32Array()
var h_T := PackedFloat32Array()
var h_L := PackedFloat32Array()
var h_u1 := PackedFloat32Array()
var h_u2 := PackedFloat32Array()
var last_t := -1.0
var last_push := -1.0

const METAL := Color(0.6, 0.63, 0.68)
const WATER := Color8(0x00, 0x00, 0x65)         # dark blue water upstream of a valve disc
const WATER_COLD := Color(0.05, 0.12, 0.45)
const WATER_HOT := Color(0.65, 0.12, 0.05)
const JACKET_COLD := Color(0.55, 0.65, 0.8)
const JACKET_HOT := Color(0.95, 0.5, 0.15)
const DISC := Color8(0x65, 0x65, 0x00)          # dark yellow
const LEVER := Color8(0x65, 0x00, 0x00)         # dark red
const PARTICLE := Color(0.55, 0.65, 0.85)
const TEMP_COL := Color(0.95, 0.4, 0.2)
const LEVEL_COL := Color(0.45, 0.65, 1.0)
const HEAT_COL := Color(0.95, 0.8, 0.2)
const FLOW_COL := Color(0.35, 0.85, 0.7)

var yaw := 0.55
var pitch := 0.22
var dist := 11.0
var target := Vector3(0, 1.7, 0)


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

	_build_plant()
	add_child(cam)
	cam.current = true
	_update_camera()
	_save_home()

	var layer := CanvasLayer.new()
	hud.position = Vector2(12, 8)
	hud.add_theme_font_size_override("font_size", 16)
	layer.add_child(hud)
	_add_help(layer)
	_build_scope(layer)
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




func _label3d(text: String, pos: Vector3, size := 36) -> Label3D:
	var lab := Label3D.new()
	lab.text = text
	lab.font_size = size
	lab.pixel_size = 0.004
	lab.billboard = BaseMaterial3D.BILLBOARD_ENABLED
	lab.modulate = Color(0.6, 0.65, 0.75)
	lab.horizontal_alignment = HORIZONTAL_ALIGNMENT_LEFT
	lab.position = pos
	add_child(lab)
	return lab




func _make_stream(y: float, x0: float, x1: float, n: int) -> void:
	var mm := MultiMesh.new()
	mm.transform_format = MultiMesh.TRANSFORM_3D
	var sm := SphereMesh.new()
	sm.radius = 0.03
	sm.height = 0.06
	sm.radial_segments = 8
	sm.rings = 4
	mm.mesh = sm
	mm.instance_count = n
	var rng := RandomNumberGenerator.new()
	rng.seed = 11 + streams.size()
	var xs := PackedFloat32Array()
	var ys := PackedFloat32Array()
	var zs := PackedFloat32Array()
	for i in n:
		xs.append(x0 + (x1 - x0) * float(i) / n)
		var r := 0.09 * sqrt(rng.randf())
		var a := rng.randf() * TAU
		ys.append(y + r * sin(a))
		zs.append(r * cos(a))
	var mi := MultiMeshInstance3D.new()
	mi.multimesh = mm
	mi.material_override = _mat(PARTICLE)
	add_child(mi)
	streams.append({"mm": mm, "xs": xs, "ys": ys, "zs": zs, "x0": x0, "x1": x1, "n": n})




func _valve(x: float, y: float, disc: Node3D, lever: Node3D, steel: Material) -> void:
	# butterfly valve in a pipe along X; disc across the pipe = closed, along the pipe = open
	_cyl(0.05, 0.6, Vector3(x, y + 0.35, 0), steel)
	_cyl(0.14, 0.25, Vector3(x, y + 0.7, 0), _mat(Color(0.25, 0.3, 0.4), 1.0, 0.3))
	disc.position = Vector3(x, y, 0)
	var dm := CylinderMesh.new()
	dm.top_radius = 0.11
	dm.bottom_radius = 0.11
	dm.height = 0.025
	var dmi := MeshInstance3D.new()
	dmi.mesh = dm
	dmi.material_override = _mat(DISC, 1.0, 0.4)
	dmi.rotation_degrees = Vector3(0, 0, 90)
	disc.add_child(dmi)
	add_child(disc)
	lever.position = Vector3(x, y + 0.84, 0)
	add_child(lever)
	var bar := MeshInstance3D.new()
	var bm := BoxMesh.new()
	bm.size = Vector3(0.06, 0.04, 0.5)
	bar.mesh = bm
	bar.material_override = _mat(LEVER)
	lever.add_child(bar)
	for fx in [-0.28, 0.28]:
		_cyl(0.17, 0.05, Vector3(x + float(fx), y, 0), steel, Vector3(0, 0, 90))    # flanges


## Dark blue water fills the window upstream of the disc (x < xv); the flow particles show through the window.
func _pipe_with_window(x0: float, x1: float, xv: float, y: float, steel: Material, glass: Material) -> MeshInstance3D:
	var wa := xv - 0.28
	var wb := xv + 0.28
	_cyl(0.12, wa - x0, Vector3((x0 + wa) / 2.0, y, 0), steel, Vector3(0, 0, 90))
	_cyl(0.12, x1 - wb, Vector3((wb + x1) / 2.0, y, 0), steel, Vector3(0, 0, 90))
	_cyl(0.12, wb - wa, Vector3(xv, y, 0), glass, Vector3(0, 0, 90))
	return _cyl(0.1, xv - 0.02 - wa, Vector3((wa + xv - 0.02) / 2.0, y, 0), _mat(WATER), Vector3(0, 0, 90))


func _torus(inner: float, outer: float, pos: Vector3, mat: Material) -> MeshInstance3D:
	var mi := MeshInstance3D.new()
	var m := TorusMesh.new()
	m.inner_radius = inner
	m.outer_radius = outer
	m.rings = 48
	m.ring_segments = 12
	mi.mesh = m
	mi.material_override = mat
	mi.position = pos
	add_child(mi)
	return mi


func _build_plant() -> void:
	var steel := _mat(METAL, 1.0, 0.5)
	var glass := _mat(Color(0.75, 0.85, 0.95), 0.2)
	# frame and tank (open top)
	for a in [0.0, 1.5, 3.0, 4.5]:
		var lx := 0.75 * cos(float(a) + 0.8)
		var lz := 0.75 * sin(float(a) + 0.8)
		_cyl(0.07, BASE_Y, Vector3(lx * 1.25, BASE_Y / 2.0, lz * 1.25), steel)
	_cyl(TANK_R + 0.08, 0.08, Vector3(0, BASE_Y - 0.04, 0), steel)
	_cyl(TANK_R, TANK_H, Vector3(0, BASE_Y + TANK_H / 2.0, 0), glass)
	_torus(TANK_R - 0.02, TANK_R + 0.04, Vector3(0, BASE_Y + TANK_H, 0), steel)
	# heating jacket: transparent shell coloured by the wall temperature, rims, heater coils glowing with the heater power
	jacket_mat = _mat(JACKET_COLD, 0.3)
	_cyl(JACKET_R, JACKET_H, Vector3(0, BASE_Y + JACKET_H / 2.0, 0), jacket_mat)
	_torus(TANK_R + 0.02, JACKET_R + 0.06, Vector3(0, BASE_Y + 0.02, 0), steel)
	_torus(TANK_R + 0.02, JACKET_R + 0.06, Vector3(0, BASE_Y + JACKET_H, 0), steel)
	coil_mat = _mat(Color(0.3, 0.25, 0.25), 1.0, 0.3)
	for i in 4:
		_torus((TANK_R + JACKET_R) / 2.0 - 0.035, (TANK_R + JACKET_R) / 2.0 + 0.035, Vector3(0, BASE_Y + 0.3 + 0.3 * float(i), 0), coil_mat)
	# water
	liquid_mesh.top_radius = TANK_R - 0.03
	liquid_mesh.bottom_radius = TANK_R - 0.03
	liquid_mesh.height = 1.0
	liquid_mesh.radial_segments = 48
	liquid.mesh = liquid_mesh
	liquid_mat = _mat(WATER_COLD, 0.9)
	liquid.material_override = liquid_mat
	add_child(liquid)
	# level scale next to the tank (0.1 .. 0.9 m)
	var scale_x := TANK_R + 0.55
	_cyl(0.025, TANK_H * 0.9, Vector3(scale_x, BASE_Y + TANK_H * 0.5, 0), _mat(Color(0.75, 0.85, 0.95), 0.3))
	for lv in [0.1, 0.3, 0.5, 0.7, 0.9]:
		var y := BASE_Y + TANK_H * float(lv)
		_cyl(0.012, 0.16, Vector3(scale_x, y, 0), steel, Vector3(0, 0, 90))
		_label3d("%.1f m" % float(lv), Vector3(scale_x + 0.6, y, 0.0), 26)
	# inlet pipe (top left) with a drop pipe over the tank, outlet pipe (bottom right)
	var pipe_glass := _mat(Color(0.75, 0.85, 0.95), 0.25)
	_pipe_with_window(PIPE_X0, IN_END_X, -2.2, IN_Y, steel, pipe_glass)          # transparent window around each valve
	_cyl(0.12, 0.45, Vector3(IN_END_X, IN_Y - 0.225, 0), steel)
	var elbow := MeshInstance3D.new()                                             # rounds the bend
	var em := SphereMesh.new()
	em.radius = 0.12
	em.height = 0.24
	elbow.mesh = em
	elbow.material_override = steel
	elbow.position = Vector3(IN_END_X, IN_Y, 0)
	add_child(elbow)
	_pipe_with_window(TANK_R, PIPE_X1, 2.4, OUT_Y, steel, pipe_glass)
	_valve(-2.2, IN_Y, in_disc, in_lever, steel)
	_valve(2.4, OUT_Y, out_disc, out_lever, steel)
	_label3d("M_YR02  inflow", Vector3(-2.2, IN_Y + 1.1, 0.0))
	_label3d("outlet  (level disturbance d2)", Vector3(2.4, OUT_Y + 1.1, 0.0))
	var fm := CylinderMesh.new()
	fm.top_radius = 0.04
	fm.bottom_radius = 0.04
	fm.height = 1.0
	fm.radial_segments = 12
	fall.mesh = fm
	fall.material_override = _mat(PARTICLE)
	add_child(fall)
	_make_stream(IN_Y, PIPE_X0, IN_END_X, 60)
	_make_stream(OUT_Y, TANK_R, PIPE_X1, 60)
	temp_label = _label3d("", Vector3(0, BASE_Y + TANK_H + 0.9, 0), 64)
	heater_label = _label3d("", Vector3(0, BASE_Y - 0.25, 1.9), 40)



# ---------------------------------------------------------------------- scope
func _build_scope(layer: CanvasLayer) -> void:
	scope.anchor_top = 1.0
	scope.anchor_bottom = 1.0
	scope.anchor_right = 1.0
	scope.offset_right = -410
	scope.offset_left = 10
	scope.offset_top = -230
	scope.offset_bottom = -28
	scope.mouse_filter = Control.MOUSE_FILTER_IGNORE
	scope.draw.connect(_scope_draw)
	layer.add_child(scope)


func _norm_T(v: float) -> float:
	return clampf((v - 20.0) / 100.0, 0.0, 1.0)


func _norm_L(v: float) -> float:
	return clampf((v - 0.1) / 0.8, 0.0, 1.0)


func _scope_draw() -> void:
	var sz := scope.size
	scope.draw_rect(Rect2(Vector2.ZERO, sz), Color(0.07, 0.08, 0.1, 0.88))
	var ml := 34.0
	var mt := 8.0
	var mb := 6.0
	var w := sz.x - ml - 8.0
	var hh := sz.y - mt - mb
	var font := ThemeDB.fallback_font
	for v in [0, 25, 50, 75, 100]:
		var y := mt + hh * (1.0 - float(v) / 100.0)
		scope.draw_line(Vector2(ml, y), Vector2(ml + w, y), Color(1, 1, 1, 0.10))
		scope.draw_string(font, Vector2(3, y + 4), str(v), HORIZONTAL_ALIGNMENT_LEFT, -1, 11, Color(0.7, 0.75, 0.85))
	if msg.is_empty() or h_t.size() < 2:
		return
	var t_now := float(h_t[h_t.size() - 1])
	# setpoint lines (UI values, a reference only - the simulator does not know the PLC setpoints)
	for item in [["sp_temp", true, TEMP_COL], ["sp_level", false, LEVEL_COL]]:
		var key: String = item[0]
		if not controls.has(key):
			continue
		var spv: float = (controls[key] as SpinBox).value
		var nv := _norm_T(spv) if bool(item[1]) else _norm_L(spv)
		var ys := mt + hh * (1.0 - nv)
		var col: Color = item[2]
		for i in 50:
			var x0 := ml + w * float(i) / 50.0
			scope.draw_line(Vector2(x0, ys), Vector2(x0 + w / 100.0, ys), Color(col.r, col.g, col.b, 0.6), 1.0)
	var pT := PackedVector2Array()
	var pL := PackedVector2Array()
	var p1 := PackedVector2Array()
	var p2 := PackedVector2Array()
	for i in h_t.size():
		var x := ml + w * (1.0 - (t_now - float(h_t[i])) / SCOPE_WINDOW)
		if x < ml:
			continue
		pT.append(Vector2(x, mt + hh * (1.0 - _norm_T(float(h_T[i])))))
		pL.append(Vector2(x, mt + hh * (1.0 - _norm_L(float(h_L[i])))))
		p1.append(Vector2(x, mt + hh * (1.0 - clampf(float(h_u1[i]) / 100.0, 0.0, 1.0))))
		p2.append(Vector2(x, mt + hh * (1.0 - clampf(float(h_u2[i]), 0.0, 1.0))))
	if pT.size() > 1:
		scope.draw_polyline(p1, HEAT_COL, 1.2)
		scope.draw_polyline(p2, FLOW_COL, 1.2)
		scope.draw_polyline(pT, TEMP_COL, 1.8)
		scope.draw_polyline(pL, LEVEL_COL, 1.8)
	scope.draw_string(font, Vector2(ml + 6, mt + 12), "temperature 20-120 degC", HORIZONTAL_ALIGNMENT_LEFT, -1, 12, TEMP_COL)
	scope.draw_string(font, Vector2(ml + 175, mt + 12), "level 0.1-0.9 m", HORIZONTAL_ALIGNMENT_LEFT, -1, 12, LEVEL_COL)
	scope.draw_string(font, Vector2(ml + 290, mt + 12), "heater 0-100 %", HORIZONTAL_ALIGNMENT_LEFT, -1, 12, HEAT_COL)
	scope.draw_string(font, Vector2(ml + 395, mt + 12), "inflow 0-1", HORIZONTAL_ALIGNMENT_LEFT, -1, 12, FLOW_COL)
	scope.draw_string(font, Vector2(ml + 480, mt + 12), "dashed = setpoint (UI)", HORIZONTAL_ALIGNMENT_LEFT, -1, 12, Color(0.8, 0.8, 0.8))
	scope.draw_string(font, Vector2(sz.x - 120, sz.y - 4), "%d s window" % int(SCOPE_WINDOW), HORIZONTAL_ALIGNMENT_LEFT, -1, 11, Color(0.5, 0.55, 0.65))


func _push_history() -> void:
	var t := float(msg["t"])
	if t < last_t:                    # simulator restarted
		h_t.clear()
		h_T.clear()
		h_L.clear()
		h_u1.clear()
		h_u2.clear()
		last_push = -1.0
	last_t = t
	if t - last_push < 0.1:
		return
	last_push = t
	var y: Array = msg["y"]
	var u: Array = msg["u"]
	h_t.append(t)
	h_T.append(float(y[0]))
	h_L.append(float(y[1]))
	h_u1.append(float(u[0]))
	h_u2.append(float(u[1]))
	while h_t.size() > 2 and t - float(h_t[0]) > SCOPE_WINDOW:
		h_t.remove_at(0)
		h_T.remove_at(0)
		h_L.remove_at(0)
		h_u1.remove_at(0)
		h_u2.remove_at(0)


# ---------------------------------------------------------------------- panel
func _build_panel(layer: CanvasLayer) -> void:
	panel.anchor_left = 1.0
	panel.anchor_right = 1.0
	panel.anchor_bottom = 1.0
	panel.offset_left = -400
	panel.offset_right = -10
	panel.offset_top = 10
	panel.offset_bottom = -10
	panel.grow_horizontal = Control.GROW_DIRECTION_BEGIN
	panel.visible = false
	layer.add_child(panel)
	var scroll := ScrollContainer.new()
	scroll.horizontal_scroll_mode = ScrollContainer.SCROLL_MODE_DISABLED
	panel.add_child(scroll)
	var box := VBoxContainer.new()
	box.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	scroll.add_child(box)

	_heading(box, "Plant")
	_spin(box, "Sample time PTSDT [s]", 0.05, 60.0, 0.05, "sample_time_s")
	_check(box, "Hold the plant (freeze the states)", "hold")
	_heading(box, "Disturbances (inputs XD01 / XD02 of the model)")
	_spin(box, "d1 heating disturbance", -200.0, 200.0, 0.5, "d1")
	_spin(box, "d2 level disturbance (+ drains)", -20.0, 20.0, 0.01, "d2")
	_spin(box, "d2 pulse amount", -20.0, 20.0, 0.01, "pulse_amount", 0.2)
	_spin(box, "   duration [s]", 0.1, 600.0, 0.5, "pulse_s", 10.0)
	_button(box, "Apply d2 pulse", func() -> void:
		pulse_seq += 1
		_send_ui())
	_heading(box, "Manual control (overrides the PLC, e.g. open-loop step tests)")
	_check(box, "Manual override", "manual_on")
	_spin(box, "Heater [%]", 0.0, 100.0, 0.5, "manual_heater")
	_spin(box, "Inflow [0..1]", 0.0, 1.0, 0.005, "manual_inflow")
	_heading(box, "Measurement (test aids)")
	_spin(box, "Temperature noise +- [degC]", 0.0, 20.0, 0.05, "noise_temp")
	_spin(box, "Level noise +- [m]", 0.0, 0.5, 0.001, "noise_level")
	_spin(box, "Temperature offset [degC]", -50.0, 50.0, 0.1, "bias_temp")
	_spin(box, "Level offset [m]", -0.5, 0.5, 0.005, "bias_level")
	_check(box, "Freeze the measured values", "freeze_sensors")
	_spin(box, "Actuator dead time [samples]", 0.0, 60.0, 1.0, "dead_time_samples")
	_heading(box, "Faults / plant-model mismatch")
	_spin(box, "Heater limit [%]", 0.0, 100.0, 1.0, "heater_max")
	_spin(box, "Heater gain factor", 0.0, 5.0, 0.05, "b_scale_heater")
	_spin(box, "Inflow gain factor", 0.0, 5.0, 0.05, "b_scale_inflow")
	_spin(box, "Plant speed factor (decay rates)", 0.0, 3.0, 0.05, "a_scale")
	_heading(box, "State")
	_button(box, "Reset to the initial state (70 degC, 0.5 m)", func() -> void:
		reset_seq += 1
		_send_ui())
	_spin(box, "Set temperature [degC]", 2.0, 120.0, 0.5, "set_temp", 70.0)
	_spin(box, "Set level [m]", 0.0, 0.9, 0.01, "set_level", 0.5)
	_button(box, "Set state", func() -> void:
		set_state_seq += 1
		_send_ui())
	_heading(box, "Scope")
	_spin(box, "Temperature setpoint line [degC]", 20.0, 120.0, 0.5, "sp_temp", 70.0)
	_spin(box, "Level setpoint line [m]", 0.1, 0.9, 0.01, "sp_level", 0.5)


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
	var d := {"reset_seq": reset_seq, "set_state_seq": set_state_seq, "pulse_seq": pulse_seq}
	for k in controls:
		var c = controls[k]
		d[k] = c.value if c is SpinBox else c.button_pressed
	cmd.put_packet(JSON.stringify(d).to_utf8_buffer())


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
			_push_history()
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
	scope.queue_redraw()

	var x: Array = msg["x"]
	var y: Array = msg["y"]
	var ym: Array = msg["ym"]
	var u: Array = msg["u"]
	var d: Array = msg["d"]
	var raw_in: Array = msg["raw_in"]
	var raw_out: Array = msg["raw_out"]
	var p: Dictionary = msg.get("params", {})
	var temp := float(x[0])
	var wall := float(x[1])
	var level := float(x[2])

	# water: height = level, colour = temperature; jacket shell colour = wall temperature
	var lh := TANK_H * level
	liquid.visible = lh > 0.004
	liquid_mesh.height = maxf(lh, 0.004)
	liquid.position = Vector3(0, BASE_Y + lh / 2.0, 0)
	liquid_mat.albedo_color = Color(WATER_COLD.lerp(WATER_HOT, clampf((temp - 20.0) / 100.0, 0.0, 1.0)), 0.9)
	var jc := JACKET_COLD.lerp(JACKET_HOT, clampf((wall - 10.0) / 90.0, 0.0, 1.0))
	jacket_mat.albedo_color = Color(jc, 0.3)
	# heater coils glow with the heater power
	var heat := clampf(float(u[0]) / 100.0, 0.0, 1.0)
	coil_mat.albedo_color = Color(0.3, 0.25, 0.25).lerp(Color(0.95, 0.3, 0.05), heat)
	coil_mat.emission_enabled = heat > 0.01
	coil_mat.emission = Color(1.0, 0.35, 0.05)
	coil_mat.emission_energy_multiplier = 0.2 + 2.6 * heat
	# valves: disc and lever turn together (0 = across the pipe = closed)
	var inflow := clampf(float(u[1]), 0.0, 1.0)
	in_disc.rotation_degrees = Vector3(0, 90.0 * inflow, 0)
	in_lever.rotation_degrees = in_disc.rotation_degrees
	var drain := clampf(float(d[1]) / 0.5, 0.0, 1.0)
	out_disc.rotation_degrees = Vector3(0, 90.0 * drain, 0)
	out_lever.rotation_degrees = out_disc.rotation_degrees
	# flow
	_move_stream(streams[0], inflow * 100.0, delta)
	_move_stream(streams[1], drain * 100.0 if level > 0.01 else 0.0, delta)
	var surface := BASE_Y + lh
	var top := IN_Y - 0.45
	fall.visible = inflow > 0.01
	var fall_len := maxf(top - surface, 0.01)
	var fw := clampf(sqrt(inflow), 0.05, 1.0) * 1.2
	fall.scale = Vector3(fw, fall_len, fw)
	fall.position = Vector3(IN_END_X, surface + fall_len / 2.0, 0)
	temp_label.text = "%.1f degC    %.3f m" % [temp, level]
	temp_label.modulate = TEMP_COL.lerp(Color(1, 1, 1), 0.5)
	heater_label.text = "heater %.0f %%   (M_YR01)" % float(u[0])

	var src := "manual override" if bool(msg.get("manual", false)) else ("PLC" if bool(msg.get("plc", false)) else "no PLC - heater off, no inflow")
	hud.text = "temperature %.1f degC  (measured %.1f, M_XG01 = %d)\nlevel %.3f m  (measured %.3f, M_XG02 = %d)\nheater %.1f %%  (M_YR01 = %d)   inflow %.3f  (M_YR02 = %d)   [%s]\nstates x1..x4: %.1f  %.1f  %.3f  %.1f      d1 %.1f   d2 %.2f      OPC UA clients: %d\nsample %d (PTSDT %.2f s)" % [
		float(y[0]), float(ym[0]), int(raw_out[0]), float(y[1]), float(ym[1]), int(raw_out[1]),
		float(u[0]), int(raw_in[0]), float(u[1]), int(raw_in[1]), src,
		float(x[0]), float(x[1]), float(x[2]), float(x[3]), float(d[0]), float(d[1]), int(msg.get("clients", 0)),
		int(msg.get("samples", 0)), float(p.get("sample_time_s", 1.0))]





func _button(parent: Control, text: String, fn: Callable) -> void:
	var b := Button.new()
	b.text = text
	b.pressed.connect(fn)
	parent.add_child(b)




func _heading(parent: Control, text: String) -> void:
	var l := Label.new()
	l.text = text
	l.add_theme_color_override("font_color", Color(0.75, 0.85, 1.0))
	parent.add_child(l)




func _spin(parent: Control, text: String, lo: float, hi: float, step: float, key: String, init := 0.0) -> void:
	var row := HBoxContainer.new()
	var l := Label.new()
	l.text = text
	l.custom_minimum_size = Vector2(230, 0)
	var sb := SpinBox.new()
	sb.min_value = lo
	sb.max_value = hi
	sb.step = step
	sb.value = init
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




func _move_stream(s: Dictionary, flow_pct: float, delta: float) -> void:
	var mm: MultiMesh = s["mm"]
	var xs: PackedFloat32Array = s["xs"]
	var ys: PackedFloat32Array = s["ys"]
	var zs: PackedFloat32Array = s["zs"]
	var x0 := float(s["x0"])
	var x1 := float(s["x1"])
	var vis := 1.0 if flow_pct > 0.5 else 0.0
	var speed := 0.4 + 2.2 * clampf(flow_pct / 100.0, 0.0, 1.0)
	for i in int(s["n"]):
		xs[i] += speed * delta * vis
		if xs[i] > x1:
			xs[i] -= x1 - x0
		mm.set_instance_transform(i, Transform3D(Basis().scaled(Vector3.ONE * vis), Vector3(xs[i], ys[i], zs[i])))




func _unhandled_input(ev: InputEvent) -> void:
	if _camera_event(ev):
		return
	if ev is InputEventMouseMotion and (ev.button_mask & MOUSE_BUTTON_MASK_LEFT):
		yaw -= ev.relative.x * 0.005
		pitch = clampf(pitch + ev.relative.y * 0.005, -0.2, 1.5)
		_update_camera()
	elif ev is InputEventMouseButton and ev.pressed:
		if ev.button_index == MOUSE_BUTTON_WHEEL_UP:
			dist = maxf(CAM_DIST_MIN, dist * 0.92)
			_update_camera()
		elif ev.button_index == MOUSE_BUTTON_WHEEL_DOWN:
			dist = minf(CAM_DIST_MAX, dist * 1.08)
			_update_camera()




func _update_camera() -> void:
	var p := target + Vector3(sin(yaw) * cos(pitch), sin(pitch), cos(yaw) * cos(pitch)) * dist
	cam.look_at_from_position(p, target, Vector3.UP)


# ------------------------------------------------------- camera: keyboard + middle mouse
## Arrow keys pan the view (the scene moves in the arrow's direction), Shift+arrows orbit, +/- (or PageUp/Down)
## zoom, Home resets the view; mouse: left drag orbits, middle drag pans, wheel zooms.
const CAM_PITCH_MIN := -0.2
const CAM_DIST_MIN := 2.0
const CAM_DIST_MAX := 40.0
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
