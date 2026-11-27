extends Node3D
## Level simulation: user interface + 3D view.
##
## All logic runs in the Python simulator (levelsim).  This program
##  * receives the state as JSON over UDP (default port 9876) and draws it,
##  * sends the parameter/command values from the panel back to the simulator
##    (UDP, port 9877, to the address the state packets come from).
##
## Scene: vessel with an inlet pipe + valve (PLC output T_FCV10) on the top left and an outlet pipe + valve
## (load / consumption) at the bottom right; level switches T_LSH10 / T_LSL10 as lamps on the vessel wall.
## A scope at the bottom shows level, valve command and the switch marks over the last two minutes.
##
## Run:  godot --path godot -- [--port 9876] [--cmd-port 9877]

var port := 9876
var cmd_port := 9877
var udp := PacketPeerUDP.new()
var cmd := PacketPeerUDP.new()
var sim_host := ""

const VESSEL_Y0 := 0.1              # bottom of the vessel (y)
const VESSEL_H := 2.0               # inner height = 100 %
const VESSEL_R := 0.7
const IN_Y := 2.75
const OUT_Y := 0.45
const IN_END_X := -0.35
const PIPE_X0 := -3.4
const PIPE_X1 := 3.4

var liquid := MeshInstance3D.new()
var liquid_mesh := CylinderMesh.new()
var fall := MeshInstance3D.new()            # stream falling from the inlet pipe into the vessel
var in_disc := Node3D.new()
var in_lever := Node3D.new()
var out_disc := Node3D.new()
var out_lever := Node3D.new()
var lamp_h := MeshInstance3D.new()
var lamp_l := MeshInstance3D.new()
var lab_h := Label3D.new()
var lab_l := Label3D.new()
var streams: Array[Dictionary] = []
var cam := Camera3D.new()

var hud := Label.new()
var panel := PanelContainer.new()
var scope := Control.new()
var controls := {}
var set_level_seq := 0
var pulse_seq := 0
var ui_ready := false
var send_timer := 0.0
var msg := {}
var last_rx_ms := -100000

# scope history (simulation time)
const SCOPE_WINDOW := 120.0
var h_t := PackedFloat32Array()
var h_y := PackedFloat32Array()
var h_u := PackedFloat32Array()
var h_h := PackedFloat32Array()
var last_t := -1.0

# dimmed palette shared with the other simulations
const METAL := Color(0.6, 0.63, 0.68)
const WATER := Color8(0x00, 0x00, 0x65)         # dark blue
const DISC := Color8(0x65, 0x65, 0x00)          # dark yellow
const LEVER := Color8(0x65, 0x00, 0x00)         # dark red
const PARTICLE := Color(0.55, 0.65, 0.85)
const LED_HIGH := Color8(0x00, 0x65, 0x00)      # dark green
const LED_LOW := Color8(0x65, 0x65, 0x00)       # dark yellow

var yaw := 0.5
var pitch := 0.25
var dist := 8.0
var target := Vector3(0, 1.5, 0)


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
	dm.top_radius = 0.16
	dm.bottom_radius = 0.16
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
		_cyl(0.2, 0.06, Vector3(x + float(fx), y, 0), steel, Vector3(0, 0, 90))    # flanges


func _build_plant() -> void:
	var steel := _mat(METAL, 1.0, 0.5)
	var glass := _mat(Color(0.75, 0.85, 0.95), 0.22)
	# vessel
	_cyl(VESSEL_R + 0.06, 0.1, Vector3(0, VESSEL_Y0 - 0.05, 0), steel)
	_cyl(VESSEL_R, VESSEL_H, Vector3(0, VESSEL_Y0 + VESSEL_H / 2.0, 0), glass)
	_cyl(VESSEL_R + 0.03, 0.06, Vector3(0, VESSEL_Y0 + VESSEL_H + 0.03, 0), steel)
	liquid_mesh.top_radius = VESSEL_R - 0.04
	liquid_mesh.bottom_radius = VESSEL_R - 0.04
	liquid_mesh.height = 1.0
	liquid_mesh.radial_segments = 48
	liquid.mesh = liquid_mesh
	liquid.material_override = _mat(WATER, 0.9)
	add_child(liquid)
	# level scale next to the vessel
	for v in [0, 25, 50, 75, 100]:
		var y := VESSEL_Y0 + VESSEL_H * float(v) / 100.0
		_cyl(0.015, 0.14, Vector3(-VESSEL_R - 0.1, y, 0), steel, Vector3(0, 0, 90))
		_label3d("%d %%" % v, Vector3(-VESSEL_R - 0.55, y, 0.0), 28)
	# inlet pipe (top left) with a short drop pipe over the vessel
	var in_len := IN_END_X - PIPE_X0
	_cyl(0.12, in_len, Vector3(PIPE_X0 + in_len / 2.0, IN_Y, 0), steel, Vector3(0, 0, 90))
	_cyl(0.12, 0.3, Vector3(IN_END_X, IN_Y - 0.15, 0), steel)
	# outlet pipe (bottom right)
	var out_len := PIPE_X1 - VESSEL_R
	_cyl(0.12, out_len, Vector3(VESSEL_R + out_len / 2.0, OUT_Y, 0), steel, Vector3(0, 0, 90))
	_valve(-2.0, IN_Y, in_disc, in_lever, steel)
	_valve(2.0, OUT_Y, out_disc, out_lever, steel)
	_label3d("T_FCV10", Vector3(-2.0, IN_Y + 1.05, 0.0))
	_label3d("outlet (load)", Vector3(2.0, OUT_Y + 1.05, 0.0))
	# falling stream into the vessel
	var fm := CylinderMesh.new()
	fm.top_radius = 0.04
	fm.bottom_radius = 0.04
	fm.height = 1.0
	fm.radial_segments = 12
	fall.mesh = fm
	fall.material_override = _mat(PARTICLE)
	add_child(fall)
	# particle streams in both pipes
	_make_stream(IN_Y, PIPE_X0, IN_END_X, 60)
	_make_stream(OUT_Y, VESSEL_R, PIPE_X1, 60)
	# level switch lamps on the front of the vessel wall
	for item in [[lamp_h, lab_h, "T_LSH10"], [lamp_l, lab_l, "T_LSL10"]]:
		var lamp: MeshInstance3D = item[0]
		var lm := SphereMesh.new()
		lm.radius = 0.07
		lm.height = 0.14
		lamp.mesh = lm
		lamp.material_override = _mat(METAL, 0.5)
		add_child(lamp)
		var lab: Label3D = item[1]
		lab.text = item[2]
		lab.font_size = 30
		lab.pixel_size = 0.004
		lab.billboard = BaseMaterial3D.BILLBOARD_ENABLED
		lab.modulate = Color(0.75, 0.8, 0.9)
		lab.horizontal_alignment = HORIZONTAL_ALIGNMENT_LEFT
		add_child(lab)


# ---------------------------------------------------------------------- scope
func _build_scope(layer: CanvasLayer) -> void:
	scope.anchor_top = 1.0
	scope.anchor_bottom = 1.0
	scope.anchor_right = 1.0
	scope.offset_right = -410
	scope.offset_left = 10
	scope.offset_top = -210
	scope.offset_bottom = -28
	scope.mouse_filter = Control.MOUSE_FILTER_IGNORE
	scope.draw.connect(_scope_draw)
	layer.add_child(scope)


func _scope_draw() -> void:
	var sz := scope.size
	scope.draw_rect(Rect2(Vector2.ZERO, sz), Color(0.07, 0.08, 0.1, 0.85))
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
	var p: Dictionary = msg.get("params", {})
	var t_now := float(h_t[h_t.size() - 1])
	# switch marks
	for item in [["h_high", Color(0.55, 0.15, 0.15)], ["h_low", Color(0.6, 0.6, 0.15)]]:
		var key: String = item[0]
		var col: Color = item[1]
		var yv := mt + hh * (1.0 - float(p.get(key, 0.0)) / 100.0)
		scope.draw_line(Vector2(ml, yv), Vector2(ml + w, yv), col, 1.0)
	# setpoint (UI value, only a reference line - the simulator does not know the PLC setpoint)
	if controls.has("setpoint"):
		var sp: float = (controls["setpoint"] as SpinBox).value
		var ys := mt + hh * (1.0 - sp / 100.0)
		for i in 40:
			var x0 := ml + w * float(i) / 40.0
			scope.draw_line(Vector2(x0, ys), Vector2(x0 + w / 80.0, ys), Color(1, 1, 1, 0.55), 1.0)
	var pu := PackedVector2Array()
	var py := PackedVector2Array()
	var ph := PackedVector2Array()
	for i in h_t.size():
		var x := ml + w * (1.0 - (t_now - float(h_t[i])) / SCOPE_WINDOW)
		if x < ml:
			continue
		pu.append(Vector2(x, mt + hh * (1.0 - float(h_u[i]) / 100.0)))
		py.append(Vector2(x, mt + hh * (1.0 - clampf(float(h_y[i]), 0.0, 100.0) / 100.0)))
		ph.append(Vector2(x, mt + hh * (1.0 - float(h_h[i]) / 100.0)))
	if pu.size() > 1:
		scope.draw_polyline(pu, Color(0.25, 0.8, 0.3), 1.5)
		scope.draw_polyline(py, Color(0.45, 0.65, 1.0), 1.5)
	scope.draw_string(font, Vector2(ml + 6, mt + 12), "level (T_LT10)", HORIZONTAL_ALIGNMENT_LEFT, -1, 12, Color(0.45, 0.65, 1.0))
	scope.draw_string(font, Vector2(ml + 120, mt + 12), "valve (T_FCV10)", HORIZONTAL_ALIGNMENT_LEFT, -1, 12, Color(0.25, 0.8, 0.3))
	scope.draw_string(font, Vector2(ml + 250, mt + 12), "setpoint (UI)", HORIZONTAL_ALIGNMENT_LEFT, -1, 12, Color(0.8, 0.8, 0.8))
	scope.draw_string(font, Vector2(sz.x - 120, sz.y - 4), "%d s window" % int(SCOPE_WINDOW), HORIZONTAL_ALIGNMENT_LEFT, -1, 11, Color(0.5, 0.55, 0.65))


func _push_history() -> void:
	var t := float(msg["t"])
	if t < last_t:                    # simulator restarted
		h_t.clear()
		h_y.clear()
		h_u.clear()
		h_h.clear()
	last_t = t
	h_t.append(t)
	h_y.append(float(msg["y"]))
	h_u.append(float(msg["u"]))
	h_h.append(float(msg["h"]))
	while h_t.size() > 2 and t - float(h_t[0]) > SCOPE_WINDOW:
		h_t.remove_at(0)
		h_y.remove_at(0)
		h_u.remove_at(0)
		h_h.remove_at(0)


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

	_heading(box, "Process (valve -> delay -> [lag] -> 2nd order -> integrator)")
	_spin(box, "Dead time [s]", 0.0, 60.0, 0.1, "delay")
	_spin(box, "Actuator lag [s]  (0 = off, order 3)", 0.0, 60.0, 0.1, "t_act")
	_spin(box, "2nd order: frequency [Hz]", 0.005, 5.0, 0.01, "fn")
	_spin(box, "2nd order: damping", 0.01, 5.0, 0.01, "zeta")
	_spin(box, "2nd order: gain", -10.0, 10.0, 0.05, "gain")
	_spin(box, "Accumulator reinforcement [1/s]", 0.0, 5.0, 0.01, "k_acc")
	_spin(box, "Outlet valve (load) [%]", 0.0, 100.0, 1.0, "outlet")
	_heading(box, "Test signals (gain in %, 0 = off)")
	_spin(box, "System oscillation: freq [Hz]", 0.0, 5.0, 0.01, "sys_osc_f")
	_spin(box, "   gain [%]", 0.0, 100.0, 0.5, "sys_osc_gain")
	_spin(box, "Random oscillation: freq [Hz]", 0.0, 5.0, 0.01, "rnd_osc_f")
	_spin(box, "   gain [%]", 0.0, 100.0, 0.5, "rnd_osc_gain")
	_spin(box, "Random consumption: rate [Hz]", 0.0, 20.0, 0.05, "cons_f")
	_spin(box, "   gain [%]", 0.0, 100.0, 0.5, "cons_gain")
	_spin(box, "Measurement noise: rate [Hz]", 0.0, 50.0, 0.5, "noise_f")
	_spin(box, "   gain [%]", 0.0, 50.0, 0.1, "noise_gain")
	_heading(box, "Level switches")
	_spin(box, "T_LSH10 high mark [%]", 0.0, 100.0, 1.0, "h_high")
	_spin(box, "T_LSL10 low mark [%]", 0.0, 100.0, 1.0, "h_low")
	_spin(box, "Hysteresis [%]", 0.0, 50.0, 0.5, "hysteresis")
	_check(box, "T_LSL10 true while level ABOVE the low mark", "lsl_inverted")
	_heading(box, "Manual valve (overrides the PLC, also without PLC)")
	_check(box, "Manual valve override", "manual_on")
	_spin(box, "Manual valve [%]", 0.0, 100.0, 1.0, "manual_u")
	_heading(box, "Actions")
	_spin(box, "Level [%]", 0.0, 100.0, 1.0, "set_level", 50.0)
	_button(box, "Set level", func() -> void:
		set_level_seq += 1
		_send_ui())
	_spin(box, "Consumption pulse [%]", -100.0, 100.0, 1.0, "pulse_amount", 20.0)
	_spin(box, "   duration [s]", 0.1, 600.0, 0.5, "pulse_s", 5.0)
	_button(box, "Apply pulse", func() -> void:
		pulse_seq += 1
		_send_ui())
	_heading(box, "Scope")
	_spin(box, "Setpoint line [%]", 0.0, 100.0, 1.0, "setpoint", 50.0)


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
	var d := {"set_level_seq": set_level_seq, "pulse_seq": pulse_seq}
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

	var h := float(msg["h"])
	var u := float(msg["u"])
	var q_in := float(msg["q_in"])
	var q_out := float(msg["q_out"])
	var p: Dictionary = msg.get("params", {})

	# liquid
	var lh := VESSEL_H * h / 100.0
	liquid.visible = lh > 0.003
	liquid_mesh.height = maxf(lh, 0.003)
	liquid.position = Vector3(0, VESSEL_Y0 + lh / 2.0, 0)
	# valves: disc and lever turn together (0 % = across the pipe = closed)
	in_disc.rotation_degrees = Vector3(0, 90.0 * clampf(u / 100.0, 0.0, 1.0), 0)
	in_lever.rotation_degrees = in_disc.rotation_degrees
	out_disc.rotation_degrees = Vector3(0, 90.0 * clampf(q_out / 100.0, 0.0, 1.0), 0)
	out_lever.rotation_degrees = out_disc.rotation_degrees
	# flow
	_move_stream(streams[0], q_in, delta)
	_move_stream(streams[1], q_out if h > 0.5 else 0.0, delta)
	var surface := VESSEL_Y0 + lh
	var top := IN_Y - 0.3
	fall.visible = q_in > 0.5
	var fall_len := maxf(top - surface, 0.01)
	fall.scale = Vector3(clampf(sqrt(q_in / 100.0), 0.05, 1.0) * 1.2, fall_len, clampf(sqrt(q_in / 100.0), 0.05, 1.0) * 1.2)
	fall.position = Vector3(IN_END_X, surface + fall_len / 2.0, 0)

	# level switch lamps follow the configured marks
	var yh := VESSEL_Y0 + VESSEL_H * float(p.get("h_high", 80.0)) / 100.0
	var yl := VESSEL_Y0 + VESSEL_H * float(p.get("h_low", 20.0)) / 100.0
	var zf := VESSEL_R + 0.04
	lamp_h.position = Vector3(0.35, yh, sqrt(maxf(zf * zf - 0.35 * 0.35, 0.0)))
	lamp_l.position = Vector3(0.35, yl, sqrt(maxf(zf * zf - 0.35 * 0.35, 0.0)))
	lab_h.position = lamp_h.position + Vector3(0.15, 0, 0.1)
	lab_l.position = lamp_l.position + Vector3(0.15, 0, 0.1)
	_led(lamp_h, bool(msg["lsh"]), LED_HIGH)
	_led(lamp_l, bool(msg["lsl"]), LED_LOW)

	var src := "manual override" if bool(msg.get("manual", false)) else ("PLC" if bool(msg.get("plc", false)) else "no PLC - valve closed")
	hud.text = "level %5.1f %%   (T_LT10 = %d,  true %.1f %%)\nvalve %5.1f %%   (T_FCV10 = %d)  [%s]\ninflow %5.1f %%   outflow %5.1f %%\nT_LSH10 %s   T_LSL10 %s      OPC UA clients: %d\n\nprocess order %d + dead time %.1f s\npoles: %s" % [
		float(msg["y"]), int(msg["lt_raw"]), h, u, int(msg["fcv_raw"]), src, q_in, q_out,
		_lamp(bool(msg["lsh"])), _lamp(bool(msg["lsl"])), int(msg.get("clients", 0)),
		int(msg["order"]), float(p.get("delay", 0.0)), ", ".join(PackedStringArray(msg["poles"]))]


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
