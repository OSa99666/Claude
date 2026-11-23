extends Node3D
## Transport / sorting simulation: user interface + 3D view.
##
## All logic runs in the Python simulator (transportsim).  This program
##  * receives the state as JSON over UDP (default port 9874) and draws it,
##  * sends the parameter/command values from the panel back to the simulator
##    (UDP, port 9875, to the address the state packets come from).
##
## Layout (world frame, metres): main conveyor T1 along +X, ejectors push towards +Z onto the side
## conveyors T2..T5 (ejector 1..4); the ejector pistons stand on the -Z side.  Positions arrive in
## ticks; an item's centre is at (pos + tpi/2) * tick_m.
##
## Run:  godot --path godot -- [--port 9874] [--cmd-port 9875]

var port := 9874
var cmd_port := 9875
var udp := PacketPeerUDP.new()
var cmd := PacketPeerUDP.new()
var sim_host := ""

var cam := Camera3D.new()
var hud := Label.new()
var panel := PanelContainer.new()
var controls := {}                  # key -> SpinBox / CheckBox / Array[CheckBox]
var seqs := {}                      # key (ending _seq) -> counter
var ui_ready := false
var send_timer := 0.0
var msg := {}
var last_rx_ms := -100000
var built_key := ""

var world := Node3D.new()           # rebuilt when the geometry changes
var item_nodes: Array[MeshInstance3D] = []
var item_mat: Array[StandardMaterial3D] = []
var stripes: Array = []             # [belt index, MeshInstance3D, phase offset]
var belt_info: Array = []           # per belt: {len, along, origin, pitch}
var stripe_phase := [0.0, 0.0, 0.0, 0.0, 0.0]
var pistons: Array[MeshInstance3D] = []
var rods: Array[MeshInstance3D] = []
var front_lamps: Array[MeshInstance3D] = []
var det_lamp := MeshInstance3D.new()
var store_boxes: Array[MeshInstance3D] = []
var motor_lamps: Array[MeshInstance3D] = []

var yaw := 0.75
var pitch := 0.6
var dist := 7.0
var target := Vector3(2.4, 0.0, 0.8)

const TYPE_COLOURS := [Color(0.5, 0.5, 0.5), Color(0.2, 0.35, 1.0), Color(0.2, 0.8, 0.3),
		Color(0.9, 0.2, 0.2), Color(0.95, 0.85, 0.2), Color(0.85, 0.85, 0.85)]
# T_XCQ identification word (ARGB item colour) -> item type 1..4; type 5 (0xFFD0D0D0) is not detected
const DETECT_TYPES := {0xFF000065: 1, 0xFF006500: 2, 0xFF650000: 3, 0xFF656500: 4}
const METAL := Color(0.6, 0.63, 0.68)     # frame / sensor housing colour; detectors show this when off
const BELT_W := 0.3
const ITEM_H := 0.16


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
	add_child(world)
	add_child(cam)
	cam.current = true
	_update_camera()

	var layer := CanvasLayer.new()
	hud.position = Vector2(12, 8)
	hud.add_theme_font_size_override("font_size", 14)
	hud.add_theme_font_override("font", _mono())
	layer.add_child(hud)
	_build_panel(layer)
	add_child(layer)


func _mono() -> Font:
	var f := SystemFont.new()
	f.font_names = PackedStringArray(["monospace", "DejaVu Sans Mono", "Liberation Mono"])
	return f


# ------------------------------------------------------------------ helpers
func _mat(c: Color, metal := 0.0) -> StandardMaterial3D:
	var m := StandardMaterial3D.new()
	m.albedo_color = c
	m.metallic = metal
	m.roughness = 0.55
	return m


func _box(size: Vector3, pos: Vector3, mat: Material, parent: Node = null) -> MeshInstance3D:
	var mi := MeshInstance3D.new()
	var bm := BoxMesh.new()
	bm.size = size
	mi.mesh = bm
	mi.material_override = mat
	mi.position = pos
	(parent if parent != null else world).add_child(mi)
	return mi


func _cyl(radius: float, height: float, pos: Vector3, rot_deg: Vector3) -> MeshInstance3D:
	var mi := MeshInstance3D.new()
	var cm := CylinderMesh.new()
	cm.top_radius = radius
	cm.bottom_radius = radius
	cm.height = height
	mi.mesh = cm
	mi.material_override = _mat(Color(0.3, 0.1, 0.1), 0.3)
	mi.position = pos
	mi.rotation_degrees = rot_deg
	world.add_child(mi)
	return mi


# ------------------------------------------------------------- scene build
func _build(g: Dictionary) -> void:
	for c in world.get_children():
		c.queue_free()
	item_nodes.clear()
	item_mat.clear()
	stripes.clear()
	belt_info.clear()
	pistons.clear()
	rods.clear()
	front_lamps.clear()
	store_boxes.clear()
	motor_lamps.clear()
	var tm := float(g["tick_m"])
	var tpi := float(g["tpi"])
	var r := tpi / 2.0
	var t1_len: float = (float(g["t1_len"]) + tpi) * tm
	var side_len: float = (float(g["side_len"]) + tpi) * tm
	var ej: Array = g["ej"]
	var belt_mat := _mat(Color(0.22, 0.23, 0.26))
	var frame_mat := _mat(METAL, 0.5)

	# main conveyor T1: x from 0 to t1_len
	_box(Vector3(t1_len, 0.06, BELT_W), Vector3(t1_len / 2.0, -0.04, 0), belt_mat)
	_box(Vector3(t1_len, 0.08, 0.03), Vector3(t1_len / 2.0, 0.0, BELT_W / 2.0 + 0.015), frame_mat)
	_box(Vector3(t1_len, 0.08, 0.03), Vector3(t1_len / 2.0, 0.0, -BELT_W / 2.0 - 0.015), frame_mat)
	belt_info.append({"len": t1_len, "along": Vector3.RIGHT, "origin": Vector3(0, -0.005, 0), "pitch": 0.25})
	# side conveyors T2..T5
	for k in 4:
		var x := float(ej[k]) * tm
		var z0 := BELT_W / 2.0
		_box(Vector3(BELT_W, 0.06, side_len), Vector3(x, -0.04, z0 + side_len / 2.0), belt_mat)
		_box(Vector3(0.03, 0.08, side_len), Vector3(x + BELT_W / 2.0 + 0.015, 0.0, z0 + side_len / 2.0), frame_mat)
		_box(Vector3(0.03, 0.08, side_len), Vector3(x - BELT_W / 2.0 - 0.015, 0.0, z0 + side_len / 2.0), frame_mat)
		belt_info.append({"len": side_len, "along": Vector3.BACK, "origin": Vector3(x, -0.005, z0), "pitch": 0.25})
		# ejector: fixed cylinder housing on the -Z side, moving rod + pusher plate
		var cyl := MeshInstance3D.new()
		var cm := CylinderMesh.new()
		cm.top_radius = 0.07
		cm.bottom_radius = 0.07
		cm.height = 0.5
		cyl.mesh = cm
		cyl.material_override = _mat(Color(0.3, 0.4, 0.6), 0.4)
		cyl.position = Vector3(x, 0.08, -BELT_W / 2.0 - 0.25)
		cyl.rotation_degrees = Vector3(90, 0, 0)
		world.add_child(cyl)
		var rod := _box(Vector3(0.03, 0.03, 0.5), Vector3(x, 0.08, -BELT_W / 2.0 - 0.25), _mat(Color(0.8, 0.8, 0.85), 0.8))
		var plate := _box(Vector3(0.18, 0.15, 0.02), Vector3(0, 0, 0.26), _mat(Color(0.9, 0.55, 0.15)), rod)
		plate.name = "plate"
		rods.append(rod)
		# "item in front of ejector" sensor: bracket on the -Z side, sensor head above the ejection position
		var bx := x - 0.14
		_box(Vector3(0.03, 0.32, 0.03), Vector3(bx, 0.15, -BELT_W / 2.0 - 0.04), frame_mat)
		_box(Vector3(0.03, 0.03, BELT_W / 2.0 + 0.06), Vector3(bx, 0.31, -BELT_W / 4.0 - 0.01), frame_mat)
		_box(Vector3(0.14 + 0.03, 0.03, 0.03), Vector3(x - 0.07, 0.31, 0.0), frame_mat)
		var lamp := _box(Vector3(0.08, 0.05, 0.08), Vector3(x, 0.27, 0.0), _mat(METAL, 0.5))
		front_lamps.append(lamp)
		# store at the end of the side conveyor
		var sb := _box(Vector3(0.3, 0.2, 0.3), Vector3(x, 0.0, z0 + side_len + 0.2), _mat(Color(0.35, 0.35, 0.4)))
		store_boxes.append(sb)
	# store 1 at the end of T1
	store_boxes.push_front(_box(Vector3(0.3, 0.2, 0.3), Vector3(t1_len + 0.2, 0.0, 0), _mat(Color(0.35, 0.35, 0.4))))
	# detector gate (centre of the detection window)
	var dx := (float(g["det"]) + r) * tm
	_box(Vector3(0.04, 0.3, 0.04), Vector3(dx, 0.1, BELT_W / 2.0 + 0.05), frame_mat)
	_box(Vector3(0.04, 0.3, 0.04), Vector3(dx, 0.1, -BELT_W / 2.0 - 0.05), frame_mat)
	_box(Vector3(0.04, 0.04, BELT_W + 0.14), Vector3(dx, 0.25, 0), frame_mat)
	det_lamp = _box(Vector3(0.08, 0.08, 0.08), Vector3(dx, 0.32, 0), _mat(METAL, 0.5))
	# motors (one per conveyor): cylinders at the side of each belt's start; colour = running
	motor_lamps.append(_cyl(0.07, 0.2, Vector3(0.2, 0.08, -BELT_W / 2.0 - 0.14), Vector3(90, 0, 0)))
	for k in 4:
		motor_lamps.append(_cyl(0.07, 0.2, Vector3(float(ej[k]) * tm + BELT_W / 2.0 + 0.14, 0.08, BELT_W / 2.0 + 0.2), Vector3(0, 0, 90)))
	# belt stripes (move with the belt speed)
	for b in belt_info.size():
		var info: Dictionary = belt_info[b]
		var n := int(float(info["len"]) / float(info["pitch"]))
		for i in n:
			var along: Vector3 = info["along"]
			var size := Vector3(0.02, 0.012, BELT_W * 0.9) if along == Vector3.RIGHT else Vector3(BELT_W * 0.9, 0.012, 0.02)
			var s := _box(size, info["origin"], _mat(Color(0.4, 0.42, 0.46)))
			stripes.append([b, s, float(i) * float(info["pitch"])])
	built_key = str(g)
	# camera target: centre of T1
	target = Vector3(t1_len / 2.0, 0.0, side_len / 2.0)
	_update_camera()


# ---------------------------------------------------------------------- panel
func _build_panel(layer: CanvasLayer) -> void:
	panel.anchor_left = 1.0
	panel.anchor_right = 1.0
	panel.offset_left = -430
	panel.offset_right = -10
	panel.offset_top = 10
	panel.offset_bottom = 780
	panel.grow_horizontal = Control.GROW_DIRECTION_BEGIN
	panel.visible = false
	layer.add_child(panel)
	var tabs := TabContainer.new()
	panel.add_child(tabs)

	var feed := _tab(tabs, "Items")
	_heading(feed, "Item feed on T1")
	_check(feed, "Feed items  (XE)", "feed")
	_check(feed, "Allow untreated items, type 5  (XE01)", "feed_untreated")
	_button(feed, "Feed single item  (XES)", "feed_single_seq")
	_button(feed, "Empty transport  (RX)", "empty_transport_seq")
	_heading(feed, "Stores")
	for n in range(1, 6):
		_button(feed, "Empty store %d  (RXQ0%d)" % [n, n], "empty_store%d_seq" % n)
	_button(feed, "Enter untreated item in store 1  (XES01)", "enter_store1_seq")

	var tim := _tab(tabs, "Timing")
	_heading(tim, "Motors [s]")
	_spin(tim, "Start delay", 0.0, 10.0, 0.05, "motor_start")
	_spin(tim, "Stop delay", 0.0, 10.0, 0.05, "motor_stop")
	_heading(tim, "Ejectors [s]")
	_spin(tim, "Extend time", 0.01, 10.0, 0.05, "ej_extend")
	_spin(tim, "Retract time", 0.01, 10.0, 0.05, "ej_retract")
	_spin(tim, "Delay before retracting", 0.0, 10.0, 0.05, "ej_delay_high")
	_spin(tim, "Delay before extending", 0.0, 10.0, 0.05, "ej_delay_low")
	_heading(tim, "Conveyor speed (animation tick period)")
	_spin(tim, "Period at full speed [ms]", 2.0, 500.0, 1.0, "period_min_ms")
	_spin(tim, "Period at min speed [ms]", 3.0, 1000.0, 1.0, "period_max_ms")
	_spin(tim, "Ticks per animation tick", 1.0, 10.0, 1.0, "anim_ticks")

	var frc := _tab(tabs, "Forces")
	_array_checks(frc, "Force stop motor  (FSL11-15)", "force_stop_motor", 5)
	_array_checks(frc, "Force extend ejector  (FSH21-24)", "force_extend", 4)
	_array_checks(frc, "Force retract ejector  (FSL21-24)", "force_retract", 4)
	_array_checks(frc, "Feedbacks extended+retracted TRUE  (FH21-24)", "force_fb_true", 4)
	_array_checks(frc, "Feedbacks extended+retracted FALSE  (FL21-24)", "force_fb_false", 4)
	_array_checks(frc, "Item-in-front sensor TRUE  (FH11-14)", "force_front_true", 4)
	_array_checks(frc, "Item-in-front sensor FALSE  (FL11-14)", "force_front_false", 4)


func _tab(tabs: TabContainer, title: String) -> VBoxContainer:
	var sc := ScrollContainer.new()
	sc.name = title
	sc.custom_minimum_size = Vector2(400, 700)
	var box := VBoxContainer.new()
	sc.add_child(box)
	tabs.add_child(sc)
	return box


func _heading(parent: Control, text: String) -> void:
	var l := Label.new()
	l.text = text
	l.add_theme_color_override("font_color", Color(0.75, 0.85, 1.0))
	parent.add_child(l)


func _spin(parent: Control, text: String, lo: float, hi: float, step: float, key: String) -> void:
	var row := HBoxContainer.new()
	var l := Label.new()
	l.text = text
	l.custom_minimum_size = Vector2(240, 0)
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


func _array_checks(parent: Control, text: String, key: String, n: int) -> void:
	_heading(parent, text)
	var row := HBoxContainer.new()
	var arr: Array = []
	for i in n:
		var cb := CheckBox.new()
		cb.text = str(i + 1)
		cb.toggled.connect(func(_on: bool) -> void: _send_ui())
		row.add_child(cb)
		arr.append(cb)
	parent.add_child(row)
	controls[key] = arr


func _button(parent: Control, text: String, key: String) -> void:
	var b := Button.new()
	b.text = text
	seqs[key] = 0
	b.pressed.connect(func() -> void:
		seqs[key] += 1
		_send_ui())
	parent.add_child(b)


func _fill_ui(p: Dictionary) -> void:
	for k in p:
		if not controls.has(k):
			continue
		var c = controls[k]
		if c is SpinBox:
			c.set_value_no_signal(float(p[k]))
		elif c is CheckBox:
			c.set_pressed_no_signal(bool(p[k]))
		elif c is Array:
			for i in c.size():
				c[i].set_pressed_no_signal(bool(p[k][i]))
	panel.visible = true
	ui_ready = true


func _send_ui() -> void:
	if not ui_ready or sim_host == "":
		return
	var d := {}
	for k in seqs:
		d[k] = seqs[k]
	for k in controls:
		var c = controls[k]
		if c is SpinBox:
			d[k] = c.value
		elif c is CheckBox:
			d[k] = c.button_pressed
		elif c is Array:
			var a := []
			for cb in c:
				a.append(cb.button_pressed)
			d[k] = a
	cmd.put_packet(JSON.stringify(d).to_utf8_buffer())


# -------------------------------------------------------------------- update
func _lamp(m: MeshInstance3D, on: bool, colour: Color, off_colour := Color(0.25, 0.1, 0.1)) -> void:
	(m.material_override as StandardMaterial3D).albedo_color = colour if on else off_colour


func _item_node(i: int) -> MeshInstance3D:
	while item_nodes.size() <= i:
		var mi := MeshInstance3D.new()
		var bm := BoxMesh.new()
		bm.size = Vector3(0.2, ITEM_H, 0.2)
		mi.mesh = bm
		var m := _mat(Color.WHITE)
		mi.material_override = m
		add_child(mi)
		item_nodes.append(mi)
		item_mat.append(m)
	return item_nodes[i]


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

	var g: Dictionary = msg["geo"]
	if str(g) != built_key:
		_build(g)
	var tm := float(g["tick_m"])
	var r := float(g["tpi"]) / 2.0
	var idx := 0

	# items on T1 and the side conveyors
	for it in msg["t1"]:
		var n := _item_node(idx)
		n.visible = true
		n.position = Vector3((float(it[1]) + r) * tm, ITEM_H / 2.0 + 0.0, 0)
		item_mat[idx].albedo_color = TYPE_COLOURS[int(it[0])]
		idx += 1
	for k in 4:
		var x := float(g["ej"][k]) * tm
		for it in msg["side"][k]:
			var n := _item_node(idx)
			n.visible = true
			n.position = Vector3(x, ITEM_H / 2.0, BELT_W / 2.0 + (float(it[1]) + r) * tm)
			item_mat[idx].albedo_color = TYPE_COLOURS[int(it[0])]
			idx += 1
	for j in range(idx, item_nodes.size()):
		item_nodes[j].visible = false

	# belt stripes
	var vel: Array = msg["vel"]
	for b in 5:
		stripe_phase[b] = fposmod(float(stripe_phase[b]) + float(vel[b]) * tm * delta, 0.25)
	for s in stripes:
		var b: int = s[0]
		var info: Dictionary = belt_info[b]
		var d := fposmod(float(s[2]) + float(stripe_phase[b]), float(info["len"]))
		(s[1] as MeshInstance3D).position = (info["origin"] as Vector3) + (info["along"] as Vector3) * d

	# ejectors, lamps, stores
	var ejp: Array = msg["ej_pos"]
	for k in 4:
		rods[k].position.z = -BELT_W / 2.0 - 0.25 + float(ejp[k]) * 0.3
	var io: Dictionary = msg["io"]
	var run: Array = msg["run"]
	for b in 5:
		_lamp(motor_lamps[b], bool(run[b]), Color(0.2, 1.0, 0.3))
	for k in 4:
		_lamp(front_lamps[k], bool(io.get("T_XCQ1%d" % (k + 1), false)), Color(1.0, 0.8, 0.1), METAL)
	var xcq := int(io.get("T_XCQ", 0))
	# gate lamp: colour of the detected item (types 1-4); off for no item and for untreated items (type 5)
	var det_type: int = DETECT_TYPES.get(xcq, 0)
	_lamp(det_lamp, det_type > 0, TYPE_COLOURS[det_type], METAL)
	var st: Array = msg["store"]
	for n in 5:
		var sm := store_boxes[n].material_override as StandardMaterial3D
		sm.albedo_color = TYPE_COLOURS[int(st[n])] if int(st[n]) > 0 else Color(0.35, 0.35, 0.4)

	hud.text = _hud(io, msg.get("cmd", {}), run, st)


func _b(v) -> String:
	return "#" if bool(v) else "."


func _hud(io: Dictionary, c: Dictionary, run: Array, st: Array) -> String:
	var t := "OPC UA clients: %d%s\n\n" % [int(msg.get("clients", 0)), "" if msg.get("plc", false) else "   (no PLC - motors/ejectors idle)"]
	t += "            T1     T2     T3     T4     T5\n"
	t += "YH  start   "
	for i in 5:
		t += "%s      " % _b(c.get("T_YH1%d" % (i + 1), false))
	t += "\nXGH running "
	for i in 5:
		t += "%s      " % _b(io.get("T_XGH1%d" % (i + 1), false))
	t += "\nYR  speed   "
	for i in 5:
		t += str(int(c.get("T_YR1%d" % (i + 1), 0))).rpad(7)
	t += "\n\n            E1     E2     E3     E4\n"
	t += "YH  extend  "
	for i in 4:
		t += "%s      " % _b(c.get("T_YH2%d" % (i + 1), false))
	t += "\nYL  retract "
	for i in 4:
		t += "%s      " % _b(c.get("T_YL2%d" % (i + 1), false))
	t += "\nXGH extended"
	for i in 4:
		t += " %s     " % _b(io.get("T_XGH2%d" % (i + 1), false))
	t += "\nXGL retracted"
	for i in 4:
		t += "%s     " % _b(io.get("T_XGL2%d" % (i + 1), false))
	t += "\nXCQ1x front "
	for i in 4:
		t += "%s      " % _b(io.get("T_XCQ1%d" % (i + 1), false))
	t += "\nXCQ0x detect"
	for i in 4:
		t += " %s     " % _b(io.get("T_XCQ0%d" % (i + 1), false))
	t += "\n\nT_XCQ  0x%08X     T_X11 (T1 velocity) %d\n" % [int(io.get("T_XCQ", 0)), int(io.get("T_X11", 0))]
	t += "Stores BQ01-05: "
	for n in 5:
		t += "%s " % _b(io.get("T_BQ0%d" % (n + 1), false))
	return t


func _unhandled_input(ev: InputEvent) -> void:
	if ev is InputEventMouseMotion and (ev.button_mask & MOUSE_BUTTON_MASK_LEFT):
		yaw -= ev.relative.x * 0.005
		pitch = clampf(pitch + ev.relative.y * 0.005, 0.1, 1.5)
		_update_camera()
	elif ev is InputEventMouseButton and ev.pressed:
		if ev.button_index == MOUSE_BUTTON_WHEEL_UP:
			dist = maxf(1.0, dist * 0.92)
			_update_camera()
		elif ev.button_index == MOUSE_BUTTON_WHEEL_DOWN:
			dist = minf(30.0, dist * 1.08)
			_update_camera()


func _update_camera() -> void:
	var p := target + Vector3(sin(yaw) * cos(pitch), sin(pitch), cos(yaw) * cos(pitch)) * dist
	cam.look_at_from_position(p, target, Vector3.UP)
