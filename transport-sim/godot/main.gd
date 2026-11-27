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
var rest_nodes: Array[MeshInstance3D] = []     # item resting in each store bin
var rest_mat: Array[StandardMaterial3D] = []
var rest_id := [0, 0, 0, 0, 0]
var store_in_seen := [-1, -1, -1, -1, -1]
var fall_items: Array = []                     # items rolling off a belt end into a bin
var end_pos: Array[Vector3] = []               # where items leave belt n (0 = T1, 1..4 = T2..T5)
var end_dir: Array[Vector3] = []
var colours_argb: Array = [0, 0xFF000065, 0xFF006500, 0xFF650000, 0xFF656500, 0xFFD0D0D0]   # T_XCQ item colours (from the simulator)
var motor_lamps: Array[MeshInstance3D] = []
var motor_leds: Array = []                    # per motor: [command LED (T_YH1n), running LED (T_XGH1n)]
var ej_leds: Array = []                       # per ejector: [T_YH2n, T_YL2n (commands), T_XGH2n, T_XGL2n (feedbacks)]

var yaw := 0.75
var pitch := 0.6
var dist := 7.0
var target := Vector3(2.4, 0.0, 0.8)

const LED_HIGH := Color8(0x00, 0x65, 0x00)      # dark green: command / High / running
const LED_LOW := Color8(0x65, 0x65, 0x00)       # dark yellow: Low / retracted
const METAL := Color(0.6, 0.63, 0.68)     # frame / sensor housing colour; detectors show this when off
const BELT_W := 0.3
const FLOOR_Y := -0.60                    # top of the ground / bin floors
const RAMP_X0 := -0.45                    # item centre at the top of the feed ramp (x, m)
const RAMP_SLOPE := 0.4                   # feed ramp: height = -0.4 * x for x in [-0.5, 0]
const BIN := 0.4                          # inner size of the store bins
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
	_save_home()

	var layer := CanvasLayer.new()
	hud.position = Vector2(12, 8)
	hud.add_theme_font_size_override("font_size", 14)
	hud.add_theme_font_override("font", _mono())
	layer.add_child(hud)
	_add_help(layer)
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


## ARGB item colour word (as delivered on T_XCQ) -> Godot Color
func _argb(c: int) -> Color:
	return Color(float((c >> 16) & 255) / 255.0, float((c >> 8) & 255) / 255.0,
			float(c & 255) / 255.0, float((c >> 24) & 255) / 255.0)


func _tc(item_id: int) -> Color:
	return _argb(int(colours_argb[clampi(item_id, 0, 5)]))


func _glass() -> StandardMaterial3D:
	var m := StandardMaterial3D.new()
	m.albedo_color = Color(0.75, 0.85, 0.95, 0.22)
	m.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	return m


func _cyl(radius: float, height: float, pos: Vector3, rot_deg: Vector3) -> MeshInstance3D:
	var mi := MeshInstance3D.new()
	var cm := CylinderMesh.new()
	cm.top_radius = radius
	cm.bottom_radius = radius
	cm.height = height
	mi.mesh = cm
	mi.material_override = _mat(METAL, 0.5)
	mi.position = pos
	mi.rotation_degrees = rot_deg
	world.add_child(mi)
	return mi


## Open-top container: floor + four walls, centre of the floor at `p`.
func _bin(p: Vector3, m: Material) -> void:
	var h := 0.4
	_box(Vector3(BIN + 0.04, 0.02, BIN + 0.04), p + Vector3(0, 0.01, 0), m)
	_box(Vector3(0.02, h, BIN + 0.04), p + Vector3(BIN / 2.0 + 0.01, h / 2.0, 0), m)
	_box(Vector3(0.02, h, BIN + 0.04), p + Vector3(-BIN / 2.0 - 0.01, h / 2.0, 0), m)
	_box(Vector3(BIN + 0.04, h, 0.02), p + Vector3(0, h / 2.0, BIN / 2.0 + 0.01), m)
	_box(Vector3(BIN + 0.04, h, 0.02), p + Vector3(0, h / 2.0, -BIN / 2.0 - 0.01), m)


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
	rest_nodes.clear()
	rest_mat.clear()
	fall_items.clear()
	end_pos.clear()
	end_dir.clear()
	motor_lamps.clear()
	motor_leds.clear()
	ej_leds.clear()
	var tm := float(g["tick_m"])
	var tpi := float(g["tpi"])
	var r := tpi / 2.0
	# belts end where an item's centre leaves them: items tip over the edge into the store bins
	var t1_len: float = (float(g["t1_len"]) + r) * tm
	var side_len: float = (float(g["side_len"]) + r) * tm
	var ej: Array = g["ej"]
	var belt_mat := _mat(Color(0.22, 0.23, 0.26))
	var frame_mat := _mat(METAL, 0.5)
	var bin_mat := _mat(Color(0.35, 0.36, 0.42), 0.2)

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
		_add_ejector_leds(x, "E%d" % (k + 1))
		# "item in front of ejector" sensor: bracket on the -Z side, sensor head above the ejection position
		var bx := x - 0.14
		_box(Vector3(0.03, 0.32, 0.03), Vector3(bx, 0.15, -BELT_W / 2.0 - 0.04), frame_mat)
		_box(Vector3(0.03, 0.03, BELT_W / 2.0 + 0.06), Vector3(bx, 0.31, -BELT_W / 4.0 - 0.01), frame_mat)
		_box(Vector3(0.14 + 0.03, 0.03, 0.03), Vector3(x - 0.07, 0.31, 0.0), frame_mat)
		var lamp := _box(Vector3(0.08, 0.05, 0.08), Vector3(x, 0.27, 0.0), _mat(METAL, 0.5))
		front_lamps.append(lamp)
		# store bin below the end of the side conveyor
		end_pos.append(Vector3(x, ITEM_H / 2.0, z0 + side_len))
		end_dir.append(Vector3.BACK)
		_bin(Vector3(x, FLOOR_Y, z0 + side_len + 0.1), bin_mat)
		# legs
		for lz in [z0 + 0.3, z0 + side_len - 0.3]:
			for lx in [-0.1, 0.1]:
				_cyl(0.025, 0.55, Vector3(x + lx, -0.345, lz), Vector3.ZERO)
	# store 1 below the end of T1
	end_pos.push_front(Vector3(t1_len, ITEM_H / 2.0, 0))
	end_dir.push_front(Vector3.RIGHT)
	_bin(Vector3(t1_len + 0.1, FLOOR_Y, 0), bin_mat)
	for lx in [0.4, t1_len / 2.0, t1_len - 0.4]:
		for lz in [-0.1, 0.1]:
			_cyl(0.025, 0.55, Vector3(lx, -0.345, lz), Vector3.ZERO)
	_box(Vector3(t1_len + 2.5, 0.02, side_len + 2.5), Vector3(t1_len / 2.0, FLOOR_Y - 0.01, side_len / 2.0), _mat(Color(0.17, 0.18, 0.2)))
	# containment (hopper) with a feed ramp at the start of T1: new items roll down the ramp onto the belt
	var ang := atan(RAMP_SLOPE)
	var ramp := _box(Vector3(0.54, 0.02, BELT_W + 0.02), Vector3(-0.25, 0.1 - 0.01, 0), frame_mat)
	ramp.rotation = Vector3(0, 0, -ang)
	_box(Vector3(0.03, 0.6, BELT_W + 0.12), Vector3(-0.53, 0.3, 0), frame_mat)
	for sz in [-1.0, 1.0]:
		_box(Vector3(0.5, 0.5, 0.02), Vector3(-0.3, 0.3, sz * (BELT_W / 2.0 + 0.06)), _glass())
	_box(Vector3(0.46, 0.02, BELT_W + 0.14), Vector3(-0.3, 0.55, 0), _glass())
	for lz in [-0.1, 0.1]:
		_cyl(0.025, 0.65, Vector3(-0.4, -0.295, lz), Vector3.ZERO)
	# item resting in each bin (shown while the store holds an item)
	for n in 5:
		var bp: Vector3 = end_pos[n] + end_dir[n] * 0.1
		var rm := _mat(Color.WHITE)
		var rn := _box(Vector3(0.2, ITEM_H, 0.2), Vector3(bp.x, FLOOR_Y + 0.02 + ITEM_H / 2.0, bp.z), rm)
		rn.visible = false
		rest_nodes.append(rn)
		rest_mat.append(rm)
	# detector gate (centre of the detection window)
	var dx := (float(g["det"]) + r) * tm
	_box(Vector3(0.04, 0.3, 0.04), Vector3(dx, 0.1, BELT_W / 2.0 + 0.05), frame_mat)
	_box(Vector3(0.04, 0.3, 0.04), Vector3(dx, 0.1, -BELT_W / 2.0 - 0.05), frame_mat)
	_box(Vector3(0.04, 0.04, BELT_W + 0.14), Vector3(dx, 0.25, 0), frame_mat)
	det_lamp = _box(Vector3(0.08, 0.08, 0.08), Vector3(dx, 0.32, 0), _mat(METAL, 0.5))
	# motors (one per conveyor): cylinders at the side of each belt's start; colour = running
	motor_lamps.append(_cyl(0.07, 0.2, Vector3(0.2, 0.08, -BELT_W / 2.0 - 0.14), Vector3(90, 0, 0)))
	_add_motor_leds(Vector3(0.2, 0.08, -BELT_W / 2.0 - 0.14), Vector3.BACK, "T1")
	for k in 4:
		var mp := Vector3(float(ej[k]) * tm + BELT_W / 2.0 + 0.14, 0.08, BELT_W / 2.0 + 0.2)
		motor_lamps.append(_cyl(0.07, 0.2, mp, Vector3(0, 0, 90)))
		_add_motor_leds(mp, Vector3.RIGHT, "T%d" % (k + 2))
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
	target = Vector3(t1_len / 2.0, -0.2, side_len / 2.0)
	_update_camera()
	_save_home()


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
		world.add_child(mi)
		item_nodes.append(mi)
		item_mat.append(m)
	return item_nodes[i]


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
		_place_t1_item(n, float(it[1]), float(g["tpi"]), tm)
		item_mat[idx].albedo_color = _tc(int(it[0]))
		idx += 1
	for k in 4:
		var x := float(g["ej"][k]) * tm
		for it in msg["side"][k]:
			var n := _item_node(idx)
			n.visible = true
			n.position = Vector3(x, ITEM_H / 2.0, BELT_W / 2.0 + (float(it[1]) + r) * tm)
			n.rotation = Vector3.ZERO
			item_mat[idx].albedo_color = _tc(int(it[0]))
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
	var cmd_in: Dictionary = msg.get("cmd", {})
	for k in 4:
		var e: Array = ej_leds[k]
		_led(e[0], bool(cmd_in.get("T_YH2%d" % (k + 1), false)), LED_HIGH)
		_led(e[1], bool(cmd_in.get("T_YL2%d" % (k + 1), false)), LED_LOW)
		_led(e[2], bool(io.get("T_XGH2%d" % (k + 1), false)), LED_HIGH)
		_led(e[3], bool(io.get("T_XGL2%d" % (k + 1), false)), LED_LOW)
	for b in 5:
		_lamp(motor_lamps[b], bool(run[b]), _tc(2), METAL)     # same green as the type-2 items
		_led(motor_leds[b][0], bool(cmd_in.get("T_YH1%d" % (b + 1), false)), LED_HIGH)
		_led(motor_leds[b][1], bool(io.get("T_XGH1%d" % (b + 1), false)), LED_HIGH)
	for k in 4:
		_lamp(front_lamps[k], bool(io.get("T_XCQ1%d" % (k + 1), false)), Color(1.0, 0.8, 0.1), METAL)
	var xcq := int(io.get("T_XCQ", 0))
	# gate lamp: exactly the colour word the PLC reads (T_XCQ); off for no item and for untreated items (type 5)
	if msg.has("colours"):
		colours_argb = msg["colours"]
	var det_on := xcq != 0 and xcq != int(colours_argb[5])
	_lamp(det_lamp, det_on, _argb(xcq), METAL)
	var st: Array = msg["store"]
	_update_stores(st, msg.get("store_in", [0, 0, 0, 0, 0]), vel, tm, delta)

	hud.text = _hud(io, msg.get("cmd", {}), run, st)


## Item on T1.  The first item length is the feed ramp of the hopper (visual only): the item rolls down
## the ramp and drops onto the belt.
func _place_t1_item(n: MeshInstance3D, pos: float, tpi: float, tm: float) -> void:
	var s := clampf(pos / tpi, 0.0, 1.0)
	var cx := lerpf(RAMP_X0, (tpi + tpi / 2.0) * tm, s)
	if pos >= tpi:
		cx = (pos + tpi / 2.0) * tm
	var blend := clampf(1.0 - cx / 0.12, 0.0, 1.0)             # 1 on the ramp, fades out just after its end
	var h := maxf(0.0, -RAMP_SLOPE * cx)
	n.position = Vector3(cx, ITEM_H / 2.0 + h * blend + ITEM_H * 0.08 * blend, 0)
	n.rotation = Vector3(0, 0, -atan(RAMP_SLOPE) * blend)


## Items that leave a belt end tip over the edge and fall into the store bin; the bin then shows the item.
func _update_stores(st: Array, store_in: Array, vel: Array, tm: float, delta: float) -> void:
	for n in 5:
		var cnt := int(store_in[n])
		if int(store_in_seen[n]) < 0 or cnt < int(store_in_seen[n]):
			store_in_seen[n] = cnt                            # first packet / simulator restarted
		elif cnt > int(store_in_seen[n]):
			for _i in mini(cnt - int(store_in_seen[n]), 2):
				_spawn_fall(n, int(st[n]), float(vel[n]) * tm)
			store_in_seen[n] = cnt
	var i := 0
	while i < fall_items.size():
		var f: Dictionary = fall_items[i]
		var node: MeshInstance3D = f["node"]
		f["vy"] = float(f["vy"]) - 9.81 * delta
		f["pos"] = (f["pos"] as Vector3) + Vector3((f["dir"] as Vector3).x * float(f["speed"]) * delta, float(f["vy"]) * delta, (f["dir"] as Vector3).z * float(f["speed"]) * delta)
		f["angle"] = minf(float(f["angle"]) + 4.9 * delta, PI / 2.0)       # tips forward while falling
		node.position = f["pos"]
		var axis := Vector3(0, 0, -1) if (f["dir"] as Vector3) == Vector3.RIGHT else Vector3(1, 0, 0)
		node.basis = Basis(axis, float(f["angle"]))
		if (f["pos"] as Vector3).y <= FLOOR_Y + 0.02 + ITEM_H / 2.0 + 0.01:
			rest_id[int(f["store"])] = int(f["id"])
			node.queue_free()
			fall_items.remove_at(i)
		else:
			i += 1
	for n in 5:
		var falling := false
		for f in fall_items:
			if int(f["store"]) == n:
				falling = true
		if not falling and rest_id[n] != int(st[n]):
			rest_id[n] = int(st[n])                           # e.g. item entered directly by the UI, or store emptied
		rest_nodes[n].visible = int(rest_id[n]) > 0
		if int(rest_id[n]) > 0:
			rest_mat[n].albedo_color = _tc(int(rest_id[n]))


func _spawn_fall(n: int, item_id: int, speed: float) -> void:
	if item_id <= 0:
		return
	var mi := MeshInstance3D.new()
	var bm := BoxMesh.new()
	bm.size = Vector3(0.2, ITEM_H, 0.2)
	mi.mesh = bm
	mi.material_override = _mat(_tc(item_id))
	mi.position = end_pos[n]
	world.add_child(mi)
	fall_items.append({"node": mi, "store": n, "id": item_id, "pos": end_pos[n], "dir": end_dir[n],
			"speed": speed, "vy": 0.0, "angle": 0.0})


## One indicator LED (sphere) at `pos`; coloured and glowing when on, metal when off.
func _led_node(pos: Vector3) -> MeshInstance3D:
	var led := MeshInstance3D.new()
	var lm := SphereMesh.new()
	lm.radius = 0.016
	lm.height = 0.032
	led.mesh = lm
	led.material_override = _mat(METAL, 0.5)
	led.position = pos
	world.add_child(led)
	return led


func _led(m: MeshInstance3D, on: bool, colour: Color) -> void:
	var mat := m.material_override as StandardMaterial3D
	mat.albedo_color = colour if on else METAL
	mat.emission_enabled = on
	mat.emission = colour
	mat.emission_energy_multiplier = 1.5


func _caption(text: String, pos: Vector3) -> void:
	var lab := Label3D.new()
	lab.text = text
	lab.font_size = 28
	lab.pixel_size = 0.0018
	lab.billboard = BaseMaterial3D.BILLBOARD_ENABLED
	lab.modulate = Color(0.6, 0.65, 0.75)
	lab.position = pos
	world.add_child(lab)


## Dark panel lying on top of a cylinder (sunk in a little so it sits on the housing); `size` is (x, z) in metres.
func _led_panel(cyl_pos: Vector3, size: Vector2) -> void:
	_box(Vector3(size.x, 0.012, size.y), cyl_pos + Vector3(0, 0.056, 0), _mat(Color(0.25, 0.3, 0.4), 0.3))


## Motor: panel on top of the motor with the command LED (T_YH1n, start) and the running LED (T_XGH1n),
## side by side along `axis` (the motor's axis).
func _add_motor_leds(motor_pos: Vector3, axis: Vector3, label: String) -> void:
	_led_panel(motor_pos, Vector2(0.12, 0.07) if axis == Vector3.RIGHT else Vector2(0.07, 0.12))
	var p := motor_pos + Vector3(0, 0.072, 0)
	motor_leds.append([_led_node(p - axis * 0.032), _led_node(p + axis * 0.032)])
	_caption(label, motor_pos + Vector3(0, 0.16, 0))


## Ejector: LEDs directly on the top of the cylinder housing, at its back end (away from the belt):
## command row (T_YH2n extend, T_YL2n retract) at the very end, feedback row (T_XGH2n extended, T_XGL2n retracted) in front of it.
func _add_ejector_leds(x: float, label: String) -> void:
	var zb := -BELT_W / 2.0 - 0.5                        # back end of the 0.5 m housing
	var l: Array = []
	for row in 2:
		for col in 2:
			var dx := -0.032 + 0.064 * col
			var top := sqrt(0.07 * 0.07 - dx * dx)           # height of the round housing surface at this x offset
			l.append(_led_node(Vector3(x + dx, 0.08 + top + 0.006, zb + 0.05 + 0.07 * row)))
	ej_leds.append(l)
	_caption(label, Vector3(x, 0.24, zb + 0.085))


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
	if _camera_event(ev):
		return
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


# ------------------------------------------------------- camera: keyboard + middle mouse
## Arrow keys pan the view (the scene moves in the arrow's direction), Shift+arrows orbit, +/- (or PageUp/Down)
## zoom, Home resets the view; mouse: left drag orbits, middle drag pans, wheel zooms.
const CAM_PITCH_MIN := 0.1
const CAM_DIST_MIN := 1.0
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
