extends Node2D
## Intersection simulation: 2D top view + user interface.
##
## All logic runs in the Python simulator (intersectionsim).  This program
##  * receives the state as JSON over UDP (default port 9878) and draws it,
##  * sends the parameter/command values from the panel back to the simulator
##    (UDP, port 9879, to the address the state packets come from).
##
## Direction 1 = west-east road, direction 2 = north-south road.  Traffic lights show what is really lit
## (command and lamp not broken); a broken lamp gets an orange cross.  Yellow disc at a pedestrian light =
## request acknowledged by the PLC (I_YPDnA), orange square = push button pressed, orange loops = vehicle detectors.
##
## Run:  godot --path godot -- [--port 9878] [--cmd-port 9879]

var port := 9878
var cmd_port := 9879
var udp := PacketPeerUDP.new()
var cmd := PacketPeerUDP.new()
var sim_host := ""

const VIEW_HALF := 42.0          # visible half width in metres
const HALF := 4.0
const LANE := 2.0
const CW0 := 4.4
const CW1 := 7.4
const STOP := -8.0
const DET0 := -11.0
const DET1 := -8.0
const CAR_LEN := 4.5
const CAR_W := 1.9
const PANEL_W := 410.0

const GRASS := Color(0.54, 0.64, 0.5)
const ROAD := Color(0.76, 0.76, 0.76)
const ZEBRA := Color(0.9, 0.9, 0.9)
const CAR_COL := Color(0.55, 0.65, 0.9)
const CRASH_COL := Color(0.75, 0.2, 0.2)
const SIG_R := Color(0.9, 0.1, 0.1)
const SIG_Y := Color(1.0, 0.65, 0.0)
const SIG_G := Color(0.1, 0.85, 0.2)
const HOUSING := Color(0.62, 0.62, 0.62)

var hud := Label.new()
var fault_box := Label.new()
var panel := PanelContainer.new()
var controls := {}
var clear_fault_seq := 0
var reset_stats_seq := 0
var reseed_seq := 0
var ui_ready := false
var send_timer := 0.0
var msg := {}
var last_rx_ms := -100000
var scale_px := 8.0
var origin := Vector2.ZERO

const LAMP_NAMES := ["TL1R", "TL1Y", "TL1G", "TL2R", "TL2Y", "TL2G", "PL1R", "PL1G", "PL2R", "PL2G"]
const DET_NAMES := ["XTD11", "XTD12", "XTD21", "XTD22"]


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
	var layer := CanvasLayer.new()
	hud.position = Vector2(12, 8)
	hud.add_theme_font_size_override("font_size", 15)
	hud.add_theme_color_override("font_color", Color(0.05, 0.07, 0.1))
	layer.add_child(hud)
	fault_box.text = "Intersection fault"
	fault_box.add_theme_font_size_override("font_size", 16)
	fault_box.add_theme_color_override("font_color", Color(0.8, 0.0, 0.0))
	fault_box.visible = false
	layer.add_child(fault_box)
	_add_help(layer)
	_build_panel(layer)
	add_child(layer)


# ------------------------------------------------------------------ helpers
func _w(x: float, y: float) -> Vector2:
	return origin + Vector2(x, y) * scale_px


func _rect_w(x0: float, y0: float, x1: float, y1: float, col: Color) -> void:
	draw_rect(Rect2(_w(x0, y0), Vector2(x1 - x0, y1 - y0) * scale_px), col)


func _car_center(arm: String, s: float) -> Vector2:
	var m := s - CAR_LEN / 2.0
	match arm:
		"W":
			return Vector2(m, LANE)
		"E":
			return Vector2(-m, -LANE)
		"N":
			return Vector2(-LANE, m)
	return Vector2(LANE, -m)


func _car_angle(arm: String) -> float:
	match arm:
		"W":
			return 0.0
		"E":
			return PI
		"N":
			return PI / 2.0
	return -PI / 2.0


func _ped_pos(cw: String, u: float, d: int) -> Vector2:
	var f := u if d > 0 else 1.0 - u
	var a := -HALF + 2.0 * HALF * f
	var mid := (CW0 + CW1) / 2.0
	match cw:
		"N":
			return Vector2(a, -mid)
		"S":
			return Vector2(a, mid)
		"W":
			return Vector2(-mid, a)
	return Vector2(mid, a)


# --------------------------------------------------------------------- draw
func _draw() -> void:
	var vs := get_viewport_rect().size
	draw_rect(Rect2(Vector2.ZERO, vs), GRASS)
	var aw := vs.x - PANEL_W
	var ah := vs.y - 28.0
	scale_px = minf(aw, ah) / (2.0 * VIEW_HALF)
	origin = Vector2(aw / 2.0, ah / 2.0)
	var v := VIEW_HALF + 10.0
	_rect_w(-v, -HALF, v, HALF, ROAD)
	_rect_w(-HALF, -v, HALF, v, ROAD)
	# centre lines (dashed orange) outside the crossing
	var x := CW1 + 1.5
	while x < v:
		_rect_w(x, -0.08, x + 2.0, 0.08, Color(0.95, 0.55, 0.1))
		_rect_w(-x - 2.0, -0.08, -x, 0.08, Color(0.95, 0.55, 0.1))
		_rect_w(-0.08, x, 0.08, x + 2.0, Color(0.95, 0.55, 0.1))
		_rect_w(-0.08, -x - 2.0, 0.08, -x, Color(0.95, 0.55, 0.1))
		x += 4.0
	# crosswalks (zebra stripes along the travel direction) and stop lines
	var i := 0
	while i < 7:
		var a := -HALF + 0.3 + float(i) * 1.15
		_rect_w(a, -CW1, a + 0.6, -CW0, ZEBRA)
		_rect_w(a, CW0, a + 0.6, CW1, ZEBRA)
		_rect_w(-CW1, a, -CW0, a + 0.6, ZEBRA)
		_rect_w(CW0, a, CW1, a + 0.6, ZEBRA)
		i += 1
	_rect_w(STOP, 0.0, STOP + 0.3, HALF, ZEBRA)
	_rect_w(-STOP - 0.3, -HALF, -STOP, 0.0, ZEBRA)
	_rect_w(-HALF, STOP, 0.0, STOP + 0.3, ZEBRA)
	_rect_w(0.0, -STOP - 0.3, HALF, -STOP, ZEBRA)
	var font := ThemeDB.fallback_font
	draw_string(font, _w(-VIEW_HALF + 1.0, -1.0), "Direction 1", HORIZONTAL_ALIGNMENT_LEFT, -1, 16, Color(0.1, 0.1, 0.1))
	draw_string(font, _w(1.0, VIEW_HALF - 1.0), "Direction 2", HORIZONTAL_ALIGNMENT_LEFT, -1, 16, Color(0.1, 0.1, 0.1))
	if msg.is_empty():
		return

	var det: Dictionary = msg.get("det", {})
	# vehicle detector loops (stop-line side of each arm)
	var loops := {"XTD11": Rect2(DET0, 0.5, 3.0, 3.0), "XTD12": Rect2(-DET1, -3.5, 3.0, 3.0),
		"XTD21": Rect2(-3.5, DET0, 3.0, 3.0), "XTD22": Rect2(0.5, -DET1, 3.0, 3.0)}
	for k in loops:
		var r: Rect2 = loops[k]
		var on := bool(det.get(k, false))
		var col := Color(1.0, 0.55, 0.1) if on else Color(0.85, 0.5, 0.2, 0.5)
		draw_rect(Rect2(_w(r.position.x, r.position.y), r.size * scale_px), col, on, 2.0)

	# signal heads (direction 1 horizontal, direction 2 vertical) and pedestrian lights
	_head(-10.5, 5.8, true, 1)
	_head(10.5, -5.8, true, 1)
	_head(-5.8, -10.5, false, 2)
	_head(5.8, 10.5, false, 2)
	_ped_head(-5.6, -5.9, 1, 0)
	_ped_head(5.6, -5.9, 1, 1)
	_ped_head(-5.6, 5.9, 1, 0)
	_ped_head(5.6, 5.9, 1, 1)
	_ped_head(-5.9, -5.6, 2, 0)
	_ped_head(-5.9, 5.6, 2, 1)
	_ped_head(5.9, -5.6, 2, 0)
	_ped_head(5.9, 5.6, 2, 1)

	# pedestrians
	var peds: Array = msg.get("peds", [])
	for q in peds:
		var cw: String = q[0]
		var st: String = q[3]
		var dd := int(q[2])
		var p := _ped_pos(cw, float(q[1]), dd)
		if st == "wait":
			var axis := Vector2(1, 0) if (cw == "N" or cw == "S") else Vector2(0, 1)
			p = _ped_pos(cw, 0.0, dd) - axis * float(dd) * 0.9       # waits at the kerb before the crosswalk
		var col := Color(0.1, 0.1, 0.5)
		if st == "wait":
			col = Color(0.35, 0.35, 0.6)
		elif st == "hit":
			col = Color(0.8, 0.1, 0.1)
		draw_circle(_w(p.x, p.y), 0.45 * scale_px, col)

	# cars
	var cars: Array = msg.get("cars", [])
	for c in cars:
		var arm: String = c[0]
		var crashed := bool(c[2])
		var ctr := _w(_car_center(arm, float(c[1])).x, _car_center(arm, float(c[1])).y)
		draw_set_transform(ctr, _car_angle(arm), Vector2.ONE)
		var col2 := CRASH_COL if crashed else CAR_COL
		draw_rect(Rect2(Vector2(-CAR_LEN / 2.0, -CAR_W / 2.0) * scale_px, Vector2(CAR_LEN, CAR_W) * scale_px), col2)
		draw_rect(Rect2(Vector2(CAR_LEN / 2.0 - 1.5, -CAR_W / 2.0 + 0.2) * scale_px, Vector2(0.9, CAR_W - 0.4) * scale_px), col2.darkened(0.45))
		draw_rect(Rect2(Vector2(-CAR_LEN / 2.0, -CAR_W / 2.0) * scale_px, Vector2(CAR_LEN, CAR_W) * scale_px), Color(0.2, 0.25, 0.4), false, 1.0)
		draw_set_transform(Vector2.ZERO, 0.0, Vector2.ONE)

	# crash smiley
	if bool(msg.get("fault", false)):
		var cc := _w(0.0, 0.0)
		draw_circle(cc, 2.2 * scale_px, Color(1.0, 0.8, 0.1))
		draw_arc(cc, 2.2 * scale_px, 0.0, TAU, 32, Color(0.2, 0.2, 0.1), 2.0)
		for sx in [-0.8, 0.8]:
			var e := cc + Vector2(float(sx), -0.6) * scale_px
			draw_line(e + Vector2(-0.35, -0.35) * scale_px, e + Vector2(0.35, 0.35) * scale_px, Color(0.2, 0.2, 0.1), 2.0)
			draw_line(e + Vector2(-0.35, 0.35) * scale_px, e + Vector2(0.35, -0.35) * scale_px, Color(0.2, 0.2, 0.1), 2.0)
		draw_line(cc + Vector2(-0.9, 0.8) * scale_px, cc + Vector2(0.9, 0.8) * scale_px, Color(0.2, 0.2, 0.1), 2.0)


func _lamp(pos: Vector2, on: bool, col: Color, broken: bool) -> void:
	draw_circle(pos, 0.5 * scale_px, col if on else col.darkened(0.75))
	if broken:
		var o := Color(0.95, 0.5, 0.1)
		draw_circle(pos, 0.3 * scale_px, o)
		draw_line(pos + Vector2(-0.2, -0.2) * scale_px, pos + Vector2(0.2, 0.2) * scale_px, Color(0.3, 0.1, 0.0), 2.0)
		draw_line(pos + Vector2(-0.2, 0.2) * scale_px, pos + Vector2(0.2, -0.2) * scale_px, Color(0.3, 0.1, 0.0), 2.0)


func _head(cx: float, cy: float, horizontal: bool, group: int) -> void:
	var lit: Dictionary = msg.get("lit", {})
	var params: Dictionary = msg.get("params", {})
	var size := Vector2(4.4, 1.7) if horizontal else Vector2(1.7, 4.4)
	draw_rect(Rect2(_w(cx - size.x / 2.0, cy - size.y / 2.0), size * scale_px), HOUSING)
	var cols := [SIG_R, SIG_Y, SIG_G]
	var letters := ["R", "Y", "G"]
	for i in 3:
		var off := float(i - 1) * 1.4
		var pos := _w(cx + off, cy) if horizontal else _w(cx, cy + off)
		var nm := "TL%d%s" % [group, letters[i]]
		_lamp(pos, bool(lit.get(nm, false)), cols[i], bool(params.get("bk_" + nm, false)))


func _ped_head(cx: float, cy: float, group: int, end: int) -> void:
	var lit: Dictionary = msg.get("lit", {})
	var params: Dictionary = msg.get("params", {})
	var cmdd: Dictionary = msg.get("cmd", {})
	var btn: Dictionary = msg.get("btn", {})
	draw_rect(Rect2(_w(cx - 0.65, cy - 1.4), Vector2(1.3, 2.8) * scale_px), HOUSING)
	var rn := "PL%dR" % group
	var gn := "PL%dG" % group
	_lamp(_w(cx, cy - 0.65), bool(lit.get(rn, false)), SIG_R, bool(params.get("bk_" + rn, false)))
	_lamp(_w(cx, cy + 0.65), bool(lit.get(gn, false)), SIG_G, bool(params.get("bk_" + gn, false)))
	if end == 0:
		# request acknowledged (yellow disc) and push button (square) next to the first head of each crosswalk
		if bool(cmdd.get("PD%dA" % group, false)):
			draw_circle(_w(cx, cy - 2.0), 0.5 * scale_px, Color(1.0, 0.85, 0.1))
		var pressed := bool(btn.get(str(group), false))
		draw_rect(Rect2(_w(cx - 0.3, cy + 1.6), Vector2(0.6, 0.6) * scale_px), Color(1.0, 0.5, 0.1) if pressed else Color(0.45, 0.45, 0.45))


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

	_heading(box, "Traffic")
	_check(box, "Vehicles on", "traffic_on")
	_spin(box, "From west (dir 1) [veh/min]", 0.0, 120.0, 1.0, "rate_w")
	_spin(box, "From east (dir 1) [veh/min]", 0.0, 120.0, 1.0, "rate_e")
	_spin(box, "From north (dir 2) [veh/min]", 0.0, 120.0, 1.0, "rate_n")
	_spin(box, "From south (dir 2) [veh/min]", 0.0, 120.0, 1.0, "rate_s")
	_spin(box, "Desired speed [m/s]", 2.0, 30.0, 0.5, "v0")
	_spin(box, "Signal runners [% of vehicles]", 0.0, 100.0, 1.0, "runner_prob")
	_spin(box, "Pedestrians group 1 [per min]", 0.0, 120.0, 1.0, "ped_rate_1")
	_spin(box, "Pedestrians group 2 [per min]", 0.0, 120.0, 1.0, "ped_rate_2")
	_spin(box, "Pedestrian speed [m/s]", 0.3, 3.0, 0.1, "ped_speed")
	_spin(box, "Random seed", 1.0, 99999.0, 1.0, "seed", 1.0)
	_button(box, "Restart random traffic with this seed", func() -> void:
		reseed_seq += 1
		_send_ui())
	_heading(box, "Fault (collision) and scoreboard")
	_button(box, "Clear fault (remove crashed vehicles)", func() -> void:
		clear_fault_seq += 1
		_send_ui())
	_button(box, "Reset scoreboard", func() -> void:
		reset_stats_seq += 1
		_send_ui())
	_heading(box, "Broken lamps (feedback reads false, drivers see a dark lamp)")
	for n in LAMP_NAMES:
		_check(box, "Lamp I_X%s broken" % n, "bk_" + n)
	_heading(box, "Vehicle detectors")
	for n in DET_NAMES:
		_check(box, "I_%s dead (always false)" % n, "dead_" + n)
		_check(box, "I_%s stuck (always true)" % n, "stuck_" + n)
	_heading(box, "Pedestrian push buttons")
	_check(box, "I_XPD1 stuck pressed", "stuck_PD1")
	_check(box, "I_XPD2 stuck pressed", "stuck_PD2")


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
	var d := {"clear_fault_seq": clear_fault_seq, "reset_stats_seq": reset_stats_seq, "reseed_seq": reseed_seq}
	for k in controls:
		var c = controls[k]
		d[k] = c.value if c is SpinBox else c.button_pressed
	cmd.put_packet(JSON.stringify(d).to_utf8_buffer())


func _add_help(layer: CanvasLayer) -> void:
	var help := Label.new()
	help.text = "yellow disc = request acknowledged (I_YPDnA)   |   orange square = push button   |   orange loop = vehicle detector   |   orange cross = broken lamp"
	help.add_theme_font_size_override("font_size", 12)
	help.add_theme_color_override("font_color", Color(0.1, 0.12, 0.16))
	help.anchor_top = 1.0
	help.anchor_bottom = 1.0
	help.offset_left = 12
	help.offset_top = -24
	layer.add_child(help)


# -------------------------------------------------------------------- update
func _process(_delta: float) -> void:
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
	queue_redraw()
	var vs := get_viewport_rect().size
	fault_box.position = Vector2(vs.x - PANEL_W - 190.0, 10.0)

	var live := Time.get_ticks_msec() - last_rx_ms < 1000
	if msg.is_empty() or not live:
		hud.text = "Waiting for simulator on UDP port %d ..." % port
		panel.visible = false
		ui_ready = false
		fault_box.visible = false
		return

	send_timer += get_process_delta_time()
	if send_timer > 0.2:
		send_timer = 0.0
		_send_ui()
	fault_box.visible = bool(msg.get("fault", false))
	var s: Dictionary = msg.get("stats", {})
	var q: Dictionary = msg.get("queue", {})
	var cars: Array = msg.get("cars", [])
	hud.text = "t %.0f s    OPC UA clients: %d%s\nvehicles on the road %d, passed %d, average wait %.1f s, stopped at red/yellow: queue W %d  E %d  N %d  S %d (max %d)\npedestrians crossed %d, average wait %.1f s     red-light violations %d     crashes %d" % [
		float(msg["t"]), int(msg.get("clients", 0)), "" if bool(msg.get("plc", false)) else "   (no PLC - all lamps dark, drivers stop)",
		cars.size(), int(s.get("cars_passed", 0)), float(s.get("car_wait_avg", 0.0)),
		int(q.get("W", 0)), int(q.get("E", 0)), int(q.get("N", 0)), int(q.get("S", 0)), int(s.get("max_queue", 0)),
		int(s.get("peds_crossed", 0)), float(s.get("ped_wait_avg", 0.0)), int(s.get("violations", 0)), int(s.get("crashes", 0))]
