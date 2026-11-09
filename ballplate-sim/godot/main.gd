extends Node3D
## Visualisation only: all physics runs in the Python simulator, which streams
## the state as JSON over UDP (default port 9870).
##
## Simulator frame -> Godot frame:  (x, y, z) -> (x, z, -y)
## Tilt convention: tilt_x > 0 => surface slopes down towards +X (sim frame),
##                  tilt_y > 0 => surface slopes down towards +Y.
##
## Run:  godot --path godot -- --port 9870

var port := 9870
var udp := PacketPeerUDP.new()

var table := Node3D.new()
var ball := MeshInstance3D.new()
var cam := Camera3D.new()
var hud := Label.new()

var built_L := -1.0
var built_R := -1.0
var msg := {}
var last_rx_ms := -100000

# orbit camera
var yaw := 0.7
var pitch := 0.65
var dist := 0.95


func _ready() -> void:
	var args := OS.get_cmdline_user_args()
	for i in args.size():
		if args[i] == "--port" and i + 1 < args.size():
			port = int(args[i + 1])
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
	_update_camera()

	var layer := CanvasLayer.new()
	hud.position = Vector2(12, 8)
	hud.add_theme_font_size_override("font_size", 18)
	layer.add_child(hud)
	add_child(layer)

	_build(0.40, 0.02)


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


func _build(L: float, R: float) -> void:
	for c in table.get_children():
		c.queue_free()
	built_L = L
	built_R = R
	var t := 0.012                   # wall thickness
	var h := maxf(0.04, 3.0 * R)     # wall height
	table.add_child(_box(Vector3(L + 2 * t, 0.02, L + 2 * t), Vector3(0, -0.01, 0), Color(0.78, 0.74, 0.62)))
	var wc := Color(0.35, 0.37, 0.42)
	table.add_child(_box(Vector3(t, h, L + 2 * t), Vector3(L / 2 + t / 2, h / 2, 0), wc))
	table.add_child(_box(Vector3(t, h, L + 2 * t), Vector3(-L / 2 - t / 2, h / 2, 0), wc))
	table.add_child(_box(Vector3(L, h, t), Vector3(0, h / 2, L / 2 + t / 2), wc))
	table.add_child(_box(Vector3(L, h, t), Vector3(0, h / 2, -L / 2 - t / 2), wc))
	# centre mark
	table.add_child(_box(Vector3(0.02, 0.0015, 0.002), Vector3(0, 0.0008, 0), Color(0.2, 0.2, 0.2)))
	table.add_child(_box(Vector3(0.002, 0.0015, 0.02), Vector3(0, 0.0008, 0), Color(0.2, 0.2, 0.2)))

	var sm := SphereMesh.new()
	sm.radius = R
	sm.height = 2.0 * R
	ball.mesh = sm
	ball.material_override = _make_ball_material()
	if ball.get_parent() == null:
		table.add_child(ball)
	else:
		ball.reparent(table, false)
	# keep ball as last child (queue_free removed the others)
	ball.position = Vector3(0, R, 0)
	dist = 2.4 * L
	_update_camera()


func _process(_dt: float) -> void:
	while udp.get_available_packet_count() > 0:
		var parsed = JSON.parse_string(udp.get_packet().get_string_from_utf8())
		if parsed is Dictionary:
			msg = parsed
			last_rx_ms = Time.get_ticks_msec()

	var live := Time.get_ticks_msec() - last_rx_ms < 1000
	if msg.is_empty() or not live:
		hud.text = "Waiting for simulator on UDP port %d ..." % port
		return

	var L := float(msg.get("L", 0.4))
	var R := float(msg.get("R", 0.02))
	if absf(L - built_L) > 1e-6 or absf(R - built_R) > 1e-6:
		_build(L, R)

	# table orientation from the two slopes
	var tx := tan(float(msg["tx"]))
	var ty := tan(float(msg["ty"]))
	var xa := Vector3(1, -tx, 0).normalized()
	var za := Vector3(0, ty, 1).normalized()
	var ya := za.cross(xa).normalized()
	table.basis = Basis(xa, ya, za).orthonormalized()

	# ball (child of the table, table-local coordinates)
	ball.position = Vector3(float(msg["x"]), R, -float(msg["y"]))
	var q: Array = msg["q"]   # w, x, y, z in the simulator frame
	ball.quaternion = Quaternion(float(q[1]), float(q[3]), -float(q[2]), float(q[0])).normalized()

	hud.text = "ball  x %+6.1f mm   y %+6.1f mm   [%s]\ntilt  X %+5.1f deg   Y %+5.1f deg\nPLC link: %s   mu_s %.2f   e_wall %.2f" % [
		float(msg["x"]) * 1000.0, float(msg["y"]) * 1000.0,
		"SLIPPING" if msg.get("slip", false) else "rolling",
		rad_to_deg(float(msg["tx"])), rad_to_deg(float(msg["ty"])),
		"OK" if msg.get("plc", false) else "NO PLC (table levelled)",
		float(msg.get("mu_s", 0.0)), float(msg.get("e", 0.0))]


func _unhandled_input(ev: InputEvent) -> void:
	if ev is InputEventMouseMotion and (ev.button_mask & MOUSE_BUTTON_MASK_LEFT):
		yaw -= ev.relative.x * 0.005
		pitch = clampf(pitch + ev.relative.y * 0.005, 0.1, 1.5)
		_update_camera()
	elif ev is InputEventMouseButton and ev.pressed:
		if ev.button_index == MOUSE_BUTTON_WHEEL_UP:
			dist = maxf(0.3, dist * 0.92)
			_update_camera()
		elif ev.button_index == MOUSE_BUTTON_WHEEL_DOWN:
			dist = minf(3.0, dist * 1.08)
			_update_camera()


func _update_camera() -> void:
	var p := Vector3(sin(yaw) * cos(pitch), sin(pitch), cos(yaw) * cos(pitch)) * dist
	cam.look_at_from_position(p, Vector3.ZERO, Vector3.UP)
