extends Node3D
## Intersection simulation: 3D view + user interface.
##
## All logic runs in the Python simulator (intersectionsim).  This program
##  * receives the state as JSON over UDP (default port 9878) and draws it,
##  * sends the parameter/command values from the panel back to the simulator
##    (UDP, port 9879, to the address the state packets come from).
##
## Direction 1 = west-east road, direction 2 = north-south road (model x = east, model y = south = Godot z).
## Traffic lights show what is really lit (command and lamp not broken); a broken lamp gets an orange diamond.
## Yellow lamp at a pedestrian light post = request acknowledged by the PLC (I_YPDnA), orange box = push button pressed,
## orange loops on the road = vehicle detectors.  I_XF (hardware fault) = box at the top; a collision shows as red cars.
##
## Run:  godot --path godot -- [--port 9878] [--cmd-port 9879]

var port := 9878
var cmd_port := 9879
var udp := PacketPeerUDP.new()
var cmd := PacketPeerUDP.new()
var sim_host := ""

const HALF := 4.0
const LANE := 2.0
const CW0 := 4.4
const CW1 := 7.4
const STOP := -8.0
const DET0 := -11.0
const DET1 := -8.0
const CAR_LEN := 4.5
const CAR_W := 1.9
const ROAD_HALF := 62.0

const GRASS := Color(0.42, 0.55, 0.38)
const ROAD := Color(0.30, 0.30, 0.32)
const ZEBRA := Color(0.85, 0.85, 0.85)
const LINE := Color(0.85, 0.5, 0.1)
const CAR_COLS := [Color(0.35, 0.45, 0.8), Color(0.75, 0.75, 0.78), Color(0.3, 0.55, 0.35), Color(0.7, 0.55, 0.2), Color(0.45, 0.3, 0.55), Color(0.2, 0.25, 0.3)]
const CRASH_COL := Color(0.75, 0.15, 0.15)
const SIG_R := Color(0.9, 0.1, 0.1)
const SIG_Y := Color(1.0, 0.65, 0.0)
const SIG_G := Color(0.1, 0.85, 0.2)
const METAL := Color(0.6, 0.63, 0.68)
const GREENS := [Color(0.2, 0.5, 0.2), Color(0.28, 0.58, 0.25), Color(0.16, 0.42, 0.22)]
const PAVE := Color(0.72, 0.71, 0.68)
const PAVE_W := 2.5                # pavement width along the road edges
const PAVE_H := 0.09               # pavement top height (road top is at 0.03)
const KERB := Color(0.86, 0.86, 0.84)
const FLAT := 0.35                 # thickness of the lamp lenses relative to their diameter
const CORNER_R := 1.0              # radius of the rounded pavement / kerb corners
const KERB_W := 0.2
const KERB_H := 0.13               # kerb stone top height
const POLE_OFF := 4.6             # signal poles stand this far from the road centre line (the road edge is at 4.0)
const PED_POLE_S := 7.9            # pedestrian light poles: distance of the crosswalk side from the crossing centre
const TREE_OFFSET := 7.5            # distance of the tree rows from the road centre line

var hud := Label.new()
var fault_box := Label.new()
var panel := PanelContainer.new()
var cam := Camera3D.new()
var controls := {}
var clear_fault_seq := 0
var reset_stats_seq := 0
var reseed_seq := 0
var ui_ready := false
var send_timer := 0.0
var msg := {}
var last_rx_ms := -100000

const LAMP_NAMES := ["TL1R", "TL1Y", "TL1G", "TL2R", "TL2Y", "TL2G", "PL1R", "PL1G", "PL2R", "PL2G"]
const DET_NAMES := ["XTD11", "XTD12", "XTD21", "XTD22"]
var lamp_mats := {}                 # lamp name -> shared material (all heads of that lamp)
var bk_marks := {}                  # lamp name -> Array of orange diamonds
var det_nodes := {}                 # detector name -> MeshInstance3D
var req_lamps := {}                 # group -> Array of yellow request lamps
var btn_nodes := {}                 # group -> Array of push button boxes
var car_nodes := {}                 # id -> Node3D
var ped_nodes := {}                 # id -> Node3D (person)

var yaw := 0.6
var pitch := 0.75
var dist := 48.0
var target := Vector3(0, 0, 0)


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
	env.background_color = Color(0.62, 0.75, 0.88)
	env.ambient_light_source = Environment.AMBIENT_SOURCE_COLOR
	env.ambient_light_color = Color(0.65, 0.67, 0.72)
	var we := WorldEnvironment.new()
	we.environment = env
	add_child(we)
	var sun := DirectionalLight3D.new()
	sun.rotation_degrees = Vector3(-55, 35, 0)
	add_child(sun)

	_build_scene()
	add_child(cam)
	cam.current = true
	cam.far = 600.0
	_update_camera()
	_save_home()

	var layer := CanvasLayer.new()
	hud.position = Vector2(12, 8)
	hud.add_theme_font_size_override("font_size", 15)
	hud.add_theme_color_override("font_color", Color(0.05, 0.07, 0.1))
	layer.add_child(hud)
	fault_box.text = "Intersection fault (I_XF)"
	fault_box.add_theme_font_size_override("font_size", 18)
	fault_box.add_theme_color_override("font_color", Color(0.85, 0.0, 0.0))
	fault_box.visible = false
	fault_box.position = Vector2(12, 90)
	layer.add_child(fault_box)
	_add_help(layer)
	_build_panel(layer)
	add_child(layer)


# ------------------------------------------------------------------ 3D scene
func _mat(c: Color, alpha := 1.0, metal := 0.0) -> StandardMaterial3D:
	var m := StandardMaterial3D.new()
	m.albedo_color = Color(c.r, c.g, c.b, alpha)
	m.metallic = metal
	m.roughness = 0.6
	if alpha < 1.0:
		m.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	return m


func _box(size: Vector3, pos: Vector3, mat: Material, parent: Node = null) -> MeshInstance3D:
	var mi := MeshInstance3D.new()
	var m := BoxMesh.new()
	m.size = size
	mi.mesh = m
	mi.material_override = mat
	mi.position = pos
	(parent if parent != null else self).add_child(mi)
	return mi


func _cyl(r: float, h: float, pos: Vector3, mat: Material, parent: Node = null) -> MeshInstance3D:
	var mi := MeshInstance3D.new()
	var m := CylinderMesh.new()
	m.top_radius = r
	m.bottom_radius = r
	m.height = h
	m.radial_segments = 16
	mi.mesh = m
	mi.material_override = mat
	mi.position = pos
	(parent if parent != null else self).add_child(mi)
	return mi


func _sphere(r: float, pos: Vector3, mat: Material, parent: Node) -> MeshInstance3D:
	var mi := MeshInstance3D.new()
	var m := SphereMesh.new()
	m.radius = r
	m.height = 2.0 * r
	m.radial_segments = 16
	m.rings = 8
	mi.mesh = m
	mi.material_override = mat
	mi.position = pos
	parent.add_child(mi)
	return mi


func _label3d(text: String, pos: Vector3, size := 48) -> Label3D:
	var lab := Label3D.new()
	lab.text = text
	lab.font_size = size
	lab.pixel_size = 0.012
	lab.billboard = BaseMaterial3D.BILLBOARD_ENABLED
	lab.modulate = Color(0.1, 0.1, 0.15)
	lab.position = pos
	add_child(lab)
	return lab


## lamp material shared by every head showing this lamp: coloured and glowing when lit, dark otherwise
func _lamp_mat(lname: String, col: Color) -> StandardMaterial3D:
	if not lamp_mats.has(lname):
		var m := _mat(col.darkened(0.75))
		lamp_mats[lname] = m
		m.set_meta("col", col)
	return lamp_mats[lname]


func _set_lamp(lname: String, on: bool) -> void:
	var m: StandardMaterial3D = lamp_mats[lname]
	var col: Color = m.get_meta("col")
	m.albedo_color = col if on else col.darkened(0.75)
	m.emission_enabled = on
	m.emission = col
	m.emission_energy_multiplier = 1.6


func _add_broken_mark(lname: String, parent: Node, pos: Vector3) -> void:
	var d := _box(Vector3(0.34, 0.34, 0.06), pos, _mat(Color(0.95, 0.5, 0.1)), parent)
	d.rotation_degrees = Vector3(0, 0, 45)
	d.visible = false
	if not bk_marks.has(lname):
		bk_marks[lname] = []
	bk_marks[lname].append(d)


## Traffic light head on a pole at (x, z); `face` is the direction the lamps look (towards the approaching traffic)
func _traffic_head(x: float, z: float, face: Vector3, group: int) -> void:
	var steel := _mat(METAL, 1.0, 0.5)
	var root := Node3D.new()
	root.position = Vector3(x, 0, z)
	root.rotation.y = atan2(face.x, face.z)
	add_child(root)
	_cyl(0.09, 4.6, Vector3(0, 2.3, 0), steel, root)
	_box(Vector3(0.6, 1.9, 0.4), Vector3(0, 4.3, 0), _mat(Color(0.12, 0.12, 0.14)), root)
	var letters := ["R", "Y", "G"]
	var cols := [SIG_R, SIG_Y, SIG_G]
	for i in 3:
		var nm := "TL%d%s" % [group, letters[i]]
		var p := Vector3(0, 4.85 - 0.62 * float(i), 0.22)
		var bulb := _sphere(0.25, p, _lamp_mat(nm, cols[i]), root)
		bulb.scale = Vector3(1.0, 1.0, FLAT)                        # flat lens instead of a ball
		_add_broken_mark(nm, root, p + Vector3(0, 0, 0.12))


func _ped_head(x: float, z: float, face: Vector3, group: int) -> void:
	var steel := _mat(METAL, 1.0, 0.5)
	var root := Node3D.new()
	root.position = Vector3(x, 0, z)
	root.rotation.y = atan2(face.x, face.z)
	add_child(root)
	_cyl(0.07, 3.0, Vector3(0, 1.5, 0), steel, root)
	_box(Vector3(0.5, 1.2, 0.3), Vector3(0, 2.7, 0), _mat(Color(0.12, 0.12, 0.14)), root)
	var rn := "PL%dR" % group
	var gn := "PL%dG" % group
	var bulb_r := _sphere(0.19, Vector3(0, 3.0, 0.16), _lamp_mat(rn, SIG_R), root)
	var bulb_g := _sphere(0.19, Vector3(0, 2.4, 0.16), _lamp_mat(gn, SIG_G), root)
	bulb_r.scale = Vector3(1.0, 1.0, FLAT)
	bulb_g.scale = Vector3(1.0, 1.0, FLAT)
	_add_broken_mark(rn, root, Vector3(0, 3.0, 0.27))
	_add_broken_mark(gn, root, Vector3(0, 2.4, 0.27))
	# request acknowledged: yellow lamp in a box on top of the pole; push button on the pole (identical on every pedestrian pole)
	var rl := _box(Vector3(0.34, 0.26, 0.34), Vector3(0, 3.46, 0), _mat(Color(1.0, 0.85, 0.1).darkened(0.7)), root)
	if not req_lamps.has(group):
		req_lamps[group] = []
	req_lamps[group].append(rl)
	var bb := _box(Vector3(0.18, 0.18, 0.1), Vector3(0, 1.3, -0.08), _mat(Color(0.45, 0.45, 0.45)), root)
	if not btn_nodes.has(group):
		btn_nodes[group] = []
	btn_nodes[group].append(bb)


## Triangle with an explicit normal; the winding is chosen so that it is a front face (Godot: clockwise seen from outside)
func _tri(st: SurfaceTool, a: Vector3, b: Vector3, c: Vector3, n: Vector3) -> void:
	if (b - a).cross(c - a).dot(n) > 0.0:
		var t := b
		b = c
		c = t
	for v in [a, b, c]:
		st.set_normal(n)
		st.add_vertex(v)


## Rounded pavement corner at the crossing corner (sx * 4, sz * 4): a road-coloured fillet cuts the pavement corner, a kerb
## stone follows the arc (centre (sx * (4 + R), sz * (4 + R)), radius R; the stone is on the pavement side).
func _kerb_corner(sx: float, sz: float) -> void:
	var r := CORNER_R
	var cx := sx * (HALF + r)
	var cz := sz * (HALF + r)
	var a0 := 0.0 if sx < 0.0 else PI
	var da := PI / 2.0 if sx * sz > 0.0 else -PI / 2.0
	var steps := 10
	var fill := SurfaceTool.new()
	fill.begin(Mesh.PRIMITIVE_TRIANGLES)
	var ring := SurfaceTool.new()
	ring.begin(Mesh.PRIMITIVE_TRIANGLES)
	var yf := PAVE_H + 0.004
	var corner := Vector3(sx * HALF, yf, sz * HALF)
	for k in steps:
		var f0 := a0 + da * float(k) / float(steps)
		var f1 := a0 + da * float(k + 1) / float(steps)
		var d0 := Vector3(cos(f0), 0, sin(f0))
		var d1 := Vector3(cos(f1), 0, sin(f1))
		var c := Vector3(cx, 0, cz)
		_tri(fill, corner, c + d0 * r + Vector3(0, yf, 0), c + d1 * r + Vector3(0, yf, 0), Vector3.UP)
		var ro := r
		var ri := r - KERB_W
		var o0 := c + d0 * ro
		var o1 := c + d1 * ro
		var i0 := c + d0 * ri
		var i1 := c + d1 * ri
		var up := Vector3(0, KERB_H, 0)
		_tri(ring, o0 + up, o1 + up, i1 + up, Vector3.UP)
		_tri(ring, o0 + up, i1 + up, i0 + up, Vector3.UP)
		var dm := (d0 + d1).normalized()
		_tri(ring, o0 + up, o1 + up, o1, dm)                    # wall towards the road
		_tri(ring, o0 + up, o1, o0, dm)
		_tri(ring, i0 + up, i1 + up, i1, -dm)                   # wall towards the pavement
		_tri(ring, i0 + up, i1, i0, -dm)
	_blob(fill.commit(), Vector3.ZERO, _mat(ROAD), self)
	_blob(ring.commit(), Vector3.ZERO, _mat(KERB), self)


## Tree: trunk and a rounded crown made of two spheres; size and green vary with k
func _tree(x: float, z: float, k: int) -> void:
	var root := Node3D.new()
	root.position = Vector3(x, 0, z)
	var sc := 0.85 + 0.1 * float((k * 7) % 5)
	root.scale = Vector3(sc, sc, sc)
	add_child(root)
	_cyl(0.18, 2.4, Vector3(0, 1.2, 0), _mat(Color(0.35, 0.24, 0.14)), root)
	var leaf := _mat(GREENS[k % GREENS.size()])
	var crown := _sphere(1.5, Vector3(0, 3.6, 0), leaf, root)
	crown.scale = Vector3(1.0, 1.15, 1.0)
	_sphere(1.0, Vector3(0.7, 4.4, 0.3), leaf, root)


func _build_scene() -> void:
	_box(Vector3(2 * ROAD_HALF, 0.2, 2 * ROAD_HALF), Vector3(0, -0.12, 0), _mat(GRASS), null)    # green square only as large as the road area
	var road := _mat(ROAD)
	_box(Vector3(2 * ROAD_HALF, 0.06, 2 * HALF), Vector3(0, 0.0, 0), road, null)
	_box(Vector3(2 * HALF, 0.06, 2 * ROAD_HALF), Vector3(0, 0.0, 0), road, null)
	var line := _mat(LINE)
	var x := CW1 + 1.5
	while x < ROAD_HALF - 2.0:
		for sgn in [-1.0, 1.0]:
			_box(Vector3(2.0, 0.02, 0.16), Vector3(float(sgn) * (x + 1.0), 0.05, 0), line, null)
			_box(Vector3(0.16, 0.02, 2.0), Vector3(0, 0.05, float(sgn) * (x + 1.0)), line, null)
		x += 4.0
	var zebra := _mat(ZEBRA)
	for i in 7:
		var a := -HALF + 0.6 + float(i) * 1.15
		_box(Vector3(0.6, 0.02, CW1 - CW0), Vector3(a, 0.05, -(CW0 + CW1) / 2.0), zebra, null)
		_box(Vector3(0.6, 0.02, CW1 - CW0), Vector3(a, 0.05, (CW0 + CW1) / 2.0), zebra, null)
		_box(Vector3(CW1 - CW0, 0.02, 0.6), Vector3(-(CW0 + CW1) / 2.0, 0.05, a), zebra, null)
		_box(Vector3(CW1 - CW0, 0.02, 0.6), Vector3((CW0 + CW1) / 2.0, 0.05, a), zebra, null)
	# stop lines
	_box(Vector3(0.3, 0.02, HALF), Vector3(STOP + 0.15, 0.05, HALF / 2.0), zebra, null)
	_box(Vector3(0.3, 0.02, HALF), Vector3(-STOP - 0.15, 0.05, -HALF / 2.0), zebra, null)
	_box(Vector3(HALF, 0.02, 0.3), Vector3(-HALF / 2.0, 0.05, STOP + 0.15), zebra, null)
	_box(Vector3(HALF, 0.02, 0.3), Vector3(HALF / 2.0, 0.05, -STOP - 0.15), zebra, null)
	# pavements along both edges of both roads (the crossing corners are covered by the strips along the direction-1 road)
	var pave := _mat(PAVE)
	var plen := ROAD_HALF - HALF
	var plen2 := ROAD_HALF - HALF - PAVE_W
	for sx in [-1.0, 1.0]:
		for sz in [-1.0, 1.0]:
			_box(Vector3(plen, PAVE_H, PAVE_W), Vector3(float(sx) * (HALF + plen / 2.0), PAVE_H / 2.0, float(sz) * (HALF + PAVE_W / 2.0)), pave, null)
			_box(Vector3(PAVE_W, PAVE_H, plen2), Vector3(float(sx) * (HALF + PAVE_W / 2.0), PAVE_H / 2.0, float(sz) * (HALF + PAVE_W + plen2 / 2.0)), pave, null)
	# kerb line along the road edge of every pavement strip; open at the crosswalks (pedestrians step off there)
	var kerb := _mat(KERB)
	var k0 := CW1 + 0.2
	var klen := ROAD_HALF - k0
	for sx in [-1.0, 1.0]:
		for sz in [-1.0, 1.0]:
			_box(Vector3(klen, KERB_H, KERB_W), Vector3(float(sx) * (k0 + klen / 2.0), KERB_H / 2.0, float(sz) * (HALF + KERB_W / 2.0)), kerb, null)
			_box(Vector3(KERB_W, KERB_H, klen), Vector3(float(sx) * (HALF + KERB_W / 2.0), KERB_H / 2.0, float(sz) * (k0 + klen / 2.0)), kerb, null)
	for sx in [-1.0, 1.0]:
		for sz in [-1.0, 1.0]:
			_kerb_corner(float(sx), float(sz))
	# vehicle detector loops (stop-line side of each arm)
	var loops := {"XTD11": Vector3(-9.5, 0.06, 2.0), "XTD12": Vector3(9.5, 0.06, -2.0),
		"XTD21": Vector3(-2.0, 0.06, -9.5), "XTD22": Vector3(2.0, 0.06, 9.5)}
	for k in loops:
		var d := _box(Vector3(3.0, 0.02, 3.0), loops[k], _mat(Color(0.85, 0.5, 0.2).darkened(0.3)), null)
		det_nodes[k] = d
	# signal heads: direction 1 (west / east arm) and direction 2 (north / south arm)
	_traffic_head(-10.5, POLE_OFF, Vector3(-1, 0, 0), 1)
	_traffic_head(10.5, -POLE_OFF, Vector3(1, 0, 0), 1)
	_traffic_head(-POLE_OFF, -10.5, Vector3(0, 0, -1), 2)
	_traffic_head(POLE_OFF, 10.5, Vector3(0, 0, 1), 2)
	# pedestrian lights at the ends of every crosswalk (facing the crosswalk)
	# (beside the crosswalk on its outer side, at the kerb, facing across the road)
	_ped_head(-POLE_OFF, -PED_POLE_S, Vector3(1, 0, 0), 1)
	_ped_head(POLE_OFF, -PED_POLE_S, Vector3(-1, 0, 0), 1)
	_ped_head(-POLE_OFF, PED_POLE_S, Vector3(1, 0, 0), 1)
	_ped_head(POLE_OFF, PED_POLE_S, Vector3(-1, 0, 0), 1)
	_ped_head(-PED_POLE_S, -POLE_OFF, Vector3(0, 0, 1), 2)
	_ped_head(-PED_POLE_S, POLE_OFF, Vector3(0, 0, -1), 2)
	_ped_head(PED_POLE_S, -POLE_OFF, Vector3(0, 0, 1), 2)
	_ped_head(PED_POLE_S, POLE_OFF, Vector3(0, 0, -1), 2)
	_label3d("Direction 1", Vector3(-ROAD_HALF * 0.55, 1.2, -1.5))
	_label3d("Direction 2", Vector3(2.5, 1.2, ROAD_HALF * 0.55))
	# avenues: a row of trees on either side of both roads, beyond the signal poles
	var ts := 16.0
	var tk := 0
	while ts < ROAD_HALF - 3.0:
		for sgn in [-1.0, 1.0]:
			for side in [-TREE_OFFSET, TREE_OFFSET]:
				var shift := 4.0 if side > 0.0 else 0.0
				_tree(float(sgn) * (ts + shift), float(side), tk)         # along the direction-1 road
				_tree(float(side), float(sgn) * (ts + shift), tk + 3)     # along the direction-2 road
				tk += 1
		ts += 8.0
	for n in LAMP_NAMES:
		_set_lamp(n, false)


func _car_center(arm: String, s: float) -> Vector3:
	var m := s - CAR_LEN / 2.0
	match arm:
		"W":
			return Vector3(m, 0, LANE)
		"E":
			return Vector3(-m, 0, -LANE)
		"N":
			return Vector3(-LANE, 0, m)
	return Vector3(LANE, 0, -m)


func _car_yaw(arm: String) -> float:
	match arm:
		"W":
			return 0.0
		"E":
			return PI
		"N":
			return -PI / 2.0
	return PI / 2.0


func _ped_pos(cw: String, u: float, d: int) -> Vector3:
	var f := u if d > 0 else 1.0 - u
	var a := -HALF + 2.0 * HALF * f
	var mid := (CW0 + CW1) / 2.0
	match cw:
		"N":
			return Vector3(a, 0, -mid)
		"S":
			return Vector3(a, 0, mid)
		"W":
			return Vector3(-mid, 0, a)
	return Vector3(mid, 0, a)


func _spow(v: float, e: float) -> float:
	return signf(v) * pow(absf(v), e)


## Rounded box: superellipsoid with half sizes a (x), b (z), c (y); exponents < 1 give a box with rounded edges, 1 an ellipsoid.
func _superellipsoid(a: float, b: float, c: float, e1: float, e2: float, n_lat := 14, n_lon := 32) -> ArrayMesh:
	var st := SurfaceTool.new()
	st.begin(Mesh.PRIMITIVE_TRIANGLES)
	var pts: Array = []
	var nrm: Array = []
	for i in n_lat + 1:
		var th := -PI / 2.0 + PI * float(i) / float(n_lat)
		for j in n_lon + 1:
			var ph := -PI + TAU * float(j) / float(n_lon)
			pts.append(Vector3(a * _spow(cos(th), e1) * _spow(cos(ph), e2), c * _spow(sin(th), e1), b * _spow(cos(th), e1) * _spow(sin(ph), e2)))
			var n := Vector3(_spow(cos(th), 2.0 - e1) * _spow(cos(ph), 2.0 - e2) / a, _spow(sin(th), 2.0 - e1) / c, _spow(cos(th), 2.0 - e1) * _spow(sin(ph), 2.0 - e2) / b)
			nrm.append(n.normalized() if n.length() > 0.00001 else Vector3.UP)
	var w := n_lon + 1
	for i in n_lat:
		for j in n_lon:
			var i00 := i * w + j
			var i01 := i * w + j + 1
			var i10 := (i + 1) * w + j
			var i11 := (i + 1) * w + j + 1
			for tri in [[i00, i01, i11], [i00, i11, i10]]:
				var t0: int = tri[0]
				var t1: int = tri[1]
				var t2: int = tri[2]
				var v0: Vector3 = pts[t0]
				var v1: Vector3 = pts[t1]
				var v2: Vector3 = pts[t2]
				if (v1 - v0).cross(v2 - v0).dot(nrm[t0]) > 0.0:       # Godot: clockwise seen from outside = front face
					var tmp := t1
					t1 = t2
					t2 = tmp
				for idx in [t0, t1, t2]:
					st.set_normal(nrm[idx])
					st.add_vertex(pts[idx])
	return st.commit()


func _blob(mesh: Mesh, pos: Vector3, mat: Material, parent: Node3D) -> MeshInstance3D:
	var mi := MeshInstance3D.new()
	mi.mesh = mesh
	mi.material_override = mat
	mi.position = pos
	parent.add_child(mi)
	return mi


var car_body_mesh: ArrayMesh
var car_cabin_mesh: ArrayMesh
var car_glass_mesh: ArrayMesh


func _make_car(cid: int) -> Node3D:
	if car_body_mesh == null:
		car_body_mesh = _superellipsoid(CAR_LEN / 2.0, CAR_W / 2.0, 0.38, 0.35, 0.3)
		car_cabin_mesh = _superellipsoid(1.15, 0.82, 0.34, 0.5, 0.5)
		car_glass_mesh = _superellipsoid(1.2, 0.86, 0.16, 0.5, 0.5)
	var root := Node3D.new()
	var mat := _mat(CAR_COLS[cid % CAR_COLS.size()], 1.0, 0.3)
	root.set_meta("mat", mat)
	root.set_meta("col", CAR_COLS[cid % CAR_COLS.size()])
	_blob(car_body_mesh, Vector3(0, 0.62, 0), mat, root)
	_blob(car_cabin_mesh, Vector3(-0.3, 1.18, 0), mat, root)
	_blob(car_glass_mesh, Vector3(-0.28, 1.2, 0), _mat(Color(0.12, 0.17, 0.25), 1.0, 0.2), root)        # window band
	var wheel := _mat(Color(0.08, 0.08, 0.08))
	for wx in [-1.4, 1.4]:
		for wz in [-0.88, 0.88]:
			var w := _cyl(0.32, 0.25, Vector3(float(wx), 0.32, float(wz)), wheel, root)
			w.rotation_degrees = Vector3(90, 0, 0)
	var head := _mat(Color(1.0, 1.0, 0.8))
	var tail := _mat(Color(0.7, 0.05, 0.05))
	for hz in [-0.62, 0.62]:
		var hl := _sphere(0.12, Vector3(CAR_LEN / 2.0 - 0.05, 0.66, float(hz)), head, root)
		var tl := _sphere(0.1, Vector3(-CAR_LEN / 2.0 + 0.05, 0.66, float(hz)), tail, root)
		hl.scale = Vector3(FLAT, 1.0, 1.3)                         # lamps lie flat on the car front / rear
		tl.scale = Vector3(FLAT, 1.0, 1.3)
	add_child(root)
	return root


const SHIRTS := [Color(0.75, 0.2, 0.2), Color(0.2, 0.45, 0.75), Color(0.9, 0.75, 0.2), Color(0.3, 0.65, 0.35), Color(0.85, 0.85, 0.85), Color(0.6, 0.3, 0.6)]
const TROUSERS := [Color(0.15, 0.17, 0.3), Color(0.25, 0.22, 0.2), Color(0.35, 0.4, 0.5), Color(0.1, 0.1, 0.1)]
const SKINS := [Color(0.95, 0.78, 0.65), Color(0.8, 0.6, 0.45), Color(0.55, 0.38, 0.28), Color(0.38, 0.26, 0.2)]
const HAIRS := [Color(0.1, 0.08, 0.06), Color(0.35, 0.22, 0.1), Color(0.75, 0.6, 0.3), Color(0.5, 0.5, 0.5)]


func _caps(r: float, h: float, pos: Vector3, mat: Material, parent: Node3D) -> MeshInstance3D:
	var m := CapsuleMesh.new()
	m.radius = r
	m.height = maxf(h, 2.0 * r)
	m.radial_segments = 16
	m.rings = 6
	return _blob(m, pos, mat, parent)


## limb that swings about its top: a pivot at `pos` with a capsule of length `len` hanging down from it
func _limb(parent: Node3D, pos: Vector3, r: float, len: float, mat: Material) -> Node3D:
	var pivot := Node3D.new()
	pivot.position = pos
	parent.add_child(pivot)
	_caps(r, len, Vector3(0, -len / 2.0, 0), mat, pivot)
	return pivot


## A person (about 1.8 m) from rounded parts: head with hair, torso, arms with hands, legs with shoes; the limbs swing
## while walking.  Local +x = walking direction.
func _make_ped(pid: int) -> Node3D:
	var root := Node3D.new()
	var shirt_col: Color = SHIRTS[pid % SHIRTS.size()]
	var shirt := _mat(shirt_col)
	var trousers := _mat(TROUSERS[(pid * 7) % TROUSERS.size()])
	var skin := _mat(SKINS[(pid * 5) % SKINS.size()])
	var hair := _mat(HAIRS[(pid * 3) % HAIRS.size()])
	var shoe := _mat(Color(0.1, 0.08, 0.07))
	root.set_meta("shirt", shirt)
	root.set_meta("shirt_col", shirt_col)
	var torso := _caps(0.2, 0.66, Vector3(0, 1.2, 0), shirt, root)
	torso.scale = Vector3(0.62, 1.0, 1.0)
	_cyl(0.05, 0.1, Vector3(0, 1.55, 0), skin, root)                              # neck
	_sphere(0.12, Vector3(0, 1.69, 0), skin, root)                                # head
	var cap := _sphere(0.128, Vector3(-0.02, 1.72, 0), hair, root)                # hair
	cap.scale = Vector3(1.0, 0.75, 1.0)
	var leg_l := _limb(root, Vector3(0, 0.93, -0.1), 0.075, 0.93, trousers)
	var leg_r := _limb(root, Vector3(0, 0.93, 0.1), 0.075, 0.93, trousers)
	for leg in [leg_l, leg_r]:
		var sh := _sphere(0.09, Vector3(0.05, -0.88, 0), shoe, leg)
		sh.scale = Vector3(1.5, 0.6, 0.9)
	var arm_l := _limb(root, Vector3(0, 1.46, -0.22), 0.055, 0.62, shirt)
	var arm_r := _limb(root, Vector3(0, 1.46, 0.22), 0.055, 0.62, shirt)
	for arm in [arm_l, arm_r]:
		_sphere(0.06, Vector3(0, -0.64, 0), skin, arm)                            # hand
	root.set_meta("legL", leg_l)
	root.set_meta("legR", leg_r)
	root.set_meta("armL", arm_l)
	root.set_meta("armR", arm_r)
	add_child(root)
	return root


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
	_heading(box, "Hardware fault I_XF")
	_check(box, "Force I_XF (any broken lamp sets it too)", "hw_fault")
	_heading(box, "Collisions and scoreboard")
	_button(box, "Clear collision (remove crashed vehicles)", func() -> void:
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
	help.text = "mouse: drag orbit, middle-drag pan, wheel zoom   |   keys: arrows pan, Shift+arrows orbit, +/- zoom, Home reset   |   orange loop = detector, orange diamond = broken lamp, yellow lamp on a pole = request acknowledged"
	help.add_theme_font_size_override("font_size", 12)
	help.add_theme_color_override("font_color", Color(0.1, 0.12, 0.16))
	help.anchor_top = 1.0
	help.anchor_bottom = 1.0
	help.offset_left = 12
	help.offset_top = -24
	layer.add_child(help)


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
		fault_box.visible = false
		return

	send_timer += delta
	if send_timer > 0.2:
		send_timer = 0.0
		_send_ui()

	var lit: Dictionary = msg.get("lit", {})
	var params: Dictionary = msg.get("params", {})
	var cmdd: Dictionary = msg.get("cmd", {})
	var btn: Dictionary = msg.get("btn", {})
	var det: Dictionary = msg.get("det", {})
	for n in LAMP_NAMES:
		_set_lamp(n, bool(lit.get(n, false)))
		for d in bk_marks[n]:
			(d as MeshInstance3D).visible = bool(params.get("bk_" + n, false))
	for k in det_nodes:
		var dn: MeshInstance3D = det_nodes[k]
		var on := bool(det.get(k, false))
		var dm := dn.material_override as StandardMaterial3D
		dm.albedo_color = Color(1.0, 0.55, 0.1) if on else Color(0.85, 0.5, 0.2).darkened(0.3)
		dm.emission_enabled = on
		dm.emission = Color(1.0, 0.55, 0.1)
	for g in req_lamps:
		var on2 := bool(cmdd.get("PD%dA" % g, false))
		for rl in req_lamps[g]:
			var rm := (rl as MeshInstance3D).material_override as StandardMaterial3D
			rm.albedo_color = Color(1.0, 0.85, 0.1) if on2 else Color(1.0, 0.85, 0.1).darkened(0.7)
			rm.emission_enabled = on2
			rm.emission = Color(1.0, 0.85, 0.1)
	for g in btn_nodes:
		var pressed := bool(btn.get(str(g), false))
		for bb in btn_nodes[g]:
			((bb as MeshInstance3D).material_override as StandardMaterial3D).albedo_color = Color(1.0, 0.5, 0.1) if pressed else Color(0.45, 0.45, 0.45)

	# cars: create / move / remove by id
	var seen := {}
	var cars: Array = msg.get("cars", [])
	for c in cars:
		var cid := int(c[3])
		seen[cid] = true
		if not car_nodes.has(cid):
			car_nodes[cid] = _make_car(cid)
		var node: Node3D = car_nodes[cid]
		var arm: String = c[0]
		node.position = _car_center(arm, float(c[1]))
		node.rotation.y = _car_yaw(arm)
		var cm: StandardMaterial3D = node.get_meta("mat")
		cm.albedo_color = CRASH_COL if bool(c[2]) else (node.get_meta("col") as Color)
	for cid in car_nodes.keys():
		if not seen.has(cid):
			(car_nodes[cid] as Node3D).queue_free()
			car_nodes.erase(cid)
	# pedestrians
	var pseen := {}
	var peds: Array = msg.get("peds", [])
	for q in peds:
		var pid := int(q[4])
		pseen[pid] = true
		if not ped_nodes.has(pid):
			ped_nodes[pid] = _make_ped(pid)
		var pn: Node3D = ped_nodes[pid]
		var cw: String = q[0]
		var st: String = q[3]
		var dd := int(q[2])
		var pp := _ped_pos(cw, float(q[1]), dd)
		if st == "wait":
			var axis := Vector3(1, 0, 0) if (cw == "N" or cw == "S") else Vector3(0, 0, 1)
			pp = _ped_pos(cw, 0.0, dd) - axis * float(dd) * 0.9       # waits at the kerb before the crosswalk
		# face the walking direction (also while waiting), swing legs and arms by the distance walked
		var pyaw := 0.0
		if cw == "N" or cw == "S":
			pyaw = 0.0 if dd > 0 else PI
		else:
			pyaw = -PI / 2.0 if dd > 0 else PI / 2.0
		var swing := 0.0
		if st == "cross":
			swing = sin(float(q[1]) * 2.0 * HALF / 1.3 * PI) * 0.6      # one half step per 1.3 m
		(pn.get_meta("legL") as Node3D).rotation.z = swing
		(pn.get_meta("legR") as Node3D).rotation.z = -swing
		(pn.get_meta("armL") as Node3D).rotation.z = -swing * 0.8
		(pn.get_meta("armR") as Node3D).rotation.z = swing * 0.8
		var shirt_m: StandardMaterial3D = pn.get_meta("shirt")
		var base_y := PAVE_H if st == "wait" else 0.03        # waiting on the pavement, crossing on the road
		if st == "hit":
			pn.position = pp + Vector3(0, base_y + 0.2, 0)
			pn.rotation = Vector3(0, pyaw, -PI / 2.0)                    # lies on the road
			shirt_m.albedo_color = Color(0.8, 0.1, 0.1)
		else:
			pn.position = pp + Vector3(0, base_y, 0)
			pn.rotation = Vector3(0, pyaw, 0)
			shirt_m.albedo_color = pn.get_meta("shirt_col") as Color
	for pid in ped_nodes.keys():
		if not pseen.has(pid):
			(ped_nodes[pid] as Node3D).queue_free()
			ped_nodes.erase(pid)

	fault_box.visible = bool(msg.get("hw_fault", false))
	var s: Dictionary = msg.get("stats", {})
	var q2: Dictionary = msg.get("queue", {})
	hud.text = "t %.0f s    OPC UA clients: %d%s\nvehicles on the road %d, passed %d, average wait %.1f s, stopped at red/yellow: queue W %d  E %d  N %d  S %d (max %d)\npedestrians crossed %d, average wait %.1f s     red-light violations %d     crashes %d" % [
		float(msg["t"]), int(msg.get("clients", 0)), "" if bool(msg.get("plc", false)) else "   (no PLC - all lamps dark, drivers stop)",
		cars.size(), int(s.get("cars_passed", 0)), float(s.get("car_wait_avg", 0.0)),
		int(q2.get("W", 0)), int(q2.get("E", 0)), int(q2.get("N", 0)), int(q2.get("S", 0)), int(s.get("max_queue", 0)),
		int(s.get("peds_crossed", 0)), float(s.get("ped_wait_avg", 0.0)), int(s.get("violations", 0)), int(s.get("crashes", 0))]
	if bool(msg.get("crash", false)):
		hud.text += "\nCOLLISION - clear it in the panel"


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
const CAM_DIST_MIN := 3.0
const CAM_DIST_MAX := 140.0
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


