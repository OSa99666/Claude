# Project context: intersection simulation (standalone, fifth simulation)

Same pattern as the other sims (`../level-sim` is the closest: Bool/Int IO, mock PLC, panel).  `subscription_tweaks.py`, `ui_link.py`,
`config.py` are COPIES (keep in sync).  Naming: OPC UA folder `Intersection`, server `Intersection simulator`, port 4844, UDP 9878/9879.

- User request: re-create the existing CODESYS intersection simulation (picture: 2 crossing roads, vehicles, pedestrian crossings, orange
  crosses = broken lamps, smiley + 'Intersection fault' = crash).  IO tags exactly as supplied (29 Bool, `I_Y*` PLC->sim, `I_X*` sim->PLC).
  The supplied %Q/%I addresses overlap; the sim does not depend on them (tag names); corrected proposal in the README.
- Model `intersectionsim/model.py` (pure Python, no OPC UA): IDM car following, stop line / yellow decision / pedestrian yield,
  runners, pedestrians with push button + request lamp, collisions -> latched `crash` (display only), scoreboard, faults (broken lamps, detectors, buttons).
  Dark head = red.  Mock PLC `FixedTime` (also used in tests) = two-phase fixed-time controller with ped requests.
- CONFIRMED by the user: pedestrian group 1 walks along direction 1 and crosses the direction-2 road (PL1 green with TL1 green);
  detectors XTD11 = W arm, XTD12 = E, XTD21 = N, XTD22 = S; detector/button faults do not set I_XF.
- CONFIRMED by the user: `I_XF` is just a binary input for any error in the traffic light HARDWARE -> model: true while any lamp is broken or the UI forces it (`hw_fault`); detector/button faults do not set it (ask if they should).  Collisions are not an IO.
- Godot (`godot/main.gd`): 3D scene (road, zebra crossings, signal poles with lamp emission, cars with rounded generated bodies (superellipsoid meshes: body, cabin, window band, wheels, lights) created per id, pedestrians as rounded people (capsule torso/limbs, head with hair, hands, shoes; swinging arms and legs, varied clothes and skin tones, lie down when hit), detector loops, tree avenues and pavement strips with kerb stones (open at the crosswalks, rounded 1 m corners at the crossing) along both roads, flat lamp lenses, pedestrian crossing signs (one per crosswalk) and yield signs on direction 1, signal poles 0.6 m from the road edge, pedestrian light poles beside the crosswalks), orbit camera block as in the other sims but with CAM_DIST_MAX 140 for the big scene, scrolling panel.  (Was a 2D top view first; replaced on request.)  Written without running Godot; the user has checked the view and confirmed it (later visual changes are checked by the user as well).
- Tests: `python -m pytest -q` in `intersection-sim` (15 pass).  Never put passwords/tokens in the repo.
- Rear-end collisions (same lane) are detected in `_check_collisions`: follower is clamped to the rear of the leader, both `crashed` (red in Godot), counts as a crash (latched like any collision, shown by red cars only).
