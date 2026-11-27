"""The tag manifest is the single list of all tags: unique, and identical to what every simulation uses itself."""
from plchub.manifest import MANIFEST, all_tags


def test_unique_and_counts():
    tags = all_tags()
    assert len(tags) == 92
    assert {s: (len(v["plc_to_sim"]), len(v["sim_to_plc"])) for s, v in MANIFEST.items()} == {
        "ballplate": (2, 6), "valve": (2, 2), "transport": (18, 28), "level": (1, 4), "intersection": (12, 17)}


def test_matches_simulations():
    import ballplate.opcua_server as b
    import intersectionsim.opcua_server as i
    import levelsim.opcua_server as l
    import transportsim.io as t
    import valvesim.opcua_server as v
    m = MANIFEST
    assert set(m["ballplate"]["plc_to_sim"]) == set(b.PLC_TO_SIM) and set(m["ballplate"]["sim_to_plc"]) == set(b.SIM_TO_PLC)
    assert set(m["valve"]["plc_to_sim"]) == set(v.PLC_TO_SIM) and set(m["valve"]["sim_to_plc"]) == set(v.SIM_TO_PLC)
    assert set(m["level"]["plc_to_sim"]) == set(l.PLC_TO_SIM) and set(m["level"]["sim_to_plc"]) == set(l.SIM_TO_PLC)
    assert set(m["intersection"]["plc_to_sim"]) == set(i.PLC_TO_SIM) and set(m["intersection"]["sim_to_plc"]) == set(i.SIM_TO_PLC)
    assert set(m["transport"]["plc_to_sim"]) == set(t.PLC_BOOL + t.PLC_INT)
    assert set(m["transport"]["sim_to_plc"]) == set(t.SIM_BOOL + t.SIM_DWORD + t.SIM_INT)
    types = {"Boolean": set(t.PLC_BOOL + t.SIM_BOOL), "Int16": set(t.PLC_INT + t.SIM_INT), "UInt32": set(t.SIM_DWORD)}
    for d in ("plc_to_sim", "sim_to_plc"):
        for tag, (vt, _) in m["transport"][d].items():
            assert tag in types[vt]
    from asyncua import ua
    for d in ("plc_to_sim", "sim_to_plc"):
        src = getattr(l, d.upper())
        for tag, (vt, _) in m["level"][d].items():
            assert {"Int16": ua.VariantType.Int16, "Boolean": ua.VariantType.Boolean}[vt] == src[tag][1]
