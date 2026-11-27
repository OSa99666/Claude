"""Hub configuration: ``config/hub.toml`` section ``[hub]`` over built-in defaults, optionally overridden per call
(a simulation's own config may carry a ``[hub]`` table)."""
from __future__ import annotations

import os
import tomllib
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
DEFAULT_PATH = ROOT / "config" / "hub.toml"

DEFAULTS = {
    "port": 4840,
    "namespace_uri": "urn:plcdemo:simulation",
    "diag_namespace_uri": "urn:plcdemo:hub",
    "publishing_interval_cap_ms": 20,
    "stop_delay_s": 30.0,
    "heartbeat_timeout_s": 1.0,
    "log_file": "logs/hub.log",
}

SIM_MARK = "plchub-sim:"          # prefix of the OPC UA session name of every simulation (hub tells simulations and PLC apart)


def load(path: str | Path | None = None) -> dict:
    p = Path(path or os.environ.get("PLCHUB_CONFIG") or DEFAULT_PATH)
    cfg = dict(DEFAULTS)
    if p.exists():
        with open(p, "rb") as f:
            cfg.update(tomllib.load(f).get("hub", {}))
    return cfg


def merge(overrides: dict | None = None) -> dict:
    cfg = load()
    cfg.update(overrides or {})
    return cfg


def url(cfg: dict) -> str:
    return "opc.tcp://127.0.0.1:%d" % int(cfg["port"])


def plc_target(sim_cfg: dict) -> tuple[str, str]:
    """(url, namespace uri) a PLC or mock PLC uses for a simulation's config: the hub (default) or its standalone server."""
    o = sim_cfg["opcua"]
    if o.get("mode", "hub") == "hub":
        h = merge(sim_cfg.get("hub"))
        return url(h), h["namespace_uri"]
    return "opc.tcp://127.0.0.1:%d" % int(o["endpoint"].rsplit(":", 1)[1]), o["namespace_uri"]
