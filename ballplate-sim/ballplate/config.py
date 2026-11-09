from __future__ import annotations
import tomllib
from pathlib import Path

DEFAULT_PATH = Path(__file__).resolve().parent.parent / "config" / "sim.toml"


def load(path: str | Path | None = None) -> dict:
    with open(path or DEFAULT_PATH, "rb") as f:
        return tomllib.load(f)


def node_id(cfg: dict, name: str) -> str:
    o = cfg["opcua"]
    return o["node_template"].format(ns=o["namespace"], device=o["device"], name=name)
