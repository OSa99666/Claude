#!/usr/bin/env bash
# Start the whole rolling-table demo: simulator (OPC UA server) + mock PLC + Godot.
#
#   ./run.sh                      simulator + mock PLC (centre) + Godot
#   ./run.sh --pattern circle     mock PLC drives the ball around a circle (center|circle|square)
#   ./run.sh --no-mock            real PLC: only simulator + Godot (PLC connects to opc.tcp://<this-pc>:4840)
#   ./run.sh --no-godot           headless (e.g. over SSH): simulator + mock PLC only
#   ./run.sh --godot /path/to/Godot_v4.3-stable_linux.x86_64
#   ./run.sh --publish-cap-ms 20  serve OPC UA clients at least every 20 ms even if they ask for 1000 ms
#
# Environment: VENV=<dir> use another virtualenv (default ./.venv, created if missing)
#              GODOT=<cmd> Godot executable
# Closing the Godot window or pressing Ctrl+C stops everything.  Logs: ./logs/

set -u
cd "$(dirname "$(readlink -f "$0")")"

PATTERN=center; MOCK=1; USE_GODOT=1; GODOT_BIN="${GODOT:-}"; SIM_ARGS=()
while [ $# -gt 0 ]; do
    case "$1" in
        --pattern)  PATTERN="${2:?--pattern needs center|circle|square}"; shift 2 ;;
        --no-mock)  MOCK=0; shift ;;
        --no-godot) USE_GODOT=0; shift ;;
        --godot)    GODOT_BIN="${2:?--godot needs a path}"; shift 2 ;;
        --publish-cap-ms) SIM_ARGS+=(--publish-cap-ms "${2:?--publish-cap-ms needs a number}"); shift 2 ;;
        -h|--help)  sed -n '2,14p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "unknown option: $1 (try --help)"; exit 2 ;;
    esac
done

# ---------------------------------------------------------------- python env
VENV="${VENV:-.venv}"
if [ ! -x "$VENV/bin/python" ]; then
    echo ">> creating virtualenv in $VENV and installing requirements"
    python3 -m venv "$VENV" || { echo "python3 -m venv failed (Ubuntu: sudo apt install python3-venv)"; exit 1; }
    "$VENV/bin/pip" install -q -r requirements.txt || exit 1
fi
PY="$VENV/bin/python"

PORT=$("$PY" -c 'import tomllib,sys,urllib.parse as u; c=tomllib.load(open("config/sim.toml","rb")); print(u.urlparse(c["opcua"]["endpoint"]).port or 4840)')

# ---------------------------------------------------------------- find Godot
find_godot() {
    if [ -n "$GODOT_BIN" ]; then echo "$GODOT_BIN"; return; fi
    for c in godot godot4; do command -v "$c" >/dev/null 2>&1 && { echo "$c"; return; }; done
    for c in "$HOME"/godot/Godot_v*_linux.x86_64 "$HOME"/Godot*/Godot_v*_linux.x86_64 "$HOME"/Downloads/Godot_v*_linux.x86_64; do
        [ -x "$c" ] && { echo "$c"; return; }
    done
    if command -v flatpak >/dev/null 2>&1 && flatpak info org.godotengine.Godot >/dev/null 2>&1; then
        echo "flatpak run --share=network org.godotengine.Godot"; return
    fi
}

mkdir -p logs
PIDS=()
cleanup() {
    trap - EXIT INT TERM
    echo; echo ">> stopping"
    for p in "${PIDS[@]}"; do kill "$p" 2>/dev/null; done
    wait 2>/dev/null
}
trap cleanup EXIT INT TERM

# ----------------------------------------------------------------- simulator
echo ">> starting simulator (log: logs/sim.log)"
"$PY" -m ballplate.sim "${SIM_ARGS[@]}" > logs/sim.log 2>&1 &
SIM=$!; PIDS+=("$SIM")

echo -n ">> waiting for OPC UA server on port $PORT "
for _ in $(seq 1 60); do
    kill -0 "$SIM" 2>/dev/null || { echo; echo "simulator exited - see logs/sim.log:"; tail -n 15 logs/sim.log; exit 1; }
    (echo > "/dev/tcp/127.0.0.1/$PORT") 2>/dev/null && { echo " up"; break; }
    echo -n "."; sleep 0.5
done
(echo > "/dev/tcp/127.0.0.1/$PORT") 2>/dev/null || { echo; echo "server did not come up - see logs/sim.log"; exit 1; }

# ------------------------------------------------------------------ mock PLC
if [ "$MOCK" = 1 ]; then
    echo ">> starting mock PLC, pattern '$PATTERN' (log: logs/mock_plc.log)"
    "$PY" -m ballplate.mock_plc --url "opc.tcp://127.0.0.1:$PORT" --pattern "$PATTERN" > logs/mock_plc.log 2>&1 &
    PIDS+=("$!")
else
    echo ">> real PLC mode: connect your PLC to opc.tcp://$(hostname -I 2>/dev/null | awk '{print $1}'):$PORT"
fi

# --------------------------------------------------------------------- Godot
if [ "$USE_GODOT" = 1 ]; then
    G=$(find_godot)
    if [ -z "$G" ]; then
        echo "!! Godot not found. Install it, or pass --godot /path/to/Godot, or set GODOT=..."
        echo "   Continuing without a 3D view; press Ctrl+C to stop."
        wait "$SIM"
    else
        echo ">> starting Godot: $G  (close its window to stop everything)"
        # shellcheck disable=SC2086
        $G --path godot > logs/godot.log 2>&1 &
        GP=$!; PIDS+=("$GP")
        wait "$GP"
    fi
else
    echo ">> running headless; press Ctrl+C to stop.  Status: tail -f logs/sim.log"
    wait "$SIM"
fi
