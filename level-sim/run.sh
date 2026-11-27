#!/usr/bin/env bash
# Start the level demo: simulator (OPC UA server) + mock PLC + Godot.  Standalone: no other
# simulation is needed, and it uses its own ports (OPC UA 4843, UDP 9876/9877).
#
#   ./run.sh               simulator + mock PLC (on/off controller on T_LSH10/T_LSL10) + Godot
#   ./run.sh --mode pi     mock PLC as PI controller on T_LT10 (--setpoint 60 in %)
#   ./run.sh --no-mock     real PLC: only simulator + Godot (PLC connects to opc.tcp://<this-pc>:4843)
#   ./run.sh --no-godot    headless: simulator + mock PLC only
#   ./run.sh --godot /path/to/Godot_v4.3-stable_linux.x86_64
#   ./run.sh --publish-cap-ms 20
#
# Environment: VENV=<dir> other virtualenv (default ./.venv, created if missing), GODOT=<cmd>
# Closing the Godot window or pressing Ctrl+C stops everything.  Logs: ./logs/

set -u
cd "$(dirname "$(readlink -f "$0")")"

MODE=onoff; SETPOINT=50; MOCK=1; USE_GODOT=1; GODOT_BIN="${GODOT:-}"; SIM_ARGS=()
while [ $# -gt 0 ]; do
    case "$1" in
        --mode)     MODE="${2:?--mode needs onoff or pi}"; shift 2 ;;
        --setpoint) SETPOINT="${2:?--setpoint needs a level in %}"; shift 2 ;;
        --no-mock)  MOCK=0; shift ;;
        --no-godot) USE_GODOT=0; shift ;;
        --godot)    GODOT_BIN="${2:?--godot needs a path}"; shift 2 ;;
        --publish-cap-ms) SIM_ARGS+=(--publish-cap-ms "${2:?--publish-cap-ms needs a number}"); shift 2 ;;
        -h|--help)  sed -n '2,14p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "unknown option: $1 (try --help)"; exit 2 ;;
    esac
done

VENV="${VENV:-.venv}"
if [ ! -x "$VENV/bin/python" ]; then
    echo ">> creating virtualenv in $VENV and installing requirements"
    python3 -m venv "$VENV" || { echo "python3 -m venv failed (Ubuntu: sudo apt install python3-venv)"; exit 1; }
    "$VENV/bin/pip" install -q -r requirements.txt || exit 1
fi
PY="$VENV/bin/python"

eval "$("$PY" -c 'import tomllib,urllib.parse as u; c=tomllib.load(open("config/level.toml","rb")); print("PORT=%d UDP_PORT=%d CMD_PORT=%d" % (u.urlparse(c["opcua"]["endpoint"]).port or 4843, c["visual"]["port"], c["visual"]["command_port"]))')"

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

echo ">> starting level simulator (log: logs/sim.log)"
"$PY" -m levelsim.sim "${SIM_ARGS[@]}" > logs/sim.log 2>&1 &
SIM=$!; PIDS+=("$SIM")

echo -n ">> waiting for OPC UA server on port $PORT "
for _ in $(seq 1 60); do
    kill -0 "$SIM" 2>/dev/null || { echo; echo "simulator exited - see logs/sim.log:"; tail -n 15 logs/sim.log; exit 1; }
    (echo > "/dev/tcp/127.0.0.1/$PORT") 2>/dev/null && { echo " up"; break; }
    echo -n "."; sleep 0.5
done
(echo > "/dev/tcp/127.0.0.1/$PORT") 2>/dev/null || { echo; echo "server did not come up - see logs/sim.log"; exit 1; }

if [ "$MOCK" = 1 ]; then
    echo ">> starting mock PLC, mode $MODE (log: logs/mock_plc.log)"
    "$PY" -m levelsim.mock_plc --url "opc.tcp://127.0.0.1:$PORT" --mode "$MODE" --setpoint "$SETPOINT" > logs/mock_plc.log 2>&1 &
    PIDS+=("$!")
else
    echo ">> real PLC mode: connect your PLC to opc.tcp://$(hostname -I 2>/dev/null | awk '{print $1}'):$PORT"
fi

if [ "$USE_GODOT" = 1 ]; then
    G=$(find_godot)
    if [ -z "$G" ]; then
        echo "!! Godot not found. Install it, or pass --godot /path/to/Godot, or set GODOT=..."
        echo "   Continuing without a 3D view; press Ctrl+C to stop."
        wait "$SIM"
    else
        echo ">> starting Godot: $G  (close its window to stop everything)"
        # shellcheck disable=SC2086
        $G --path godot -- --port "$UDP_PORT" --cmd-port "$CMD_PORT" > logs/godot.log 2>&1 &
        GP=$!; PIDS+=("$GP")
        wait "$GP"
    fi
else
    echo ">> running headless; press Ctrl+C to stop.  Status: tail -f logs/sim.log"
    wait "$SIM"
fi
