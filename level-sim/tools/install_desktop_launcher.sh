#!/usr/bin/env bash
# Creates a desktop/app-menu launcher for this simulation (Ubuntu/GNOME etc.).
#   tools/install_desktop_launcher.sh            -> ~/.local/share/applications + ~/Desktop (if it exists)
# Extra arguments are passed on to run.sh, e.g.  tools/install_desktop_launcher.sh --no-mock
set -eu
HERE="$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)"
NAME="Level Simulation"
ENTRY="[Desktop Entry]
Type=Application
Name=$NAME
Comment=Level simulator (OPC UA server) with Godot view
Exec=bash -c 'cd \"$HERE\" && ./run.sh $*; echo; read -p \"Press Enter to close\"'
Terminal=true
Categories=Development;Education;"
mkdir -p "$HOME/.local/share/applications"
printf '%s\n' "$ENTRY" > "$HOME/.local/share/applications/level-sim.desktop"
if [ -d "$HOME/Desktop" ]; then
    printf '%s\n' "$ENTRY" > "$HOME/Desktop/level-sim.desktop"
    chmod +x "$HOME/Desktop/level-sim.desktop"
    command -v gio >/dev/null && gio set "$HOME/Desktop/level-sim.desktop" metadata::trusted true 2>/dev/null || true
fi
echo "launcher installed ($NAME)"
