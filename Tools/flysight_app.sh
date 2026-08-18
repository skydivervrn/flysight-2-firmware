#!/bin/sh
# Start the FlySight BLE app in the bleak venv and open its page.
#
# CoreBluetooth is not reachable from a sandboxed process, so this must be run
# from a normal shell — a sandboxed run finds no devices at all, which looks
# exactly like a FlySight that is switched off.
#
#     Tools/flysight_app.sh            # http://127.0.0.1:8765/
#     Tools/flysight_app.sh --port 8770 --no-autoconnect
#
# Any arguments are passed straight through to flysight_app.py.
set -eu

VENV="${FLYSIGHT_VENV:-$HOME/.venvs/ble}"
PYTHON="$VENV/bin/python"
HERE="$(cd "$(dirname "$0")" && pwd)"
APP="$HERE/flysight_app.py"

if [ ! -x "$PYTHON" ]; then
    echo "No python at $PYTHON." >&2
    echo "Create the venv once:  python3 -m venv ~/.venvs/ble && ~/.venvs/ble/bin/pip install bleak" >&2
    echo "Or point FLYSIGHT_VENV at an existing one." >&2
    exit 1
fi

# The port the page lives on, so the browser can be opened at the right one.
PORT=8765
prev=""
for arg in "$@"; do
    case "$prev" in --port) PORT="$arg" ;; esac
    case "$arg" in --port=*) PORT="${arg#--port=}" ;; esac
    prev="$arg"
done

URL="http://127.0.0.1:$PORT/"

# Open the page a moment after the server binds. If the port is already taken
# the app exits with its own message and this just opens the page of whatever
# is already running there — which is the useful outcome either way.
( sleep 1; open "$URL" >/dev/null 2>&1 || true ) &

exec "$PYTHON" "$APP" "$@"
