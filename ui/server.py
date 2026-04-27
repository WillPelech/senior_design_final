"""
AI Valet Robot – Web UI Server
Run on your laptop: python server.py --host <pi-ip> --user will
Then open http://localhost:5000 in your browser.

Demo mode (no robot needed):
    python server.py --demo

Real mode (SSHes into Pi to send commands):
    python server.py --host 10.20.3.136 --user will
"""

import argparse
import json
import os
import subprocess
import threading
import time
from flask import Flask, jsonify, render_template, request

app = Flask(__name__)

# Filled in from CLI args
PI_HOST = None
PI_USER = None
COMMAND_FILE  = "/tmp/valet_cmd"
TMUX_SESSION  = "valet"

# ---------------------------------------------------------------------------
# Shared state
# ---------------------------------------------------------------------------

state = {
    "robot_state":  "IDLE",
    "mission":      "NONE",
    "car1_status":  "waiting",
    "car2_status":  "waiting",
    "demo_mode":    False,
    "connected":    False,
}
state_lock = threading.Lock()


# ---------------------------------------------------------------------------
# SSH helpers
# ---------------------------------------------------------------------------

def _ssh(command: str) -> bool:
    """Run a command on the Pi via SSH. Returns True on success."""
    if not PI_HOST:
        return False
    try:
        result = subprocess.run(
            ["ssh", "-o", "ConnectTimeout=3", "-o", "BatchMode=yes",
             f"{PI_USER}@{PI_HOST}", command],
            timeout=5, capture_output=True,
        )
        return result.returncode == 0
    except Exception as e:
        print(f"SSH error: {e}")
        return False


def _send_robot_command(key: str):
    """Send a keypress to the robot's tmux session via SSH."""
    _ssh(f"tmux send-keys -t {TMUX_SESSION} '{key}' ''")


def _poll_pi_status():
    """Periodically fetch robot state from Pi via SSH."""
    while True:
        try:
            result = subprocess.run(
                ["ssh", "-o", "ConnectTimeout=2", "-o", "BatchMode=yes",
                 f"{PI_USER}@{PI_HOST}", f"cat /tmp/valet_status.json 2>/dev/null"],
                timeout=4, capture_output=True, text=True,
            )
            if result.returncode == 0 and result.stdout.strip():
                data = json.loads(result.stdout)
                with state_lock:
                    state["robot_state"] = data.get("robot_state", state["robot_state"])
                    state["mission"]     = data.get("mission",      state["mission"])
                    state["connected"]   = True
            else:
                with state_lock:
                    state["connected"] = False
        except Exception:
            with state_lock:
                state["connected"] = False
        time.sleep(1)


# ---------------------------------------------------------------------------
# Demo mode – simulates robot state transitions without SSH
# ---------------------------------------------------------------------------

def _demo_mission(car_num: int):
    key = f"car{car_num}_status"
    steps = [
        ("being_picked_up", f"CAR{car_num}", "NAVIGATE", 2),
        ("being_picked_up", f"CAR{car_num}", "AT_SPOT",  1),
        ("being_picked_up", f"CAR{car_num}", "DELIVER",  2),
        ("parked",          f"CAR{car_num}", "AT_EXIT",  1),
        ("parked",          "NONE",          "RETURN",   2),
        ("parked",          "NONE",          "IDLE",     0),
    ]
    for car_status, mission, robot_state, delay in steps:
        with state_lock:
            state[key]           = car_status
            state["mission"]     = mission
            state["robot_state"] = robot_state
        if delay:
            time.sleep(delay)


def _demo_retrieve(car_num: int):
    key = f"car{car_num}_status"
    steps = [
        ("being_picked_up", f"CAR{car_num}", "NAVIGATE", 2),
        ("being_picked_up", f"CAR{car_num}", "AT_SPOT",  1),
        ("waiting",         "NONE",          "RETURN",   2),
        ("waiting",         "NONE",          "IDLE",     0),
    ]
    for car_status, mission, robot_state, delay in steps:
        with state_lock:
            state[key]           = car_status
            state["mission"]     = mission
            state["robot_state"] = robot_state
        if delay:
            time.sleep(delay)


# ---------------------------------------------------------------------------
# Routes
# ---------------------------------------------------------------------------

@app.route("/")
def index():
    return render_template("index.html")


@app.route("/api/status")
def api_status():
    with state_lock:
        return jsonify(dict(state))


@app.route("/api/command", methods=["POST"])
def api_command():
    data = json.loads(request.data)
    cmd  = data.get("command")

    with state_lock:
        demo = state["demo_mode"]

    if cmd == "car1_ready":
        with state_lock:
            state["car1_status"] = "ready"
        if demo:
            threading.Thread(target=_demo_mission, args=(1,), daemon=True).start()
        else:
            _send_robot_command("1")

    elif cmd == "car2_ready":
        with state_lock:
            state["car2_status"] = "ready"
        if demo:
            threading.Thread(target=_demo_mission, args=(2,), daemon=True).start()
        else:
            _send_robot_command("2")

    elif cmd == "retrieve_car1":
        with state_lock:
            if state["car1_status"] != "parked":
                return jsonify({"error": "Car 1 is not parked"}), 400
        if demo:
            threading.Thread(target=_demo_retrieve, args=(1,), daemon=True).start()
        else:
            _send_robot_command("1")

    elif cmd == "retrieve_car2":
        with state_lock:
            if state["car2_status"] != "parked":
                return jsonify({"error": "Car 2 is not parked"}), 400
        if demo:
            threading.Thread(target=_demo_retrieve, args=(2,), daemon=True).start()
        else:
            _send_robot_command("2")

    elif cmd == "estop":
        with state_lock:
            state["robot_state"] = "ESTOP"
        if not demo:
            _send_robot_command(" ")

    return jsonify({"ok": True})


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--demo", action="store_true", help="Demo mode — no robot needed")
    parser.add_argument("--host", default=None,  help="Pi IP address")
    parser.add_argument("--user", default="will", help="Pi SSH username (default: will)")
    args = parser.parse_args()

    with state_lock:
        state["demo_mode"] = args.demo

    if args.demo:
        print("Running in DEMO MODE — no robot required")
    else:
        if not args.host:
            print("ERROR: provide --host <pi-ip> or use --demo")
            raise SystemExit(1)
        PI_HOST = args.host
        PI_USER = args.user
        print(f"Connecting to Pi at {PI_USER}@{PI_HOST}")
        threading.Thread(target=_poll_pi_status, daemon=True).start()

    app.run(host="0.0.0.0", port=5000, debug=False)
