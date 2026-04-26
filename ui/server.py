"""
AI Valet Robot – Web UI Server
Run on your laptop: python server.py
Then open http://localhost:5000 in your browser.

Demo mode (no robot needed):
    python server.py --demo

Real mode (robot running on Pi):
    python server.py
Commands are written to /tmp/valet_cmd and status is read from /tmp/valet_status.json
"""

import argparse
import json
import os
import threading
import time
from flask import Flask, jsonify, render_template, request

app = Flask(__name__)

STATUS_FILE  = "/tmp/valet_status.json"
COMMAND_FILE = "/tmp/valet_cmd"

# ---------------------------------------------------------------------------
# Shared state
# ---------------------------------------------------------------------------

state = {
    "robot_state":  "IDLE",
    "mission":      "NONE",
    "car1_status":  "waiting",   # waiting | ready | being_picked_up | parked | ready_to_retrieve
    "car2_status":  "waiting",
    "demo_mode":    False,
}
state_lock = threading.Lock()


# ---------------------------------------------------------------------------
# Demo mode – simulates robot state transitions
# ---------------------------------------------------------------------------

def _demo_mission(car_num: int):
    statuses = [
        ("being_picked_up", f"CAR{car_num}", "NAVIGATE",  2),
        ("being_picked_up", f"CAR{car_num}", "AT_SPOT",   1),
        ("being_picked_up", f"CAR{car_num}", "DELIVER",   2),
        ("parked",          f"CAR{car_num}", "AT_EXIT",   1),
        ("parked",          "NONE",          "RETURN",    2),
        ("parked",          "NONE",          "IDLE",      0),
    ]
    key = f"car{car_num}_status"
    for car_status, mission, robot_state, delay in statuses:
        with state_lock:
            state[key]           = car_status
            state["mission"]     = mission
            state["robot_state"] = robot_state
        if delay:
            time.sleep(delay)


def _demo_retrieve(car_num: int):
    key = f"car{car_num}_status"
    steps = [
        ("being_picked_up", f"CAR{car_num}", "NAVIGATE",  2),
        ("being_picked_up", f"CAR{car_num}", "AT_SPOT",   1),
        ("waiting",         "NONE",          "RETURN",    2),
        ("waiting",         "NONE",          "IDLE",      0),
    ]
    for car_status, mission, robot_state, delay in steps:
        with state_lock:
            state[key]           = car_status
            state["mission"]     = mission
            state["robot_state"] = robot_state
        if delay:
            time.sleep(delay)


# ---------------------------------------------------------------------------
# Real mode – read status from file written by robot
# ---------------------------------------------------------------------------

def _poll_status():
    while True:
        try:
            if os.path.exists(STATUS_FILE):
                with open(STATUS_FILE) as f:
                    data = json.load(f)
                with state_lock:
                    state["robot_state"] = data.get("robot_state", state["robot_state"])
                    state["mission"]     = data.get("mission",      state["mission"])
        except Exception:
            pass
        time.sleep(0.5)


def _send_command(cmd: str):
    try:
        with open(COMMAND_FILE, "w") as f:
            f.write(cmd)
    except Exception as e:
        print(f"Failed to write command: {e}")


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
            _send_command("1")

    elif cmd == "car2_ready":
        with state_lock:
            state["car2_status"] = "ready"
        if demo:
            threading.Thread(target=_demo_mission, args=(2,), daemon=True).start()
        else:
            _send_command("2")

    elif cmd == "retrieve_car1":
        with state_lock:
            if state["car1_status"] != "parked":
                return jsonify({"error": "Car 1 is not parked"}), 400
        if demo:
            threading.Thread(target=_demo_retrieve, args=(1,), daemon=True).start()
        else:
            _send_command("1")

    elif cmd == "retrieve_car2":
        with state_lock:
            if state["car2_status"] != "parked":
                return jsonify({"error": "Car 2 is not parked"}), 400
        if demo:
            threading.Thread(target=_demo_retrieve, args=(2,), daemon=True).start()
        else:
            _send_command("2")

    elif cmd == "estop":
        with state_lock:
            state["robot_state"] = "ESTOP"
        if not demo:
            _send_command(" ")

    return jsonify({"ok": True})


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--demo", action="store_true", help="Run in demo mode (no robot needed)")
    args = parser.parse_args()

    with state_lock:
        state["demo_mode"] = args.demo

    if args.demo:
        print("Running in DEMO MODE — no robot required")
    else:
        print("Running in REAL MODE — reading robot status from", STATUS_FILE)
        threading.Thread(target=_poll_status, daemon=True).start()

    app.run(host="0.0.0.0", port=5000, debug=False)
