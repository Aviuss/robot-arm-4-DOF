from flask import Flask, render_template, jsonify, request
import threading
import robot_loop


app = Flask(__name__)

@app.route("/")
def index():
    return render_template("index.html")

@app.route("/action/hello", methods=["POST"])
def hello_action():
    return jsonify({"message": "Hello world!"})

if __name__ == "__main__":
    robot_state = {
        "joint_angles": [0, 0, 0, 0],
        "enabled": False
    }

    state_lock = threading.Lock()

    robot_loop.init(robot_state, state_lock)

    app.run(
        host="0.0.0.0",
        port=5000,
        debug=True,
        use_reloader=False
    )