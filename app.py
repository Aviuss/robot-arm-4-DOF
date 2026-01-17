from flask import Flask, render_template, jsonify, request
import threading
import robot_loop

shared_state = None
state_lock = None
MOVE_INCREMENT = 2.5

app = Flask(__name__)

@app.route("/")
def index():
    return render_template("index.html")

@app.route("/action/hello", methods=["POST"])
def hello_action():
    return jsonify({"message": "Hello world!"})
    

@app.route("/action/xplus", methods=["POST"])
def xplus():
    if shared_state != None and state_lock != None:
        with state_lock:
            shared_state["desired_hand_position"][0] += MOVE_INCREMENT

    return "", 200
    
@app.route("/action/yplus", methods=["POST"])
def yplus():
    if shared_state != None and state_lock != None:
        with state_lock:
            shared_state["desired_hand_position"][1] += MOVE_INCREMENT
    
    return "", 200
        
@app.route("/action/zplus", methods=["POST"])
def zplus():
    if shared_state != None and state_lock != None:
        with state_lock:
            shared_state["desired_hand_position"][2] += MOVE_INCREMENT

    return "", 200
    
@app.route("/action/xminus", methods=["POST"])
def xminus():
    if shared_state != None and state_lock != None:
        with state_lock:
            shared_state["desired_hand_position"][0] -= MOVE_INCREMENT
    
    return "", 200
        
@app.route("/action/yminus", methods=["POST"])
def yminus():
    if shared_state != None and state_lock != None:
        with state_lock:
            shared_state["desired_hand_position"][1] -= MOVE_INCREMENT

    return "", 200
    
@app.route("/action/zminus", methods=["POST"])
def zminus():
    if shared_state != None and state_lock != None:
        with state_lock:
            shared_state["desired_hand_position"][2] -= MOVE_INCREMENT

    return "", 200
    



if __name__ == "__main__":
    shared_state = {}
    state_lock = threading.Lock()

    robot_loop.init(shared_state, state_lock)

    app.run(
        host="0.0.0.0",
        port=5000,
        debug=True,
        use_reloader=False
    )