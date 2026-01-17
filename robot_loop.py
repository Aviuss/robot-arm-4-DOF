import time
import RPi.GPIO as GPIO
import pigpio
import threading
import define_static
from ik_solutions import numberical_solution, deg2rad, rad2deg
import numpy as np

MOTOR_SPEED = 3

def set_pulse(motor_q, pulse, global_state):
    pin = global_state["q pinout"][motor_q]
    bounds = global_state["q pulse restriction"][motor_q]
    pi = global_state["variables"]["pi"]

    if pin is None or bounds is None or pi is None:
        print("Invalid data for setting pulse")
        return

    if pulse < bounds[0]:
        pulse = bounds[0]
    elif pulse > bounds[1]:
        pulse = bounds[1]

    pi.set_servo_pulsewidth(pin, pulse)

def set_degrees(motor_q, degrees, global_state):
    q_degrees_bounds = global_state["q degrees bounds"][motor_q]
    set_position_lambda = global_state["set position lambda"][motor_q]

    if q_degrees_bounds is None or set_position_lambda is None:
        print("Invalid data for setting degrees")
        return

    if degrees < q_degrees_bounds[0]:
        degrees = q_degrees_bounds[0]
    elif degrees > q_degrees_bounds[1]:
        degrees = q_degrees_bounds[1]

    set_pulse(motor_q, set_position_lambda(degrees), global_state)

def update_motors(global_state):
    with global_state["variables"]["shared"]["lock"]:
        state = global_state["variables"]["shared"]["state"]
        def handle_motor(q):        
            if (state["desired"][q] != state["actual"][q]):
                displacement = state["desired"][q] - state["actual"][q]
                displacement = min(max(displacement, -MOTOR_SPEED), MOTOR_SPEED)
                state["actual"][q] += displacement
                set_degrees(q, state["actual"][q], global_state)

        handle_motor("q1")
        handle_motor("q2")
        handle_motor("q3")
        handle_motor("q4")
        handle_motor("q5")

def toIKVector(object):
    return np.array([
        deg2rad(object["q1"]),
        deg2rad(object["q2"]),
        deg2rad(object["q3"]),
        deg2rad(object["q4"]),
    ])

def toIKBounds(global_state):
    return [
        (deg2rad(global_state["q degrees bounds"]["q1"][0]), deg2rad(global_state["q degrees bounds"]["q1"][1])),
        (deg2rad(global_state["q degrees bounds"]["q2"][0]), deg2rad(global_state["q degrees bounds"]["q2"][1])),
        (deg2rad(global_state["q degrees bounds"]["q3"][0]), deg2rad(global_state["q degrees bounds"]["q3"][1])),
        (deg2rad(global_state["q degrees bounds"]["q4"][0]), deg2rad(global_state["q degrees bounds"]["q4"][1]))
    ]

def L_runAndApplyIK(shared_state, global_state, desired_hand_position):
        ik_res = numberical_solution(
            toIKVector(shared_state["actual"]),
            global_state["lengths"],
            toIKBounds(global_state),
            desired_hand_position,
            gradient_iterations=10, gradient_iterations_per_optimization=200, gradient_a = 0.001,
            custom_optimization_iterations_per_gradient = 0, custom_optimization_step = 0.005
        )
        degrees_q = list(map(lambda x: rad2deg(x), ik_res["best_q"]))
        shared_state["desired"]["q1"] = degrees_q[0]
        shared_state["desired"]["q2"] = degrees_q[1]
        shared_state["desired"]["q3"] = degrees_q[2]
        shared_state["desired"]["q4"] = degrees_q[3]

def L_init_on_loop(shared_state, global_state):
    global_state["variables"]["shared"]["state"].clear()
    global_state["variables"]["shared"]["state"].update({
        "desired": { "q1": 0, "q2": 90, "q3": -45, "q4": -45, "q5": 0 },
        "actual":  { "q1": 0, "q2": 90, "q3": -45, "q4": -45, "q5": 0 },
        "desired_hand_position": np.array([30, 0, 10]),
        "prev_desired_hand_position": np.array([9999, 9999, 9999])
    })
    set_degrees("q1", shared_state["actual"]["q1"], global_state)
    set_degrees("q2", shared_state["actual"]["q2"], global_state)
    set_degrees("q3", shared_state["actual"]["q3"], global_state)
    set_degrees("q4", shared_state["actual"]["q4"], global_state)
    set_degrees("q5", shared_state["actual"]["q5"], global_state)

def loop(global_state):
    shared_lock = global_state["variables"]["shared"]["lock"]
    shared_state = global_state["variables"]["shared"]["state"]
    with shared_lock:
        L_init_on_loop(shared_state, global_state)

    while True:
        with shared_lock:
            difference = np.linalg.norm(shared_state["prev_desired_hand_position"] - shared_state["desired_hand_position"])
            if (difference > 0.5):
                L_runAndApplyIK(shared_state, global_state, shared_state["desired_hand_position"])
                shared_state["prev_desired_hand_position"] = shared_state["desired_hand_position"].copy()
            

        update_motors(global_state)
        time.sleep(0.1)



def init(shared_state, shared_lock):
    pi = pigpio.pi()
    tries = 0
    while not pi.connected:
        print("pigpio not connected!")
        tries += 1
        time.sleep(5)
        pi = pigpio.pi()
        if tries > 5:
            raise Exception("Could not connect to pigpio")
        
    GPIO.setmode(GPIO.BCM)

    global_state = define_static.define(shared_state, shared_lock, pi)

    def loopWithCleaning(global_state):
        try:
            loop(global_state)
        except Exception as e:
            print("Error:", e)
        finally:
            GPIO.cleanup()
            pi.stop()

    threading.Thread(target=loopWithCleaning, args=(global_state,), daemon=True).start()

if __name__ == "__main__":
    init({}, threading.Lock())
    while True:
        time.sleep(60)
