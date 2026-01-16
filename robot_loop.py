import time
import RPi.GPIO as GPIO
import pigpio
import threading
import define_static

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


def loop(global_state):
    min_v = -60
    max_v = 60
    degrees = min_v
    increment = 1
    while True:
        set_degrees("q1", degrees, global_state)
        degrees += increment
        if degrees >= max_v:
            increment = -abs(increment)
            degrees = max_v
            time.sleep(0.5)
        elif degrees <= min_v:
            increment = abs(increment)
            degrees = min_v
            time.sleep(0.5)
            
        time.sleep(0.05)



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
