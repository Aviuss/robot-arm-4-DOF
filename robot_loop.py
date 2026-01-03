import time
import RPi.GPIO as GPIO
import pigpio
import threading


def loop(pi, shared_state, lock):
    '''with lock:
        if shared_state["enabled"]:
            angles = shared_state["joint_angles"].copy()
        else:
            angles = None'''
    
    pulse = 500
    increment = 20

    while True:
        pi.set_servo_pulsewidth(13, pulse)
        pulse += increment
        if pulse >= 2500:
            increment = -abs(increment)
            pulse = 2500
            time.sleep(0.5)
        elif pulse <= 500:
            increment = abs(increment)
            pulse = 500
            time.sleep(0.5)
            
        time.sleep(0.05)



def init(shared_state, lock):
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

    def loopWithCleaning(pi, shared_state, lock):
        try:
            loop(pi, shared_state, lock)
        except Exception as e:
            print("Error:", e)
        finally:
            GPIO.cleanup()
            pi.stop()

    threading.Thread(target=loopWithCleaning, args=(pi, shared_state, lock), daemon=True).start()

if __name__ == "__main__":
    init({}, threading.Lock())
    while True:
        time.sleep(60)
