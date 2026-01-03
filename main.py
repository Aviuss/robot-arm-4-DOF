import time
import RPi.GPIO as GPIO
import pigpio
import threading


def main():
    pi = pigpio.pi()
    if not pi.connected:
        print("pigpio not connected!")
        exit()

    GPIO.setmode(GPIO.BCM)
    while True:
        pulse = int(input("Enter servo pulse width (500-2500): "))
        pi.set_servo_pulsewidth(13, pulse)
        time.sleep(0.5)


def start_robot_loop(shared_state, lock):
    def loop():
        while True:
            with lock:
                if shared_state["enabled"]:
                    angles = shared_state["joint_angles"].copy()
                else:
                    angles = None

            if angles:
                # Put your robot movement logic here
                print("Moving robot to:", angles)

            time.sleep(0.05)  # 20 Hz loop

    worker = threading.Thread(target=loop, daemon=True)
    worker.start()
    return worker


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print("Error:", e)
    finally:
        GPIO.cleanup()
        pi.stop()