import time
import RPi.GPIO as GPIO
import pigpio


pi = pigpio.pi()
if not pi.connected:
    print("pigpio not connected!")
    exit()

for pulse in range(1000, 2001, 100):
    pi.set_servo_pulsewidth(18, pulse)
    time.sleep(0.5)


def main():
    GPIO.setmode(GPIO.BCM)
    LED_PIN = 12
    GPIO.setup(LED_PIN, GPIO.OUT)

    while True:
        print("loop")
        GPIO.output(LED_PIN, GPIO.LOW)
        time.sleep(1)
        GPIO.output(LED_PIN, GPIO.HIGH)
        pi.set_servo_pulsewidth(18, 0)    # off
        time.sleep(1)
        pi.set_servo_pulsewidth(18, 1000) # position anti-clockwise
        time.sleep(1)
        pi.set_servo_pulsewidth(18, 1500) # middle
        time.sleep(1)
        pi.set_servo_pulsewidth(18, 2000) # position clockwise
        time.sleep(1)



if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print("Error:", e)
    finally:
        GPIO.cleanup()
        pi.stop()