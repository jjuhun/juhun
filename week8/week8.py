import RPi.GPIO as GPIO
import time

# 모터 핀 설정
motor1_pwm = 18  # PWM 핀
motor2_pwm = 23

GPIO.setmode(GPIO.BCM)
GPIO.setup(motor1_pwm, GPIO.OUT)
GPIO.setup(motor2_pwm, GPIO.OUT)

motor1 = GPIO.PWM(motor1_pwm, 100)  # 주파수 100Hz
motor2 = GPIO.PWM(motor2_pwm, 100)

motor1.start(0)  # 초기 속도 0
motor2.start(0)

def forward(speed):
    motor1.ChangeDutyCycle(speed)
    motor2.ChangeDutyCycle(speed)

def stop():
    motor1.ChangeDutyCycle(0)
    motor2.ChangeDutyCycle(0)

try:
    while True:
        command = input("Enter command (f: forward, s: stop): ")
        if command == 'f':
            forward(100)  # 최대 속도
        elif command == 's':
            stop()

except KeyboardInterrupt:
    pass

finally:
    motor1.stop()
    motor2.stop()
    GPIO.cleanup()
