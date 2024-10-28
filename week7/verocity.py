import RPi.GPIO as GPIO
import time

# 핀 설정
PWMA = 18  # 왼쪽 모터 PWM
AIN1 = 22  # 왼쪽 모터 방향 1
AIN2 = 27  # 왼쪽 모터 방향 2

PWMB = 23  # 오른쪽 모터 PWM
BIN1 = 25  # 오른쪽 모터 방향 1
BIN2 = 24  # 오른쪽 모터 방향 2

# GPIO 초기화
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(PWMA, GPIO.OUT)
GPIO.setup(AIN1, GPIO.OUT)
GPIO.setup(AIN2, GPIO.OUT)
GPIO.setup(PWMB, GPIO.OUT)
GPIO.setup(BIN1, GPIO.OUT)
GPIO.setup(BIN2, GPIO.OUT)

# PWM 인스턴스 생성
L_Motor = GPIO.PWM(PWMA, 500)  # 왼쪽 모터
R_Motor = GPIO.PWM(PWMB, 500)  # 오른쪽 모터
L_Motor.start(0)
R_Motor.start(0)

try:
    while True:
        # 모터 정방향 회전
        GPIO.output(AIN1, 0)
        GPIO.output(AIN2, 1)
        GPIO.output(BIN1, 0)
        GPIO.output(BIN2, 1)
        
        # 속도 설정
        L_Motor.ChangeDutyCycle(50)
        R_Motor.ChangeDutyCycle(50)
        time.sleep(0.1)  # 대기

        # 모터 정지
        L_Motor.ChangeDutyCycle(0)
        R_Motor.ChangeDutyCycle(0)
        time.sleep(0.1)  # 대기

except KeyboardInterrupt:
    pass  # 종료 처리

# GPIO 정리
GPIO.cleanup()
