import RPi.GPIO as GPIO
import time
import threading
import serial

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

gData = ""  # 전역 변수로 Bluetooth 명령 저장

def go():
    motor1.ChangeDutyCycle(100)  # 최대 속도
    motor2.ChangeDutyCycle(100)

def back():
    motor1.ChangeDutyCycle(100)
    motor2.ChangeDutyCycle(100)

def left():
    motor1.ChangeDutyCycle(50)  # 왼쪽 모터 속도 조정
    motor2.ChangeDutyCycle(100)

def right():
    motor1.ChangeDutyCycle(100)
    motor2.ChangeDutyCycle(50)  # 오른쪽 모터 속도 조정

def stop():
    motor1.ChangeDutyCycle(0)
    motor2.ChangeDutyCycle(0)

def serial_thread():
    global gData
    # Bluetooth 시리얼 포트 설정 (포트 이름을 확인 후 수정)
    ser = serial.Serial('/dev/rfcomm0', 9600)  # Bluetooth 시리얼 포트
    while True:
        if ser.in_waiting > 0:
            gData = ser.readline().decode().strip()  # 명령 읽기
            print("Received command:", gData)

def main():
    global gData
    threading.Thread(target=serial_thread, daemon=True).start()  # 시리얼 스레드 시작

    try:
        while True:
            if gData == "go":
                go()
            elif gData == "back":
                back()
            elif gData == "left":
                left()
            elif gData == "right":
                right()
            elif gData == "stop":
                stop()
            time.sleep(0.1)  # 주기적으로 명령 체크

    except KeyboardInterrupt:
        stop()  # 프로그램 종료 시 정지
    finally:
        motor1.stop()
        motor2.stop()
        GPIO.cleanup()

if __name__ == "__main__":
    main()

