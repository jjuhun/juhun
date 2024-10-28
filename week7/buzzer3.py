# -*- coding: utf-8 -*-

import RPi.GPIO as GPIO
import time

# 핀 설정
SWITCH_PINS = [5, 6, 13, 19]  # 스위치 핀
BUZZER_PIN = 12                # 버저 핀

# 주파수 매핑
NOTES = {
    'C': 262,       # 도
    'D': 294,       # 레
    'E': 330,       # 미
    'C_high': 523   # 높은 도
}

# GPIO 초기화 함수
def setup_gpio():
    GPIO.setwarnings(False)  # 경고 메시지 비활성화
    GPIO.setmode(GPIO.BCM)   # BCM 모드 설정
    GPIO.setup(BUZZER_PIN, GPIO.OUT)  # 버저 핀을 출력으로 설정
    GPIO.setup(SWITCH_PINS, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)  # 스위치 핀을 입력으로 설정

# 음 재생 함수
def play_tone(pwm, note):
    if note in NOTES:  # 유효한 음인지 확인
        pwm.ChangeFrequency(NOTES[note])  # 주파수 변경
        pwm.ChangeDutyCycle(50)  # 듀티 사이클 설정
        print(f"Playing {note}")  # 현재 재생 중인 음 출력

# 메인 루프
def main():
    setup_gpio()  # GPIO 초기화
    pwm = GPIO.PWM(BUZZER_PIN, NOTES['C'])  # PWM 설정, 기본 음은 'C'
    pwm.start(0)  # PWM 시작

    try:
        while True:
            sound_playing = False  # 소리 재생 상태 초기화
            
            for i, pin in enumerate(SWITCH_PINS):
                if GPIO.input(pin) == GPIO.HIGH:  # 스위치가 눌렸는지 확인
                    if i == 0:
                        play_tone(pwm, 'C')  # SW1: 도
                    elif i == 1:
                        play_tone(pwm, 'D')  # SW2: 레
                    elif i == 2:
                        play_tone(pwm, 'E')  # SW3: 미
                    elif i == 3:
                        play_tone(pwm, 'C_high')  # SW4: 높은 도
                    sound_playing = True  # 소리 재생 상태 업데이트

            if not sound_playing:
                pwm.ChangeDutyCycle(0)  # 소리 정지
            
            time.sleep(0.1)  # CPU 사용률 절약

    except KeyboardInterrupt:
        pass  # Ctrl+C로 종료 시 예외 처리
    finally:
        pwm.stop()  # PWM 중지
        GPIO.cleanup()  # GPIO 설정 정리

if __name__ == "__main__":
    main()  # 메인 함수 실행
