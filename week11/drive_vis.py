import cv2 as cv
import numpy as np
import threading
import time
import SDcar

# 상수 및 전역 변수
SPEED = 40  # 속도
EPSILON = 0.0001  # 오차 허용 범위
ENABLE_LINE_TRACING = False  # 라인 추적 활성화 여부
IS_RUNNING = True  # 프로그램 실행 상태
MOMENT = np.array([0, 0, 0])  # 물체의 중심 좌표 기록용

# 라인 추적을 위한 그리드 설정
V_X = 320  # 화면의 가로 크기
V_Y = 240  # 화면의 세로 크기
V_X_GRID = [int(V_X * i / 10) for i in range(1, 10)]  # 그리드 라인 위치 설정

# 자동차 인스턴스
car = SDcar.Drive()

def func_thread():
    """프로그램을 계속 실행하기 위한 쓰레드 함수."""
    count = 0
    while True:
        time.sleep(1)
        count += 1
        if not IS_RUNNING:
            break

def key_cmd(which_key):
    """키 입력에 따른 자동차 제어."""
    global ENABLE_LINE_TRACING
    is_exit = False

    if which_key & 0xFF == ord('w'):
        print('앞으로 이동')
        car.motor_go(SPEED)
    elif which_key & 0xFF == ord('s'):
        print('뒤로 이동')
        car.motor_back(SPEED)
    elif which_key & 0xFF == ord('a'):
        print('왼쪽으로 회전')
        car.motor_left(SPEED)
    elif which_key & 0xFF == ord('d'):
        print('오른쪽으로 회전')
        car.motor_right(SPEED)
    elif which_key & 0xFF == ord('q'):  # 정지
        car.motor_stop()
        print('정지')
        is_exit = True
    elif which_key & 0xFF == ord('e'):  # 라인 추적 시작
        ENABLE_LINE_TRACING = True
        print(f'라인 추적 활성화: {ENABLE_LINE_TRACING}')
    elif which_key & 0xFF == ord('r'):  # 라인 추적 중지
        ENABLE_LINE_TRACING = False
        car.motor_stop()
        print(f'라인 추적 비활성화: {ENABLE_LINE_TRACING}')

    return is_exit

def detect_maskY_HSV(frame):
    """HSV 색 공간을 사용하여 노란색 마스크를 감지."""
    hsv_frame = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    blurred_hsv = cv.GaussianBlur(hsv_frame, (5, 5), cv.BORDER_DEFAULT)
    mask_yellow = cv.inRange(blurred_hsv, (25, 50, 100), (25, 255, 255))
    return mask_yellow

def detect_maskY_BGR(frame):
    """BGR 색 공간을 사용하여 노란색 마스크를 감지."""
    b, g, r = cv.split(frame)
    yellow = (g * 0.5 + r * 0.5 - b * 0.7).astype(np.uint8)
    yellow_blurred = cv.GaussianBlur(yellow, (5, 5), cv.BORDER_DEFAULT)
    _, mask_yellow = cv.threshold(yellow_blurred, 100, 255, cv.THRESH_BINARY)
    return mask_yellow

def show_grid(img):
    """디버깅을 위해 이미지에 그리드를 표시."""
    height, _, _ = img.shape
    for x in V_X_GRID:
        cv.line(img, (x, 0), (x, height), (0, 255, 0), 1, cv.LINE_4)

def line_tracing(cx):
    """라인 추적 로직, 자동차의 위치를 기반으로 제어."""
    global MOMENT
    tolerance = 0.1  # 허용 오차
    diff = 0

    if MOMENT[0] != 0 and MOMENT[1] != 0 and MOMENT[2] != 0:
        avg_moment = np.mean(MOMENT)
        diff = np.abs(avg_moment - cx) / V_X

    print(f'차이: {diff:.4f}')

    if diff <= tolerance:
        MOMENT[0] = MOMENT[1]
        MOMENT[1] = MOMENT[2]
        MOMENT[2] = cx

        if V_X_GRID[1] <= cx < V_X_GRID[2]:
            car.motor_go(SPEED)
            print('앞으로 이동')
        elif cx < V_X_GRID[1]:
            car.motor_left(SPEED)
            print('왼쪽으로 회전')
        elif cx >= V_X_GRID[2]:
            car.motor_right(SPEED)
            print('오른쪽으로 회전')
    else:
        car.motor_go(SPEED)
        print('앞으로 이동')
        MOMENT = [0, 0, 0]

def main():
    """카메라 처리 및 자동차 제어 메인 함수."""
    global IS_RUNNING, ENABLE_LINE_TRACING

    camera = cv.VideoCapture(0)
    camera.set(cv.CAP_PROP_FRAME_WIDTH, V_X)
    camera.set(cv.CAP_PROP_FRAME_HEIGHT, V_Y)

    try:
        while camera.isOpened():
            ret, frame = camera.read()
            if not ret:
                break

            frame = cv.flip(frame, -1)
            cv.imshow('Camera', frame)

            # 라인 추적을 위한 이미지 처리
            crop_img = frame[180:, :]
            mask_y = detect_maskY_BGR(crop_img)

            contours, _ = cv.findContours(mask_y, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
            if contours:
                largest_contour = max(contours, key=cv.contourArea)
                moments = cv.moments(largest_contour)

                if moments['m00'] != 0:
                    cx = int(moments['m10'] / moments['m00'])
                    cy = int(moments['m01'] / moments['m00'])
                    cv.circle(crop_img, (cx, cy), 3, (0, 0, 255), -1)
                    cv.drawContours(crop_img, contours, -1, (0, 255, 0), 3)
                    cv.putText(crop_img, str(cx), (10, 10), cv.FONT_HERSHEY_DUPLEX, 0.5, (0, 255, 0))

                    if ENABLE_LINE_TRACING:
                        line_tracing(cx)

            # 디버깅용 그리드 표시
            show_grid(crop_img)
            cv.imshow('Processed Image', cv.resize(crop_img, dsize=(0, 0), fx=2, fy=2))

            # 키 입력 처리
            is_exit = False
            which_key = cv.waitKey(20)
            if which_key > 0:
                is_exit = key_cmd(which_key)

            if is_exit:
                cv.destroyAllWindows()
                break

    except Exception as e:
        print(f"오류: {e}")
        IS_RUNNING = False

if __name__ == '__main__':
    # 백그라운드 쓰레드 시작
    threading.Thread(target=func_thread, daemon=True).start()

    print(V_X_GRID)
    main()

    # 정리
    IS_RUNNING = False
    car.clean_GPIO()
    print('프로그램 종료')
