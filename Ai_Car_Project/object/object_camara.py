import cv2 as cv
import numpy as np
import threading
import time

# 클래스 이름 로드
class_names = []
with open('object_detection_classes_coco.txt', 'r') as f:
    class_names = f.read().split('\n')

# 랜덤 색상 생성
COLORS = np.random.uniform(0, 255, size=(len(class_names), 3))

# 객체 탐지 모델 로드
model = cv.dnn.readNetFromTensorflow(model='frozen_inference_graph.pb', config='ssd_mobilenet_v2_coco_2018_03_29.pbtxt')

# 카메라로부터 실시간 영상 받기
camera = cv.VideoCapture(0)  # 기본 웹캠을 사용

# 영상 프레임을 가져오는 함수
def capture_frame():
    while True:
        ret, frame = camera.read()  # 카메라에서 한 프레임 읽기
        if not ret:
            print("Failed to grab frame")
            break
        if len(frame.shape) == 2:  # frame이 2D 배열이면 (흑백 이미지)
            frame = cv.cvtColor(frame, cv.COLOR_GRAY2BGR)  # BGR로 변환
        queue.put(frame)

# 객체 탐지 함수
def detect_objects():
    while True:
        if not queue.empty():
            frame = queue.get()  # 큐에서 프레임 가져오기
            image_height, image_width, _ = frame.shape

            # 이미지를 모델에 입력하기 위해 blob 형태로 변환
            blob = cv.dnn.blobFromImage(frame, 1.0, (300, 300), (104, 177, 123), crop=False)

            # 모델에 blob 입력
            model.setInput(blob)

            # 모델을 통해 출력 값 받기
            output = model.forward()

            # 탐지된 객체들을 순회
            for detection in output[0, 0, :, :]:
                confidence = detection[2]  # 신뢰도

                # 신뢰도가 일정 기준 이상일 경우만 처리
                if confidence > 0.4:
                    class_id = int(detection[1])  # 객체의 클래스 ID
                    class_name = class_names[class_id - 1]  # 클래스 ID로 객체 이름 가져오기

                    if class_name == 'fire hydrant':  # '소화전'만 탐지
                        color = COLORS[class_id]  # 클래스 ID에 맞는 색상

                        # 바운딩 박스 좌표 계산
                        box_x = int(detection[3] * image_width)
                        box_y = int(detection[4] * image_height)
                        box_width = int(detection[5] * image_width)
                        box_height = int(detection[6] * image_height)

                        # 바운딩 박스 그리기
                        cv.rectangle(frame, (box_x, box_y), (box_x + box_width, box_y + box_height), color, 2)

                        # 클래스 이름을 이미지에 텍스트로 표시
                        cv.putText(frame, class_name, (box_x, box_y - 5), cv.FONT_HERSHEY_SIMPLEX, 1, color, 2)

            # 실시간으로 처리된 이미지를 화면에 표시
            cv.imshow('Object Detection', frame)

# 큐를 사용하여 프레임과 객체 탐지 작업을 병렬로 처리
from queue import Queue
queue = Queue(maxsize=1)  # 큐 크기 1로 설정

# 스레드로 프레임 캡처 및 객체 탐지 실행
capture_thread = threading.Thread(target=capture_frame)
detect_thread = threading.Thread(target=detect_objects)

capture_thread.start()
detect_thread.start()

# 'q' 키를 누르면 종료
while True:
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

# 종료 시 리소스 해제
capture_thread.join()
detect_thread.join()
camera.release()
cv.destroyAllWindows()
