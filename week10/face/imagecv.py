import cv2
import time

# 얼굴 검출을 위한 Haar Cascade 파일 경로
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

# 라즈베리 파이 카메라 사용
# '/dev/video0'은 일반적인 카메라 장치 경로입니다. 필요에 따라 카메라 장치 번호를 변경할 수 있습니다.
cap = cv2.VideoCapture(0)

# 카메라 열기 확인
if not cap.isOpened():
    print("카메라를 열 수 없습니다.")
    exit()

# 비디오 캡처 크기 설정 (optional)
cap.set(3, 640)  # 가로 해상도
cap.set(4, 480)  # 세로 해상도

# 실시간 얼굴 검출 및 비디오 출력
while True:
    # 카메라로부터 프레임을 읽음
    ret, frame = cap.read()
    
    if not ret:
        print("프레임을 읽을 수 없습니다.")
        break
    
    # 그레이스케일 변환 (얼굴 검출을 위해 필요)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # 얼굴 검출
    faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

    # 얼굴 주위에 사각형 그리기
    for (x, y, w, h) in faces:
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

    # 결과 이미지 화면에 표시
    cv2.imshow('Face Detection', frame)

    # 'q'를 누르면 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 카메라 리소스 해제
cap.release()
cv2.destroyAllWindows()
