import cv2
import numpy as np
from matplotlib import pyplot as plt

# 이미지 경로들
image_paths = [
    '/home/juhun/juhunnote/juhun/week10/line/imgs/1.jpg',
    '/home/juhun/juhunnote/juhun/week10/line/imgs/2.jpg',           
    '/home/juhun/juhunnote/juhun/week10/line/imgs/3.jpg',           
    '/home/juhun/juhunnote/juhun/week10/line/imgs/4.jpg'                          
]

def linedetection(image_path):
    image = cv2.imread(image_path)
    if image is None:
        print(f"Error: Could not load image at {image_path}")
        return
    
    # 이미지 크기 조정 (비율 설정)
    scale_percent = 60
    width = int(image.shape[1] * scale_percent / 100)
    height = int(image.shape[0] * scale_percent / 100)
    resized_image = cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA)

    # HSV로 변환
    hsv = cv2.cvtColor(resized_image, cv2.COLOR_BGR2HSV)

    # 노란색과 흰색 범위 정의
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([30, 255, 255])

    lower_white = np.array([0, 0, 180])  # 흰색 범위 조정
    upper_white = np.array([180, 25, 255])

    # 마스크 생성
    yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    white_mask = cv2.inRange(hsv, lower_white, upper_white)

    # 두 마스크 결합
    combined_mask = cv2.bitwise_or(yellow_mask, white_mask)

    # 마스크 적용
    extracted_lines = cv2.bitwise_and(resized_image, resized_image, mask=combined_mask)

    # 그레이스케일로 변환 후 이진화
    gray_lines = cv2.cvtColor(extracted_lines, cv2.COLOR_BGR2GRAY)
    _, binary_lines = cv2.threshold(gray_lines, 1, 255, cv2.THRESH_BINARY)

    # 결과 출력
    plt.figure(figsize=(15, 5))
    plt.subplot(1, 2, 1)
    plt.title('Original Image (Resized)')
    plt.imshow(cv2.cvtColor(resized_image, cv2.COLOR_BGR2RGB))

    plt.subplot(1, 2, 2)
    plt.title('Extracted Lines (Yellow and White)')
    plt.imshow(binary_lines, cmap='gray')
    plt.show()

# 첫 번째 이미지 경로를 사용하여 라인 검출
for image_path in image_paths:
    linedetection(image_path)  # 인덱스 1번째 이미지를 사용

cv2.destroyAllWindows()
