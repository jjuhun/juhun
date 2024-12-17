import cv2 as cv
import numpy as np
import threading
import time
import SDcar
import sys
import tensorflow as tf
from tensorflow.keras.models import load_model

speed = 80
epsilon = 0.0001


def func_thread():
    i = 0
    while True:
        #print("alive!!")    
        time.sleep(1)
        i = i+1
        if is_running is False:
            break

class_names = []
with open('/home/juhun/juhun/Ai_Car_Project/object/object_detection_classes_coco.txt', 'r') as f:
    class_names = f.read().split('\n')

COLORS = np.random.uniform(0, 255, size=(len(class_names), 3))
# Load the object detection model
models = cv.dnn.readNetFromTensorflow(model='/home/juhun/juhun/Ai_Car_Project/object/frozen_inference_graph.pb',\
                        config='/home/juhun/juhun/Ai_Car_Project/object/ssd_mobilenet_v2_coco_2018_03_29.pbtxt')

def handle_object_detected():
    """
    물체가 감지되면 차량을 멈추고, LED와 부저를 제어하는 함수
    """
    # 차량 멈추기
    # stop_mode_led() 호출하여 차량의 정지 상태 LED를 켬
    car.stop_mode_led()
    # 부저 소리 울리기 (1초 동안 부저가 울리도록 설정)
    car.buzzer()  # 부저를 1초간 울리도록 설정 (부저 관련 함수는 SDcar에 따라 달라질 수 있음)
    car.motor_stop()

def detect_objects(image_shape):
    """
    물체 감지 함수
    """
    if image_shape is None:
        print("이미지를 로드할 수 없습니다.")
        return False

    global object_detected

    blob = cv.dnn.blobFromImage(image=image_shape, size=(300, 300), swapRB=True)
    models.setInput(blob)
    output = models.forward()
    for detection in output[0, 0, :, :]:
        confidence = detection[2]
        if confidence > 0.4:
            class_id = int(detection[1])
            class_name = class_names[class_id - 1]
            if class_name == 'fire hydrant':  # 예시로 'fire hydrant'만 감지
                print("find_object")
                car.motor_stop()
    # stop_mode_led() 호출하여 차량의 정지 상태 LED를 켬
                car.stop_mode_led()
    # 부저 소리 울리기 (1초 동안 부저가 울리도록 설정)
                car.buzzer()  
                object_detected = True
                # 물체가 감지되었으면 True 반환
    object_detected = False  # 물체가 감지되지 않으면 False로 설정
     # 물체가 감지되지 않으면 False 반환


def key_cmd(which_key):
    global enable_AIdrive
    is_exit = False
    if which_key & 0xFF == 82:  # Forward
        print('which_key', which_key)
        car.motor_go(speed)
        car.go_mode_led()
    elif which_key & 0xFF == 84:  # Backward
        print('which_key', which_key)
        car.motor_back(speed)
    elif which_key & 0xFF == 81:  # Left
        print('which_key', which_key)
        car.motor_left(speed)
        car.left_mode_led()
    elif which_key & 0xFF == 83:  # Right
        print('which_key', which_key)
        car.motor_right(speed)
        car.right_mode_led()
    elif which_key & 0xFF == 32:  # Stop
        print('which_key', which_key)
        car.motor_stop()
        enable_AIdrive = False
    elif which_key & 0xFF == ord('q'):  # Quit
        print('which_key', which_key)
        car.motor_stop()
        print('exit')
        enable_AIdrive = False
        is_exit = True
    elif which_key & 0xFF == ord('e'):  # Enable AI
        print('which_key', which_key)
        enable_AIdrive = True
    elif which_key & 0xFF == ord('w'):  # Disable AI
        print('which_key', which_key)
        enable_AIdrive = False
        car.motor_stop()
    return is_exit

def detect_maskY_BGR(frame):
    B = frame[:,:,0]
    G = frame[:,:,1]
    R = frame[:,:,2]
    Y = np.zeros_like(G, np.uint8)
    # need to tune params
    Y = G*0.5 + R*0.5 - B*0.7 # 연산 수행 시 float64로 바뀜
    Y = Y.astype(np.uint8)
    Y = cv.GaussianBlur(Y, (5,5), cv.BORDER_DEFAULT)
    # need to tune params
    _, mask_Y = cv.threshold(Y, 100, 255, cv.THRESH_BINARY)
    return mask_Y

def line_tracing(cx):
    #print('cx, ', cx)
    #print('v_x_grid', v_x_grid)
    global moment
    global v_x
    tolerance = 0.1
    diff = 0

    if moment[0] != 0 and moment[1] != 0 and moment[2] != 0:
        avg_m = np.mean(moment)
        diff = np.abs(avg_m - cx) / v_x
    
    #print('diff ={:.4f}'.format(diff))

    if diff <= tolerance:

        moment[0] = moment[1]
        moment[1] = moment[2]
        moment[2] = cx
        print('cx : ', cx)
        if v_x_grid[2] <= cx < v_x_grid[4]:
            car.motor_go(speed) 
            print('go')
        elif v_x_grid[3] >= cx:
            car.motor_left(30) 
            print('turn left')
        elif v_x_grid[1] <= cx:
            car.motor_right(30) 
            print('turn right')
        else:
            print("skip")    
        
    else:
        car.motor_go(speed) 
        print('go')    
        moment = [0,0,0]

def show_grid(img):
    h, _, _ = img.shape
    for x in v_x_grid:
        #print('show_grid', x)
        cv.line(img, (x, 0), (x, h), (0,255,0), 1, cv.LINE_4)

def test_fun(model):
    camera = cv.VideoCapture(0)
    camera.set(cv.CAP_PROP_FRAME_WIDTH,v_x) 
    camera.set(cv.CAP_PROP_FRAME_HEIGHT,v_y)
    ret, frame = camera.read()
    frame = cv.flip(frame,-1)
    # cv.imshow('camera',frame)
    crop_img = frame[int(v_y/2):,:]
    crop_img = cv.resize(crop_img, (200, 66))
    crop_img = np.expand_dims(crop_img, 0)
    a = model.predict(crop_img)
    print('okey, a: ', a)

def detect_objects_th(image_shape):
    """
    물체 감지 및 0.3초 대기를 위한 스레드 함수
    """
    # 이 함수에서 물체 감지 실행
    detect_objects(image_shape)
    
    # 0.3초 동안 대기
    time.sleep(1)

def drive_AI(image):
    img = np.expand_dims(image, 0)
    res = model.predict(img)[0]
    steering_angle = np.argmax(np.array(res))
    if steering_angle == 0:  # Go
        speedSet = 60
        car.motor_go(speedSet)
        car.go_mode_led()
    elif steering_angle == 1:  # Left
        speedSet = 20
        car.motor_left(speedSet)
        car.left_mode_led()
    elif steering_angle == 2:  # Right
        speedSet = 20
        car.motor_right(speedSet)
        car.right_mode_led()

def main():
    global object_detected

    camera = cv.VideoCapture(0)
    camera.set(cv.CAP_PROP_FRAME_WIDTH,v_x) 
    camera.set(cv.CAP_PROP_FRAME_HEIGHT,v_y)

    try:
        while(camera.isOpened()):
            ret, frame = camera.read()
            frame = frame.reshape((240, 320, 3))
            frame = cv.flip(frame,-1)
            crop_img = frame[int(v_y/2):,:]  # 하단 절반만 자르기
            
            crop_img = cv.resize(crop_img, (200, 66))
            cv.imshow('crop_img ', cv.resize(crop_img, dsize=(0,0), fx=2, fy=2))
            
            
            # if enable_AIdrive:  # 물체가 없으면 AI 자율주행 계속
            #     drive_AI(crop_img)
            if enable_AIdrive:  # 일단 자율주행
                # cv.destroyAllWindows()
                drive_AI(crop_img)
                current_time = time.time()
                if current_time % 1 < 0.1:
                    detect_objects(frame)

                if not object_detected: #만약에 아니면 다시 진행
                    drive_AI(crop_img)
                elif object_detected :  # 맞으면 멈춰
                        car.motor_stop()
                        car.stop_mode_led()
                        car.buzzer()

            
            # if object_detected:
            #     handle_object_detected()
            # elif enable_AIdrive:  # 물체가 없으면 AI 자율주행 계속
            #     detect_objects(frame)  # 물체 감지 수행
            #     if not object_detected:
            #         drive_AI(crop_img)
            
            # detect_objects(frame)
            # 물체 감지 후 필요한 처리를 추가 가능 (예: 차량 멈추기)
            # 키 입력 대기
            is_exit = False
            which_key = cv.waitKey(20)
            if which_key > 0:
                is_exit = key_cmd(which_key)
            if is_exit:
                cv.destroyAllWindows()
                break
        time.sleep(0.1)        
    except Exception as e:
        exception_type, exception_object, exception_traceback = sys.exc_info()
        filename = exception_traceback.tb_frame.f_code.co_filename
        line_number = exception_traceback.tb_lineno
        print("Exception type: ", exception_type)
        print("File name: ", filename)
        print("Line number: ", line_number)
        global is_running
        is_running = False

if __name__ == '__main__':
    v_x = 320
    v_y = 240
    v_x_grid = [int(v_x*i/10) for i in range(1, 10)]
    print(v_x_grid)
    moment = np.array([0, 0, 0])

    model_path = '/home/juhun/juhun/Ai_Car_Project/models/lane_navigation_20241210_1313.h5'
    
    model = load_model(model_path)
    
    
    t_task1 = threading.Thread(target = func_thread)
    t_task1.start()
    
    car = SDcar.Drive()
    
    is_running = True
    enable_AIdrive = False
    object_detected = False
    main()
    is_running = False
    car.clean_GPIO()
    print('End of program')
