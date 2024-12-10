import cv2 as cv
import numpy as np
import threading, time
import SDcar 
import sys
import os
import datetime

def func_thread():
    i = 0
    while True:
        #print("alive!!")    
        time.sleep(1)
        i = i+1
        if is_running is False:
            break

def save_img(frame, angle):
    global filecnt
    print('filecnt', filecnt)

    filename = 'train_{0:05d}_{1:03d}.png'.format(filecnt, angle)
    filename = os.path.join(filepath, filename)
    #filename = filepath + 'train_{}_{}.png'.format(filecnt, angle)
    print('filename', filename)
    cv.imwrite(filename, frame)
    filecnt+=1
def key_cmd(which_key, frame):
    print('which_key', which_key)
    is_exit = False
    if which_key & 0xFF == 82:  # 직진
        print('up')
        car.motor_go(speed)  # 차를 앞으로 이동
        time.sleep(0.5)  # 0.5초 동안만 실행
        car.motor_stop()  # 차를 멈추기
    elif which_key & 0xFF == 84: # 후진
        print('down')
        car.motor_back(speed)  # 차를 뒤로 이동
        time.sleep(0.5)  # 0.5초 동안만 실행
        car.motor_stop()  # 차를 멈추기
    elif which_key & 0xFF == 81:
        print('left')     
        car.motor_left(speed_rot)  # 차를 왼쪽으로 회전
        car.left_mode_led()
        time.sleep(0.5)  # 0.5초 동안만 실행
        car.motor_stop()  # 차를 멈추기
    elif which_key & 0xFF == 83:
        print('right')   
        car.motor_right(speed_rot)  # 차를 오른쪽으로 회전
        car.right_mode_led()
        time.sleep(0.5)  # 0.5초 동안만 실행
        car.motor_stop()  # 차를 멈추기
    elif which_key & 0xFF == 32:
        car.motor_stop()
        print('stop')   
    elif which_key & 0xFF == ord('q'):  
        car.motor_stop()
        print('exit')
        car.motor_stop()  # 차 멈추기        
        is_exit = True
    elif which_key & 0xFF == ord('x'):  
        save_img(frame, 0)
        print('save forward img')  
    elif which_key & 0xFF == ord('z'):  
        save_img(frame, 1)
        print('save left img')  
    elif which_key & 0xFF == ord('c'):  
        save_img(frame, 2)
        print('save right img')                      
    return is_exit  

def detect_maskY_HSV(frame):
    crop_hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    crop_hsv = cv.GaussianBlur(crop_hsv, (5,5), cv.BORDER_DEFAULT)
    # need to tune params
    mask_Y = cv.inRange(crop_hsv, (25, 50, 100), (35, 255, 255))
    return mask_Y

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

def main():

    camera = cv.VideoCapture(0)

    camera.set(cv.CAP_PROP_FRAME_WIDTH,v_x) 
    camera.set(cv.CAP_PROP_FRAME_HEIGHT,v_y)
    if not camera.isOpened():
       print("Error: Camera could not be opened")
       return  # 카메라가 열리지 않으면 main 함수 종료
    try:
        while( camera.isOpened() ):
            ret, frame = camera.read()
            frame = cv.flip(frame,-1)
            crop_img = frame[int(v_y/2):,:]
            crop_img = cv.resize(crop_img, (200, 66))
            cv.imshow('crop_img',cv.resize(crop_img, (0,0), fx=2, fy=2))

            # image processing start here
            # image processing end here

            is_exit = False
            which_key = cv.waitKey(20)
            if which_key > 0:
                is_exit = key_cmd(which_key, crop_img)    
            if is_exit is True:
                cv.destroyAllWindows()
                break
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
    speed = 30
    speed_rot = 30
    parent_dir = "dataset"
    if not os.path.isdir(parent_dir):
        os.mkdir(parent_dir)
    save_dir = datetime.datetime.now().strftime('%Y%m%d_%H%M')
    #print('save_dir', save_dir)
    filepath = os.path.join(parent_dir, save_dir)
    print('filepath', filepath)
    if not os.path.isdir(filepath):
        os.mkdir(filepath)

    filecnt = 0
    t_task1 = threading.Thread(target = func_thread)
    t_task1.start()

    car = SDcar.Drive()
    
    is_running = True
    main() 
    is_running = False
    car.clean_GPIO()



