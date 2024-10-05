#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# CvBridge와 전역 변수 설정
bridge = CvBridge()
cv_image = None

# HSV 값 출력 콜백 함수
def click_event(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN and cv_image is not None:  # 왼쪽 버튼 클릭 시
        # 클릭한 좌표의 색상 값 가져오기 (BGR 형식)
        pixel = cv_image[y, x]
        
        # BGR을 HSV로 변환
        pixel_hsv = cv2.cvtColor(np.uint8([[pixel]]), cv2.COLOR_BGR2HSV)[0][0]
        h, s, v = pixel_hsv

        # 주황색과 빨간색을 구분하기 위한 조건
        if 0 <= h <= 15 or 160 <= h <= 180:  # 빨간색 범위
            lower_hsv = np.array([max(0, h - 10), max(50, s - 50), max(50, v - 50)])
            upper_hsv = np.array([min(180, h + 10), min(255, s + 50), min(255, v + 50)])
        elif 15 < h <= 30:  # 주황색 범위
            lower_hsv = np.array([max(0, h - 10), max(50, s - 50), max(50, v - 50)])
            upper_hsv = np.array([min(180, h + 10), min(255, s + 50), min(255, v + 50)])
        else:
            # 기본 범위 설정
            lower_hsv = np.array([max(0, h - 15), max(0, s - 70), max(0, v - 70)])
            upper_hsv = np.array([min(180, h + 15), min(255, s + 70), min(255, v + 70)])
        
        # 터미널에 복사 붙여넣기 가능한 형식으로 출력
        print(f"[{lower_hsv[0]}, {lower_hsv[1]}, {lower_hsv[2]}], "
              f"[{upper_hsv[0]}, {upper_hsv[1]}, {upper_hsv[2]}]")
        
        print(f"HSV {pixel_hsv}")


# 이미지 콜백 함수
def img_callback(data):
    global cv_image
    # ROS 이미지 메시지를 OpenCV 이미지로 변환
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except Exception as e:
        print("Error in image conversion:", e)

def main():
    # ROS 노드 초기화
    rospy.init_node('image_processor_node', anonymous=True)

    # 토픽 구독 설정
    rospy.Subscriber("/usb_cam/image_raw", Image, img_callback)

    # ROS가 종료될 때까지 루프
    while not rospy.is_shutdown():
        if cv_image is not None:
            # 원본 이미지를 화면에 출력
            cv2.imshow('Camera Feed', cv_image)

            # 마우스 클릭 이벤트 연결
            cv2.setMouseCallback("Camera Feed", click_event)

            # 'q' 키를 누르면 종료
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            print("Waiting for image data...")

    # 모든 창을 닫고 종료
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
