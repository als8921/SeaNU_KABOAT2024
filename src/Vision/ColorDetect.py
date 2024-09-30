#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# 각 색상에 대한 HSV 범위 설정
color_ranges = {
    "Red": ([0, 75, 94], [18, 215, 234]),    # 빨강
    "Blue": ([99, 132, 97], [129, 255, 237]),   # 파랑
    "Black": ([111, 0, 0], [141, 104, 108]),     # 검정
    "White": ([75, 0, 185], [105, 74, 255])     # white
}

# 각 색상 범위를 NumPy 배열로 변환
color_ranges = {key: [np.array(lower), np.array(upper)] for key, (lower, upper) in color_ranges.items()}

# CvBridge와 전역 변수 설정
bridge = CvBridge()
cv_image = None

# HSV 범위에 따라 색상 인식 함수
def detect_color(hsv_pixel):
    for color_name, (lower_hsv, upper_hsv) in color_ranges.items():
        # 픽셀 값이 해당 색상의 범위에 있는지 확인
        if all(lower_hsv <= hsv_pixel) and all(hsv_pixel <= upper_hsv):
            return color_name
    return "Unknown"

# 마우스 클릭 이벤트 콜백 함수
def click_event(event, x, y, flags, param):
    global cv_image
    if event == cv2.EVENT_LBUTTONDOWN and cv_image is not None:  # 왼쪽 버튼 클릭 시
        # 클릭한 좌표의 색상 값 가져오기 (BGR 형식)
        pixel = cv_image[y, x]
        
        # BGR을 HSV로 변환
        hsv_pixel = cv2.cvtColor(np.uint8([[pixel]]), cv2.COLOR_BGR2HSV)[0][0]
        
        # 터미널에 출력 (해당 색상 이름 표시)
        detected_color = detect_color(hsv_pixel)
        print(f"Detected Color at ({x}, {y}): {detected_color}, HSV: {hsv_pixel}")

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
    rospy.init_node('color_detector_node', anonymous=True)

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
