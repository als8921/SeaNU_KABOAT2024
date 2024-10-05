#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# 각 색상에 대한 HSV 범위 설정
color_ranges = {
    "Red": ([0, 75, 94], [18, 215, 234]),    # 빨강
    "Blue": ([99, 132, 97], [129, 255, 237]),   # 파랑
    "Black": ([111, 0, 0], [141, 104, 108]),     # 검정
    "Green": ([40, 70, 70], [80, 255, 255]),  # 초록
    "White": ([75, 0, 185], [105, 74, 255]),     # 흰색
    "Orange":([1, 90, 132], [31, 230, 255])
}

# 각 색상 범위를 NumPy 배열로 변환
color_ranges = {key: [np.array(lower), np.array(upper)] for key, (lower, upper) in color_ranges.items()}

# CvBridge와 전역 변수 설정
bridge = CvBridge()

# 도형 판별 함수
def detect_shape(vertices):
    if vertices == 3:
        return "Triangle"
    elif vertices == 4:
        return "Square"
    elif 5 <= vertices <= 8:
        return "Circle"
    elif vertices >= 9:
        return "Cross"
    return "Other"

# 색상과 도형을 탐지하고 X 좌표를 확인하는 함수
def process_image(image, input_color, input_shape):
    # 이미지의 해상도
    height, width, _ = image.shape

    # 상단 1/2 영역만을 ROI로 설정
    roi_end = height // 2
    roi = image[:roi_end, :]  # 상단 1/2 부분

    # BGR을 HSV로 변환
    hsv_image = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    # 입력한 색상의 HSV 범위 가져오기
    lower_hsv, upper_hsv = color_ranges[input_color]

    # 색상에 맞는 마스크 생성
    mask = cv2.inRange(hsv_image, lower_hsv, upper_hsv)

    # 그레이스케일로 변환
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

    # 잡음 제거를 위한 블러 처리
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    # 적응형 이진화 적용
    thresh = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                   cv2.THRESH_BINARY_INV, 11, 2)

    # 엣지 검출
    edges = cv2.Canny(thresh, 50, 150)

    # 컨투어 및 계층 정보 찾기
    contours, hierarchy = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    detected_x_coordinate = None  # 탐지된 X 좌표를 저장할 변수

    # 도형 탐지 로직
    for i, contour in enumerate(contours):
        # 면적이 너무 작은 도형은 무시 (잡음 제거를 위함)
        if cv2.contourArea(contour) < 200:
            continue

        # 부모 컨투어가 없는 경우만 처리 (내부 도형 무시)
        if hierarchy[0][i][3] == -1:  # 부모 컨투어가 없는 경우에만 처리
            # 도형의 꼭짓점 검출 및 면적 계산
            epsilon = 0.04 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            vertices_count = len(approx)
            shape = detect_shape(vertices_count)

            if shape == input_shape:  # 입력한 도형과 일치하는 경우
                M = cv2.moments(contour)
                if M["m00"] > 0:
                    # X 좌표 계산
                    cX = int(M["m10"] / M["m00"])  # 무게 중심의 X좌표

                    # 탐지된 X 좌표를 저장
                    detected_x_coordinate = cX

                    # 도형의 위치에 맞게 좌표 조정
                    x, y, w, h = cv2.boundingRect(contour)
                    cv2.drawContours(roi, [contour], -1, (0, 255, 0), 2)
                    
                    # X 좌표 텍스트를 화면에 출력
                    label = f"X: {cX}"
                    cv2.putText(roi, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                    # 첫 번째로 발견된 도형만 반환하고 함수 종료
                    return detected_x_coordinate

    # 그레이스케일 처리된 이미지와 컨투어 탐지된 이미지 결과 출력 (이전 기능 유지)
    cv2.imshow("Shape Detection", roi)

    return None  # 도형이 탐지되지 않은 경우


# 이미지 콜백 함수
def image_callback(data):
    global cv_image
    try:
        # ROS 이미지 메시지를 OpenCV 이미지로 변환
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except Exception as e:
        print("Error in image conversion:", e)
        return

    # 입력한 색상과 도형 설정
    input_color = 'Orange'  # 탐지할 색상
    input_shape = 'Circle'  # 탐지할 도형

    # 색상과 도형 탐지 및 결과 처리
    section = process_image(cv_image, input_color, input_shape)

    if section:
        print(f"Detected {input_color} {input_shape} in section {section}")
    else:
        print(f"No {input_color} {input_shape} detected.")

    # OpenCV 창에 이미지 출력
    cv2.imshow("Color and Shape Detection", cv_image)
    cv2.waitKey(1)  # 영상 출력을 위해 반드시 필요


# 이미지 콜백 함수
# def image_callback(data):
    try:
        # ROS 이미지 메시지를 OpenCV 이미지로 변환
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except Exception as e:
        rospy.logerr(f"Error in image conversion: {e}")
        return

    # 입력한 색상과 도형 설정
    input_color = 'Green'  # 탐지할 색상
    input_shape = 'Circle'  # 탐지할 도형

    # 색상과 도형 탐지 및 결과 처리
    x_coordinate = process_image(cv_image, input_color, input_shape)

    if x_coordinate is not None:
        # 탐지된 X 좌표를 ROS 로그에 출력
        rospy.loginfo(f"Detected {input_color} {input_shape} at X coordinate: {x_coordinate}")
    else:
        rospy.loginfo(f"No {input_color} {input_shape} detected.")

# 메인 함수
def main():
    # ROS 노드 초기화
    rospy.init_node('color_shape_detection_node', anonymous=True)

    # ROS 토픽 구독 설정
    rospy.Subscriber("/usb_cam/image_raw", Image, image_callback)

    # ROS가 종료될 때까지 대기
    rospy.spin()

    # 모든 창을 닫고 종료
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
