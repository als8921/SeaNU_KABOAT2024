#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from std_msgs.msg import Float32  # 퍼블리시할 메시지 타입

input_color = 'Orange'
input_shape = 'Cross'

# 각 색상에 대한 HSV 범위 설정
color_ranges = {
    "Red": ([0, 106, 162], [14, 206, 255]),    # 빨강
    # "Blue": ([110, 158, 81], [140, 255, 221]),   # 파랑
    "Black": ([64, 23, 0], [94, 163, 92]),     # 검정
    "Orange": ([7, 83, 165], [27, 183, 255]),  # 주황
    # "ALL": ([0, 0, 0], [255, 255, 255]),  # 모든 색상
    "Green": ([66, 50, 62], [96, 190, 202])  # 초록
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

# 도형 내부 색상 비율을 확인하는 함수
def check_color_ratio(mask, contour, threshold=0.8):
    contour_area = cv2.contourArea(contour)
    mask_contour = np.zeros(mask.shape, dtype=np.uint8)
    cv2.drawContours(mask_contour, [contour], -1, 255, -1)
    color_check = cv2.bitwise_and(mask, mask_contour)
    color_pixel_count = np.count_nonzero(color_check)
    color_ratio = color_pixel_count / contour_area if contour_area > 0 else 0
    return color_ratio > threshold

# 색상과 도형을 탐지하고 X 좌표를 확인하는 함수
def process_image(image, input_color, input_shape):
    height, width, _ = image.shape
    roi = image[:]  
    hsv_image = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    lower_hsv, upper_hsv = color_ranges[input_color]
    mask = cv2.inRange(hsv_image, lower_hsv, upper_hsv)
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    thresh = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2)
    edges = cv2.Canny(thresh, 50, 150)
    contours, hierarchy = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    detected_x_coordinate = None

    for i, contour in enumerate(contours):
        if cv2.contourArea(contour) < 400:
            continue

        if hierarchy[0][i][3] == -1:
            epsilon = 0.04 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            vertices_count = len(approx)
            shape = detect_shape(vertices_count)

            if check_color_ratio(mask, contour, threshold=0.8):
                if shape == input_shape:
                    M = cv2.moments(contour)
                    if M["m00"] > 0:
                        cX = int(M["m10"] / M["m00"])
                        detected_x_coordinate = cX
                        x, y, w, h = cv2.boundingRect(contour)
                        cv2.drawContours(roi, [contour], -1, (0, 255, 0), 2)
                        label = f"X: {cX}"
                        cv2.putText(roi, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        return detected_x_coordinate

    cv2.imshow("Shape Detection", roi)
    return None

# 이미지 콜백 함수
def image_callback(data):
    global cv_image
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except Exception as e:
        print("Error in image conversion:", e)
        return

    x_coordinate = process_image(cv_image, input_color, input_shape)

    if x_coordinate is not None:
        x_coordinate_float = float(x_coordinate)
        x_pub.publish(x_coordinate_float)  # X 좌표 퍼블리시
        print(f"Detected {input_color} {input_shape} at X coordinate: {x_coordinate_float}")
    else:
        print(f"No {input_color} {input_shape} detected.")

    cv2.imshow("Color and Shape Detection", cv_image)
    cv2.waitKey(1)

# 메인 함수
def main():
    global x_pub
    rospy.init_node('color_shape_detection_node', anonymous=True)

    # X 좌표 퍼블리셔 생성
    x_pub = rospy.Publisher('/detected_x_coordinate', Float32, queue_size=10)

    rospy.Subscriber("/usb_cam/image_raw", Image, image_callback)

    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
