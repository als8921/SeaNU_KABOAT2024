#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np

# 각 색상에 대한 HSV 범위 설정
color_ranges = {
    "Red": ([0, 75, 94], [18, 215, 234]),    # 빨강
    "Blue": ([99, 132, 97], [129, 255, 237]),   # 파랑
    "Black": ([111, 0, 0], [141, 104, 108]),     # 검정
    "White": ([75, 0, 185], [105, 74, 255])     # 흰색
}

# 각 색상 범위를 NumPy 배열로 변환
color_ranges = {key: [np.array(lower), np.array(upper)] for key, (lower, upper) in color_ranges.items()}

# HSV 범위에 따라 색상 인식 함수
def detect_color(hsv_pixel):
    for color_name, (lower_hsv, upper_hsv) in color_ranges.items():
        if all(lower_hsv <= hsv_pixel) and all(hsv_pixel <= upper_hsv):
            return color_name
    return "Unknown"

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

# 색상과 도형을 탐지하고 위치를 확인하는 함수
def process_image(image, input_color, input_shape):
    # 이미지의 ROI 설정
    height, width, _ = image.shape
    section_width = width // 3  # 화면 가로 기준 3등분

    # ROI(이미지의 상단 2/3 부분만 사용)
    roi_start = height * 1 // 3
    roi_end = height * 2 // 3
    roi = image[:roi_end, :]  # 상단 2/3 부분 사용

    # BGR을 HSV로 변환
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

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

    section_detected = None  # 탐지된 섹션을 저장할 변수

    # 기존 도형 탐지 로직
    for i, contour in enumerate(contours):
        # 면적이 너무 작은 도형은 무시 (잡음 제거를 위함)
        if cv2.contourArea(contour) < 100:
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
                    cX = int(M["m10"] / M["m00"])  # 무게 중심의 X좌표

                    # 가로 기준 3등분하여 물체의 위치 확인
                    if cX < section_width:
                        section_detected = 1
                    elif cX < 2 * section_width:
                        section_detected = 2
                    else:
                        section_detected = 3

                    # ROI 내부에서 도형의 위치에 맞게 좌표 조정
                    x, y, w, h = cv2.boundingRect(contour)
                    cv2.drawContours(roi, [contour], -1, (0, 255, 0), 2)
                    label = f"{input_color} {input_shape}, Section: {section_detected}"
                    cv2.putText(roi, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # 그레이스케일 처리된 이미지와 컨투어 탐지된 이미지 결과 출력 (이전 기능 유지)
    cv2.imshow("Shape Detection", roi)

    return section_detected  # 탐지된 섹션 반환

# 메인 함수
def main():
    # 웹캠 열기
    cap = cv2.VideoCapture(0)

    # 입력한 색상과 도형 설정
    input_color = 'Blue'  # 탐지할 색상
    input_shape = 'Circle'  # 탐지할 도형

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # 색상과 도형 탐지 및 결과 처리
        section = process_image(frame, input_color, input_shape)

        if section:
            # 탐지된 섹션을 cmd 창에 출력 (추가된 기능)
            print(f"Detected {input_color} {input_shape} in section {section}")
        else:
            print(f"No {input_color} {input_shape} detected.")

        # 화면에 결과 표시 (원래 기능 유지)
        cv2.imshow("Color and Shape Detection", frame)

        # 'q' 키를 누르면 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main() 