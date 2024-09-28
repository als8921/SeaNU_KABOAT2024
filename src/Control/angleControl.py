#!/usr/bin/env python
import time
import rospy
from std_msgs.msg import Float32, Int16MultiArray

# PD 제어를 위한 설정 값
class SETTINGS:
    Kp = 1.0  # 비례 게인
    Kd = 0.1  # 미분 게인
    maxSaturation = 255  # PWM 출력 최대값 제한

# 초기 변수 설정
desired_heading = 0.0
last_psi_error = 0.0
tauX = 0.0  # 상수로 가정 (전진 힘 또는 토크)

# 컨트롤 퍼블리셔
control_publish = rospy.Publisher('/control', Int16MultiArray, queue_size=10)
heading = 0
# 헤딩 토픽 콜백 함수
def heading_callback(msg):
    global heading
    heading = msg.data

# 노드 초기화 및 구독
def main():
    rospy.init_node('heading_controller')

    # /KABOAT/Heading 토픽에서 현재 헤딩 값을 받아옴
    rospy.Subscriber('/KABOAT/Heading', Float32, heading_callback)

    # 원하는 헤딩 값 설정
    global desired_heading
    desired_heading = rospy.get_param('~desired_heading', 0.0)

    # 루프 실행
    while(True):
        global last_psi_error

        # 현재 헤딩 값 받아오기
        current_heading = heading

        # 헤딩 오차 계산
        psi_error = desired_heading - current_heading

        # PD 제어를 위한 미분 항 계산
        derivative = psi_error - last_psi_error

        # 토크(tauN) 계산: PD 제어 적용
        tauN = SETTINGS.Kp * psi_error + SETTINGS.Kd * derivative

        # PWM 값 계산
        pwmL = tauX + tauN * 0.5
        pwmR = tauX - tauN * 0.5
        pwmF = -tauN

        # Int16MultiArray로 제어 값을 퍼블리시할 준비
        control_values = Int16MultiArray(data=[0, 0, 0])
        control_values.data[0] = max(-SETTINGS.maxSaturation, min(SETTINGS.maxSaturation, int(pwmL)))
        control_values.data[1] = max(-SETTINGS.maxSaturation, min(SETTINGS.maxSaturation, int(pwmR)))
        control_values.data[2] = max(-SETTINGS.maxSaturation, min(SETTINGS.maxSaturation, int(pwmF)))

        # 제어 신호가 0일 경우
        if psi_error == 0 and tauX == 0:
            control_values.data = [0, 0, 0]

        # 제어 값을 퍼블리시
        control_publish.publish(control_values)
        print(control_values.data)

        # psi_error를 업데이트
        last_psi_error = psi_error
        time.sleep(0.1)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
