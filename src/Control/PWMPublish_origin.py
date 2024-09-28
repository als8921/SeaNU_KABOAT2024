#!/usr/bin/env python3
import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

import rospy
from std_msgs.msg import Float32MultiArray, Float64MultiArray
import time
import SETTINGS

class MotorController:
    def __init__(self):
        rospy.init_node('motor_controller', anonymous=True)
        self.control_pub = rospy.Publisher('/control', Float64MultiArray, queue_size=1)
        self.i = 0.0
        # command 배열을 구독
        rospy.Subscriber('/command', Float32MultiArray, self.command_callback)

        self.last_error = 0.0

        self.last_time = time.time()

    def command_callback(self, data):
        if len(data.data) < 2:
            rospy.logwarn("command 배열의 길이가 충분하지 않습니다.")
            return
        
        psi_error = data.data[0]
        tauX = data.data[1]
        tauN = self.pd_control(psi_error)
        
        control_values = Float64MultiArray(data=[0, 0, 0])
        pwmL = tauX + tauN * 0.5
        pwmR = tauX - tauN * 0.5
        pwmF = -tauN

        control_values.data[0] = max(-SETTINGS.maxSaturation, min(SETTINGS.maxSaturation, int(pwmL)))
        control_values.data[1] = max(-SETTINGS.maxSaturation, min(SETTINGS.maxSaturation, int(pwmR)))
        control_values.data[2] = max(-SETTINGS.maxSaturation, min(SETTINGS.maxSaturation, int(pwmF)))

        if(psi_error == 0 and tauX == 0):
            control_values.data = [0, 0, 0]
            
        self.control_pub.publish(control_values)
        # print(control_values)

    def pd_control(self, error):
        # 현재 시간
        current_time = time.time()
        dt = current_time - self.last_time  # 시간 간격 계산
        self.last_time = current_time  # 현재 시간을 마지막 시간으로 업데이트
        
        # PD 제어 계산
        derivative = (error - self.last_error) / dt if dt > 0 else 0
        self.last_error = error
        
        tauN = SETTINGS.Kp * error + SETTINGS.Kd * derivative
        return tauN

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        controller = MotorController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
