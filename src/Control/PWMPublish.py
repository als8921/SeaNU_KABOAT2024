#!/usr/bin/env python3
import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

import rospy
from std_msgs.msg import Float32MultiArray, Int16MultiArray
import time
import SETTINGS

class MotorController:
    def __init__(self):
        rospy.init_node('motor_controller', anonymous=True)
        self.control_pub = rospy.Publisher('/control', Int16MultiArray, queue_size = 10)
        rospy.Subscriber('/command', Float32MultiArray, self.command_callback)

        self.last_error = 0.0

        self.last_time = time.time()

    def command_callback(self, data):

        psi_error = data.data[0]
        tauX = data.data[1]
        tauN = self.pd_control(psi_error)
        
        control_values = Int16MultiArray(data=[0, 0, 0])
        pwmL = tauX + tauN * 0.5
        pwmR = tauX - tauN * 0.5
        pwmF = -tauN

        if(data.data[2] == 0):
            control_values.data[0] = max(-SETTINGS.maxSaturation, min(SETTINGS.maxSaturation, int(pwmL)))
            control_values.data[1] = max(-SETTINGS.maxSaturation, min(SETTINGS.maxSaturation, int(pwmR)))
            control_values.data[2] = max(-SETTINGS.maxSaturation, min(SETTINGS.maxSaturation, int(pwmF)))
        else:
            control_values.data[0] = int(max(-data.data[2], min(data.data[2], int(pwmL))))
            control_values.data[1] = int(max(-data.data[2], min(data.data[2], int(pwmR))))
            control_values.data[2] = int(max(-data.data[2], min(data.data[2], int(pwmF))))


        if(psi_error == 0 and tauX == 0):
            control_values.data = [0, 0, 0]
            
        self.control_pub.publish(control_values)

    def pd_control(self, error):
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        derivative = (error - self.last_error) / dt if dt > 0 else 0
        self.last_error = error
        
        output = SETTINGS.Kp * error + SETTINGS.Kd * derivative
        return output

if __name__ == '__main__':
    try:
        controller = MotorController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
