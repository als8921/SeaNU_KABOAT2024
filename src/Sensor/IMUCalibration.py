#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32

class ImuCalibration:
    def __init__(self):
        self.bias_x = 0.0
        self.bias_y = 0.0
        self.bias_z = 0.0
        self.calibrated = False
        self.samples = []
        self.sample_count = 100  # Number of samples for calibration
        rospy.Subscriber("/handsfree/imu", Imu, self.calibration_callback)
        self.pub = rospy.Publisher("/imu/data_calibrated", Imu, queue_size=10)

    def calibration_callback(self, data):
        if not self.calibrated:
            self.samples.append(data.linear_acceleration)

            if len(self.samples) >= self.sample_count:
                self.calibrate_bias()
                self.calibrated = True
        else:
            print("IMU CALIBRATION DONE")
            data.linear_acceleration.x -= self.bias_x
            data.linear_acceleration.y -= self.bias_y
            data.linear_acceleration.z -= self.bias_z
            self.pub.publish(data)

    def calibrate_bias(self):
        sum_x = sum(sample.x for sample in self.samples)
        sum_y = sum(sample.y for sample in self.samples)
        sum_z = sum(sample.z for sample in self.samples)

        self.bias_x = sum_x / len(self.samples)
        self.bias_y = sum_y / len(self.samples)
        self.bias_z = sum_z / len(self.samples)

        rospy.loginfo("Calibration complete. Biases: x=%f, y=%f, z=%f", self.bias_x, self.bias_y, self.bias_z)


    #imu데이터 수신 -> 콜백 -> 리스트에 수신된 데이터 리스트로 추가 -> 임의의 개수만큼 데이터가 모이면 평균 계산, 위치에서 차감 -> 차감한 데이터 송신

if __name__ == '__main__':
    rospy.init_node('imu_calibration')
    ImuCalibration()
    rospy.spin()