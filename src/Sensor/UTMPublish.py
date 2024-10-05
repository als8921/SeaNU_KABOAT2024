#!/usr/bin/env python3
import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
import rospy
from sensor_msgs.msg import NavSatFix
import utm
from std_msgs.msg import Float64MultiArray
import SETTINGS

ref_x = SETTINGS.ref_utm_x
ref_y = SETTINGS.ref_utm_y

utm_x1 = None
utm_y1 = None
utm_x2 = None
utm_y2 = None

def calculate_center():
    if utm_x1 is not None and utm_y1 is not None and utm_x2 is not None and utm_y2 is not None:
        # 중심 좌표 계산
        center_x = (utm_x1 + utm_x2) / 2
        center_y = (utm_y1 + utm_y2) / 2

        # 상대 좌표 계산
        relative_center_x = center_x - ref_x
        relative_center_y = center_y - ref_y

        UTMdata = Float64MultiArray()
        UTMdata.data = [relative_center_x, relative_center_y]  # 상대 중심 좌표 저장
        print(UTMdata.data)
        pub.publish(UTMdata)

def callback1(data):
    global utm_x1, utm_y1
    utm_x1, utm_y1, _, _ = utm.from_latlon(data.latitude, data.longitude)
    calculate_center()  # 중심 좌표 계산 호출

def callback2(data):
    global utm_x2, utm_y2
    utm_x2, utm_y2, _, _ = utm.from_latlon(data.latitude, data.longitude)
    calculate_center()  # 중심 좌표 계산 호출

def listener():
    rospy.init_node('GPStoUTM_Node', anonymous=True)
    sub1 = rospy.Subscriber('/smc_2000/fix', NavSatFix, callback1)
    sub2 = rospy.Subscriber('/smc_plus/fix', NavSatFix, callback2)
    rospy.spin()

if __name__ == '__main__':
    pub = rospy.Publisher('KABOAT/UTM', Float64MultiArray, queue_size=10)
    listener()
