#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
import math

def publisher():
    # ROS 노드 초기화
    rospy.init_node('sine_wave_publisher', anonymous=True)
    pub = rospy.Publisher('/command', Float32MultiArray, queue_size=10)
    
    rate = rospy.Rate(10)  # 10Hz로 퍼블리시
    t = 0.0  # 시간 변수 초기화
    while not rospy.is_shutdown():
        # 사인파 값 계산
        a = 100 * math.sin(t)
        b = math.sin(t + math.pi / 2)  # 90도 위상차
        
        # Float32MultiArray 메시지 생성
        msg = Float32MultiArray()
        msg.data = [a, 0]
        if(t > 30):
            msg.data = [0, 0]
        # 퍼블리시
        pub.publish(msg)
        rospy.loginfo("Published: [%f, %f]", a, 0)
        
        t += 0.1  # 시간 증가
        rate.sleep()  # 주기 유지

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
