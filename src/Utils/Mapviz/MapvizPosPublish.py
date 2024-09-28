#!/usr/bin/python3

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
import tf
import math

gps_x = 0.0
gps_y = 0.0

def gps_callback(gps_msg):
    global gps_x, gps_y
    # GPS 데이터에서 위치 정보를 가져옵니다.
    gps_x = gps_msg.longitude  # 위도를 y로 사용
    gps_y = gps_msg.latitude   # 경도를 x로 사용

def heading_callback(msg):
    """헤딩 데이터를 처리하여 PoseStamped 메시지를 생성하는 콜백 함수"""
    pose = create_pose_stamped(msg.data)
    pose_pub.publish(pose)

def desired_heading_callback(msg):
    """원하는 헤딩 데이터를 처리하여 PoseStamped 메시지를 생성하는 콜백 함수"""
    pose = create_pose_stamped(msg.data)
    desired_pose_pub.publish(pose)

def create_pose_stamped(heading):
    """헤딩에 따라 PoseStamped 메시지를 생성하는 함수"""
    pose = PoseStamped()
    pose.header.frame_id = "/wgs84"  # GPS 데이터의 기준 프레임
    pose.header.stamp = rospy.Time.now()
    
    # PoseStamped 메시지의 위치 설정
    pose.pose.position.x = gps_x  # 기준 GPS로부터의 X 거리
    pose.pose.position.y = gps_y  # 기준 GPS로부터의 Y 거리
    pose.pose.position.z = 0.0  # 필요에 따라 조정

    # Heading을 도(degree)에서 라디안으로 변환
    heading_rad = math.radians(90 - heading)

    # Heading을 쿼터니언으로 변환
    quaternion = tf.transformations.quaternion_from_euler(0, 0, heading_rad)

    # PoseStamped 메시지의 orientation 값 설정
    pose.pose.orientation.x = quaternion[0]
    pose.pose.orientation.y = quaternion[1]
    pose.pose.orientation.z = quaternion[2]
    pose.pose.orientation.w = quaternion[3]

    return pose

if __name__ == '__main__':
    rospy.init_node('heading_to_pose_publisher')

    # GPS 데이터 수신을 위한 구독자
    rospy.Subscriber('/ublox_gps/fix', NavSatFix, gps_callback)
    # 헤딩 데이터 수신을 위한 구독자
    rospy.Subscriber('/KABOAT/Heading', Float32, heading_callback)
    # 원하는 헤딩 데이터 수신을 위한 구독자
    rospy.Subscriber('/desired_Heading', Float32, desired_heading_callback)

    # PoseStamped 메시지를 publish 하기 위한 퍼블리셔
    pose_pub = rospy.Publisher('/pose_stamped', PoseStamped, queue_size=10)
    # 원하는 PoseStamped 메시지를 publish 하기 위한 퍼블리셔
    desired_pose_pub = rospy.Publisher('/desired_pose_stamped', PoseStamped, queue_size=10)

    rospy.spin()  # 노드가 종료될 때까지 대기
