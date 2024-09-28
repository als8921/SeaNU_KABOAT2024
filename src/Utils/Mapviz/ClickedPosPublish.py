#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray
import utm
import RTK.RTKSetting

def create_marker(data, marker_id):
    """새로운 마커를 생성하는 함수"""
    marker = Marker()
    marker.header.frame_id = "wgs84"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "points"
    marker.id = marker_id
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.position.x = data.point.x
    marker.pose.position.y = data.point.y
    marker.pose.position.z = 0
    marker.scale.x = 2
    marker.scale.y = 2
    marker.scale.z = 2
    marker.color.a = 0.8
    marker.color.r = 0.0
    marker.color.g = 0.8
    marker.color.b = 0.0
    return marker

def callback(data):
    """클릭한 포인트를 처리하는 콜백 함수"""
    # 클릭한 포인트를 UTM으로 변환
    utm_x, utm_y, _, _ = utm.from_latlon(data.point.y, data.point.x)

    # 상대 좌표 계산
    relative_x = utm_x - RTK.RTKSetting.ref_utm_x
    relative_y = utm_y - RTK.RTKSetting.ref_utm_y

    # 새로운 PointStamped 생성
    relative_point = PointStamped()
    relative_point.header.stamp = rospy.Time.now()
    relative_point.point.x = relative_x
    relative_point.point.y = relative_y
    relative_point.point.z = 0

    # 마커 개수 체크 및 비우기
    if len(point_list.markers) >= 5:
        # 기존 마커 삭제
        delete_marker = Marker()
        delete_marker.header.frame_id = "wgs84"
        delete_marker.action = Marker.DELETEALL
        
        # 삭제할 마커를 퍼블리시
        pub.publish(MarkerArray(markers=[delete_marker]))
        point_list.markers.clear()  # 리스트 비우기
    else:
        # 새로운 마커 생성 및 추가
        marker = create_marker(data, len(point_list.markers))
        point_list.markers.append(marker)

        # 상대 좌표 퍼블리시
        relative_pub.publish(relative_point)

        # MarkerArray 퍼블리시
        pub.publish(point_list)

if __name__ == "__main__":
    rospy.init_node("Point_Node")
    rospy.Subscriber("/mapviz/clicked_point", PointStamped, callback)

    point_list = MarkerArray()  # MarkerArray 초기화
    pub = rospy.Publisher('/visual_rviz', MarkerArray, queue_size=10)  # 마커 퍼블리셔 생성
    relative_pub = rospy.Publisher('/Waypoint', PointStamped, queue_size=10)  # 상대 좌표 퍼블리셔 생성

    rospy.spin()  # 노드가 종료될 때까지 대기
