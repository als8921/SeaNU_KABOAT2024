# -*- coding:utf-8 -*-
#!/usr/bin/env python3
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
import SETTINGS
import rospy
from std_msgs.msg import Float64MultiArray, Float32
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Polygon
import message_filters
import tkinter as tk
from geometry_msgs.msg import PointStamped
import Control.AutonomousModule as AutonomousModule
import math


# 초기 거리 데이터 설정
distances = [0] * 360
angles = np.radians(np.arange(360))

# 위치 및 방위각 변수
gps_position = [0.0, 0.0]  # [x, y] 좌표
heading_angle = 0.0  # 방위각

ref_position = [30, 15]
visual_size = 40

# 클릭된 좌표 저장 리스트
clicked_points = []
desired_heading=0
cost_x_List=[]
cost_y_List=[]
field_points=[[-5,-5],[-5,24],[60,24],[60,-5]]
# 시각화 함수
def update(frame):
    global distances, gps_position, heading_angle,cost_x_List,cost_y_List,desired_heading
    
    # 평면 그래프 업데이트
    ax.clear()
    ax.set_title('Distance Data in Cartesian Coordinates')
    ax.set_xlim(ref_position[0] - visual_size ,ref_position[0] + visual_size)  # x축 범위 설정
    ax.set_ylim(ref_position[1] - visual_size ,ref_position[1] + visual_size)  # y축 범위 설정

    # 라이다 데이터의 Cartesian 좌표 변환
    x_positions = distances * np.sin(angles + np.radians(heading_angle)) + gps_position[0]
    y_positions = distances * np.cos(angles + np.radians(heading_angle)) + gps_position[1]

    # 라이다 데이터 시각화
    ax.scatter(x_positions, y_positions, color='blue', s=2)  # 거리 데이터 점으로 표시

    # GPS 위치 표시
    ax.scatter(gps_position[0], gps_position[1], color='red', s=50, label='GPS Position')
    
    # IMU 각도에 따라 초록색 직선 그리기
    line_length = 10  # 직선의 길이 설정
    line_x_end = gps_position[0] + line_length * np.sin(np.radians(heading_angle))
    line_y_end = gps_position[1] + line_length * np.cos(np.radians(heading_angle))
    ax.plot([gps_position[0], line_x_end], [gps_position[1], line_y_end], color='green', linewidth=2, label='Heading Direction')

    # 클릭된 좌표들을 빨간 점으로 표시
    if clicked_points:
        clicked_x, clicked_y = zip(*clicked_points)
        ax.scatter(clicked_x, clicked_y, color='red', s=50, label='Clicked Points')
        
    # 4개의 점이 입력된 경우 사각형 그리기
    polygon = Polygon(field_points, closed=True, fill=None, edgecolor='orange')

    try:
        cost_x_List=[]
        cost_y_List=[]
        dx = waypoint.point.x - gps_position[0]
        dy = waypoint.point.y - gps_position[1]


        Goal_Psi = np.arctan2(dx, dy) * 180 / np.pi - heading_angle
        Goal_Psi = AutonomousModule.normalize_angle(Goal_Psi)
        cost=AutonomousModule.Final_cost(distances,AutonomousModule.calculate_safe_zone(distances),Goal_Psi)
        desired_heading=AutonomousModule.calculate_optimal_psi_d(distances,AutonomousModule.calculate_safe_zone(distances),Goal_Psi)
        if heading_angle>0:
            for i in range(-180,180):
                prepared_i=AutonomousModule.normalize_angle(i+180)
                cost_x_List.append([gps_position[0],gps_position[0] + 10/(cost[i]+1) * np.cos(np.radians(prepared_i))])
                cost_y_List.append([gps_position[1],gps_position[1] + 10/(cost[i]+1) * np.sin(np.radians(prepared_i))])
        else:
            for i in range(-180,180):
                prepared_i=AutonomousModule.normalize_angle(-i)
                cost_x_List.append([gps_position[0],gps_position[0] + 10/(cost[i]+1) * np.cos(np.radians(prepared_i))])
                cost_y_List.append([gps_position[1],gps_position[1] + 10/(cost[i]+1) * np.sin(np.radians(prepared_i))])
        print(desired_heading,AutonomousModule.normalize_angle(-desired_heading+90),heading_angle)
        ax.scatter(cost_x_List[AutonomousModule.normalize_angle(desired_heading)], cost_y_List[AutonomousModule.normalize_angle(desired_heading)])
    except:
        pass


    ax.plot(cost_x_List, cost_y_List, color='purple', linewidth=0.5, label='Cost')
    ax.add_patch(polygon)
    ax.grid(True)
    ax.legend()

def distance_callback(data):
    global distances
    distances = data.data  # 수신한 거리 데이터로 업데이트

def gps_callback(data):
    global gps_position
    gps_position = data.data  # GPS 위치 업데이트 (예: [x, y])

def imu_callback(data):
    global heading_angle
    heading_angle = data.data  # 방위각 업데이트

def laser_scan_callback(data):
    global distances, angles
    distances = np.flip(data.ranges)  # 수신한 거리 데이터
    distances = np.concatenate((distances[180:],distances[:180]))
    distances[distances > 100] = 0  # 임계값 초과 시 0으로 변경
    angles = np.radians(np.arange(len(distances)))
def publishWaypoint(x, y, z):
    global waypoint,cost_x_List,cost_y_List
    waypoint=PointStamped()
    waypoint.point.x = x
    waypoint.point.y = y
    waypoint.point.z = z
    Waypont_pub.publish(waypoint)


# 클릭된 좌표 초기화 함수
def reset_points():
    global clicked_points
    clicked_points = []
    print("Clicked points reset!")

# 마우스 클릭 이벤트 핸들러
def on_click(event):
    if event.inaxes:  # 클릭이 축 내부에서 발생한 경우에만 처리
        x, y = event.xdata, event.ydata
        
        # 커스텀 팝업 창 생성
        root = tk.Tk()
        root.withdraw()  # Tkinter 메인 윈도우 숨김
        popup = tk.Toplevel(root)
        popup.title("좌표 확인")
        
        label = tk.Label(popup, text=f"x: {x:.2f}, y: {y:.2f} 의 WayPoint를 Publish하시겠습니까?")
        label.pack(padx=20, pady=20)
        
        # OK 버튼
        ok_button = tk.Button(popup, text="OK", command=lambda: [clicked_points.append((x, y)), publishWaypoint(x, y, 0),  print(f"Clicked at x: {x:.2f}, y: {y:.2f}"), popup.destroy()])
        ok_button.pack(side=tk.LEFT, padx=10)

        # Cancel 버튼
        cancel_button = tk.Button(popup, text="Cancel", command=popup.destroy)
        cancel_button.pack(side=tk.LEFT, padx=10)

        # Reset 버튼
        reset_button = tk.Button(popup, text="Reset", command=lambda: [reset_points(), publishWaypoint(0, 0, -1), popup.destroy()])
        reset_button.pack(side=tk.LEFT, padx=10)
        
        root.mainloop()

def listener():
    rospy.init_node('distance_visualizer', anonymous=True)

    # 메시지 필터 설정
    if SETTINGS.isSimulator:
        lidar_sub = rospy.Subscriber("Lidar", Float64MultiArray, distance_callback)
        gps_sub = message_filters.Subscriber("KABOAT/UTM", Float64MultiArray)
        imu_sub = message_filters.Subscriber("KABOAT/Heading", Float32)
        # imu_sub = message_filters.Subscriber("/android/imu", Float32)

    else:
        lidar_sub = rospy.Subscriber("scan", LaserScan, laser_scan_callback)
        gps_sub = message_filters.Subscriber("KABOAT/UTM", Float64MultiArray)
        imu_sub = message_filters.Subscriber("KABOAT/Heading", Float32)
        # imu_sub = message_filters.Subscriber("/android/imu", Float32)

    # 동기화 설정 (1초 이내의 메시지를 동기화)
    ts = message_filters.ApproximateTimeSynchronizer([gps_sub, imu_sub], queue_size=1000, slop=0.1, allow_headerless=True)
    ts.registerCallback(lambda gps_data, imu_data: (gps_callback(gps_data), imu_callback(imu_data)))

    # Matplotlib 설정
    global fig, ax
    fig, ax = plt.subplots()
    
    # 클릭 이벤트 연결
    cid = fig.canvas.mpl_connect('button_press_event', on_click)

    # 애니메이션 설정
    ani = FuncAnimation(fig, update, interval=10)  # 10ms마다 업데이트
    plt.show()

if __name__ == '__main__':
    try:
        # 시뮬레이터 여부에 따라 listener 호출
        Waypont_pub=rospy.Publisher("/Waypoint",PointStamped)
        listener()  # 'True'로 변경하면 시뮬레이터 모드로 실행
    except rospy.ROSInterruptException:
        pass
