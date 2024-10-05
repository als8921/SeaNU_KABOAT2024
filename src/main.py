#!/usr/bin/env python3
import SETTINGS
import time, math
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64MultiArray, Float32, Float32MultiArray, Int16MultiArray
import message_filters
import Control.AutonomousModule as AutonomousModule





class Boat:
    def __init__(self):
        self.position = [10000000, 10000000]
        self.psi = 0
        self.scan = [0] * 360

class CallBack:
    threshold = 100.0  # 특정 거리 임계값 설정 (예: 100.0 센티미터)
    @staticmethod
    def laser_scan_callback(data):
        distances = np.flip(data.ranges)
        distances = np.concatenate((distances[180:],distances[:180]))
        distances[distances > CallBack.threshold] = 0
        boat.scan = distances

    @staticmethod
    def simulator_laser_scan_callback(data):
        distances = np.array(data.data)  # 수신한 거리 데이터
        distances[distances > CallBack.threshold] = 0  # 임계값 초과 시 0으로 변경
        boat.scan = distances

    @staticmethod
    def gps_callback(data):
        boat.position = data.data

    @staticmethod
    def imu_callback(data):
        boat.psi = data.data

def loginfoOnce(s):
    global pastPrint
    if(pastPrint != s):
        rospy.loginfo(s)
        pastPrint = s

def ros_init():
    global command_publish, waypoint_publish
    rospy.init_node('main_node', anonymous=True)

    command_publish = rospy.Publisher('/command', Float32MultiArray, queue_size=10)
    waypoint_publish = rospy.Publisher('waypoint', Float32MultiArray, queue_size=10)

    if SETTINGS.isSimulator:
        rospy.Subscriber("Lidar", Float64MultiArray, CallBack.simulator_laser_scan_callback)
    else:
        rospy.Subscriber("scan", LaserScan, CallBack.laser_scan_callback)

    gps_sub = message_filters.Subscriber("KABOAT/UTM", Float64MultiArray)
    imu_sub = message_filters.Subscriber("KABOAT/Heading", Float32)

    ts = message_filters.ApproximateTimeSynchronizer([gps_sub, imu_sub], queue_size=10, slop=0.1, allow_headerless=True)
    ts.registerCallback(lambda gps_data, imu_data: (CallBack.gps_callback(gps_data), CallBack.imu_callback(imu_data)))

def Autonomous(goal_x,goal_y, goal_range = SETTINGS.GoalRange, maxSaturation = SETTINGS.maxSaturation):
    rate = rospy.Rate(10)
    rospy.loginfo(f"[{goal_x}, {goal_y}]  AUTONOMOUS MODE START")
    while not rospy.is_shutdown():
        loginfoOnce(f"[Autonomous Mode] [{goal_x:.2f}, {goal_y:.2f}] Start")
        

        path = AutonomousModule.pathplan(boat, goal_x, goal_y)

        if AutonomousModule.goal_passed(boat, goal_x, goal_y, goal_range):

            command_publish.publish(Float32MultiArray(data=[0, 0, 0]))
            waypoint_publish.publish(Float32MultiArray(data = [0, boat.position[0], boat.position[1]]))
            rospy.loginfo(f"[Autonomous Mode] [{goal_x:.2f}, {goal_y:.2f}] Arrived")
            break
        else:
            temp = [path[0], path[1], maxSaturation]
            command_publish.publish(Float32MultiArray(data=temp))
            waypoint_publish.publish(Float32MultiArray(data = [0, goal_x, goal_y]))
        rate.sleep()
    rospy.loginfo(f"[{goal_x}, {goal_y}]  AUTONOMOUS MODE FINISH")


def Rotate(goal_psi, maxSaturation = SETTINGS.maxSaturation):
    rate = rospy.Rate(10)
    rospy.loginfo(f"{goal_psi}[deg] Rotate START")
    while not rospy.is_shutdown():
        path = AutonomousModule.rotate(boat, goal_psi)
        temp = [path[0], path[1], maxSaturation]
        command_publish.publish(Float32MultiArray(data=temp))
        waypoint_publish.publish(Float32MultiArray(data = [1, boat.position[0], boat.position[1]]))

        if abs(path[0])<10:
            print(boat.psi ,path[0])
            break
        
        rate.sleep()
    
    rospy.loginfo(f"{goal_psi}[deg] Rotate FINISH")
    

def Wait(wait_time):
    rate = rospy.Rate(10)
    rospy.loginfo(f"{wait_time}[s] WAIT START") 
    for _ in range(10 * wait_time):
        command_publish.publish(Float32MultiArray(data=[0, 0, 0]))
        waypoint_publish.publish(Float32MultiArray(data = [2, boat.position[0], boat.position[1]]))
        rate.sleep()

    rospy.loginfo(f"{wait_time}[s] WAIT FINISH")


def Tuning(goal_psi):
    rate = rospy.Rate(5)
    rospy.loginfo(f"{goal_psi}[deg]  Tuning START")
    while not rospy.is_shutdown():
        path = AutonomousModule.rotate(boat, goal_psi)

        command_publish.publish(Float32MultiArray(data=path))
        
        rate.sleep()




def main():
    ros_init()

    # 작은 경기장
    ###########################################################
    # Autonomous(-15.354178238951135, -6.478271604515612)
    # Autonomous(0.9966924921027385, -36.02468355698511)
    # Autonomous(-15.354178238951135, -6.478271604515612)
    # Autonomous(0.9966924921027385, -36.02468355698511)

    # Tuning(0)
    ###############################################
    # 메인 경기장
    """Autonomous(6.92633210652275, -30.96493609342724, 2.5)
    Wait(2)
    Autonomous(11.367798224033322, -29.614067806396633, 2)
    Rotate(-17)


    # 초록색 사각형, 초록색 원,     빨간색 십자가  
    # 오렌지 십자가, 빨간색 삼각형, 오렌지 원 
    # 오렌지 삼각형, 검은색 십자가, 검은색 사각형
    
    mission_2_index = 3
    if mission_2_index == 1:
        Autonomous(9.465563243546057, -27.737363590393215, 1)
        Autonomous(6.709498732059728, -19.69653245760128, 1)
    elif mission_2_index == 2:
        Autonomous(10.648695708310697, -27.171221407596022, 1)
        Autonomous(8.264857014233712, -19.509023894090205, 1)
    elif mission_2_index == 3:
        Autonomous(11.950871016364545, -26.48359113559127, 1)
        Autonomous(9.60564893251285, -19.3926944504492, 1)

    Autonomous(8.57, -16.35)

    Autonomous(4.50, -4.70)

    Autonomous(2.29, 2.84)

    Autonomous(-2.30, 1.20)"""

    ###############################################
    # 시뮬레이터
    Autonomous(33, 1.5, 2, 500)
    Rotate(0)
    Wait(2)

    Autonomous(33, 8.5, 0.5, 200)
    Rotate(-90)
    Wait(2)

    mission_2_index = 2
    if mission_2_index == 1:
        Autonomous(32, 4.9, 1) # mission2 첫 번째 입구
        Autonomous(17, 4.9, 1.2, 500) # mission2 첫 번째 입구
    elif mission_2_index == 2:
        Autonomous(32, 6.5, 1) # mission2 두 번째 입구
        Autonomous(17, 6.5, 1.2, 500) # mission2 두 번째 입구
    elif mission_2_index == 3:
        Autonomous(32, 8.3, 1) # mission3 세 번째 입구
        Autonomous(17, 8.3, 1.2, 500) # mission3 세 번째 입구

    Autonomous(15.5, 6.5, 200)
    Rotate(-90)
    Wait(2)
    Autonomous(0, 6.5)
    Rotate(180)
    Wait(2)
    Autonomous(0, 0)
    Rotate(90)
    print("MISSION CLEAR")
    
    ###############################################







if __name__ == '__main__':
    try:
        pastPrint = None ## 같은 내용 한번만 print 되도록 하기
        boat = Boat()
        main()
    except rospy.ROSInterruptException:   
        pass
