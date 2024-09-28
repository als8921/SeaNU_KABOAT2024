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
        self.position = [0, 0]
        self.psi = 0
        self.scan = [0] * 360
        self.waypoints = [
            [1.6417313533020206, 5.421214632689953]
        ]

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
    def waypoint_callback(data):
        if(data.point.z == -1):
            boat.waypoints = []
        else:
            boat.waypoints.append([data.point.x, data.point.y])
        print(boat.waypoints)
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
    global command_publish
    rospy.init_node('main_node', anonymous=True)

    command_publish = rospy.Publisher('/command', Float32MultiArray, queue_size=10)

    if SETTINGS.isSimulator:
        rospy.Subscriber("Lidar", Float64MultiArray, CallBack.simulator_laser_scan_callback)
    else:
        rospy.Subscriber("scan", LaserScan, CallBack.laser_scan_callback)

    gps_sub = message_filters.Subscriber("KABOAT/UTM", Float64MultiArray)
    imu_sub = message_filters.Subscriber("KABOAT/Heading", Float32)

    rospy.Subscriber("/Waypoint", PointStamped, CallBack.waypoint_callback)

    ts = message_filters.ApproximateTimeSynchronizer([gps_sub, imu_sub], queue_size=10, slop=0.1, allow_headerless=True)
    ts.registerCallback(lambda gps_data, imu_data: (CallBack.gps_callback(gps_data), CallBack.imu_callback(imu_data)))





def utm(data):
    utm_x, utm_y, _, _ = utm.from_latlon(data.latitude, data.longitude)

def Automode(goal_x,goal_y):
    rate = rospy.Rate(10)  # 10Hz로 퍼블리시
    while not rospy.is_shutdown():
        loginfoOnce(f"[Autonomous Mode] [{goal_x:.2f}, {goal_y:.2f}] Start")
        

        path = AutonomousModule.pathplan(boat, goal_x, goal_y)

        if AutonomousModule.goal_passed(boat, goal_x, goal_y, SETTINGS.GoalRange):

            command_publish.publish(Float32MultiArray(data=[0, 0]))
            rospy.loginfo(f"[Autonomous Mode] [{goal_x:.2f}, {goal_y:.2f}] Arrived")
            break
        else:
            command_publish.publish(Float32MultiArray(data=path))
        rate.sleep()


def rotate_mode(goal_psi):
    while not rospy.is_shutdown():
        path = AutonomousModule.rotate(boat, goal_psi)

        command_publish.publish(Float32MultiArray(data=path))
        rospy.loginfo(path)

        loginfoOnce(f"[Autonomous Mode] WayPoint Empty")
        if abs(path[0])<1:
            print(boat.psi ,path[0])
            loginfoOnce(f"[Rotate Finish]")
            break
    

def Wait(wait_time):
    first_time=time.time()
    while(time.time() - first_time < wait_time):
        command_publish.publish(Float32MultiArray(data=[0,0]))






def main():
    ros_init()
    rate = rospy.Rate(10)  # 10Hz로 퍼블리시
    # Automode(-3,3)
    # Automode(-25,4)
    # Automode(-3,3)
    # rotate_mode(0)
    # command_publish.publish(Float32MultiArray(data=[0,0]))
    # time.sleep(2)
    # # Wait(3)
    # Automode(64,25)
    # rotate_mode(-90)
    # # Wait(3)
    # command_publish.publish(Float32MultiArray(data=[0,0]))
    # time.sleep(2)
    # Automode(0,25)
    # rotate_mode(-180)
    # # Wait(3)
    # command_publish.publish(Float32MultiArray(data=[0,0]))
    # time.sleep(2)
    # Automode(0,0)
    # print('Rcsdjhfewiunf')







    
    # rotate_mode(180)
    # Automode(0,0)
    # rotate_mode(180)
    # Automode(5,5)

    """
    배를 제자리에서 회전하도록 만드는 부분
    """
    #############################################################################################################################################################
    
    while not rospy.is_shutdown():
            
        """
        배를 제자리에서 회전하도록 만드는 부분
        """
        path = AutonomousModule.rotate(boat, 0)

        command_publish.publish(Float32MultiArray(data=path))
        rospy.loginfo(path)
        print(boat.psi)

        rate.sleep()  # 주기 유지
    ##########################################



if __name__ == '__main__':
    try:
        pastPrint = None ## 같은 내용 한번만 print 되도록 하기
        boat = Boat()
        print(boat.waypoints)
        main()
    except rospy.ROSInterruptException:
        pass
