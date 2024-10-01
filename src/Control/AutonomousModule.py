#!/usr/bin/env python3
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
import numpy as np
import SETTINGS
from math import ceil, floor, exp

class Boat:
    def __init__(self):
        self.position = [0, 0]
        self.psi = 0
        self.scan = [0] * 360
        
Goal_Psi = 0
Goal_Distance = 0


def normalize_angle(angle): return (angle + 180) % 360 - 180

def cost_func_angle(x):
    # x = abs(x)
    # return 0.01 * x if x <= 10 else 0.1 * (x - 10)
    return 1 - exp(- (x / 100) ** 2)

def cost_func_distance(x):
    # return 0 if x > 7 else 10 - 10/7 * x
    return exp(- (x / 3) ** 2)

def calculate_safe_zone(ld):
    safe_zone = [SETTINGS.AVOID_RANGE] * 360
    for i in range(-180, 181):
        if 0 < ld[i] < SETTINGS.AVOID_RANGE:
            safe_zone[i] = 0

    temp = np.array(safe_zone)
    for i in range(-180, 180):
        if safe_zone[i] > safe_zone[i + 1]:
            for j in range(floor(i + 1 - np.arctan2(SETTINGS.BOAT_WIDTH/2, ld[i + 1]) * 180 / np.pi), i + 1):
                temp[j] = 0
        if safe_zone[i] < safe_zone[i + 1]:
            for j in range(i, ceil(i + np.arctan2(SETTINGS.BOAT_WIDTH/2, ld[i]) * 180 / np.pi) + 1):
                temp[j] = 0
    return temp

def calculate_optimal_psi_d(ld, safe_ld, goal_psi):
    """
    Cost함수를 적용하여 각도별 Cost를 계산
    목적지 까지의 각도와 각도별 LaserScan 데이터에 대한 함수 사용

    Args:
        LaserScan ld
        Float[] safe_ld
        Float goal_psi

    Returns:
        Cost가 가장 낮은 각도 리턴
    """
    theta_list = [[0, 10000]]

    for i in range(-180, 180):
        if safe_ld[i] > 0:
            cost = (SETTINGS.GAIN_PSI * cost_func_angle(i - goal_psi) + 
                    SETTINGS.GAIN_DISTANCE * cost_func_distance(ld[i]))
            theta_list.append([i, cost])
    return sorted(theta_list, key=lambda x: x[1])[0][0]


def Final_cost(ld, safe_ld, goal_psi):
    """
    Cost함수를 적용하여 각도별 Cost를 계산
    목적지 까지의 각도와 각도별 LaserScan 데이터에 대한 함수 사용

    Args:
        LaserScan ld
        Float[] safe_ld
        Float goal_psi

    Returns:
        Cost가 가장 낮은 각도 리턴
    """
    theta_list = [[0, 10000]]

    for i in range(-180, 180):
        if safe_ld[i] > 0:
            cost = (SETTINGS.GAIN_PSI * cost_func_angle(i - goal_psi) + 
                    SETTINGS.GAIN_DISTANCE * cost_func_distance(ld[i]))
            theta_list.append(cost)
        else:
            theta_list.append(11110)
    theta_list.pop(0)
    return theta_list

def pathplan(boat=Boat(), goal_x=None, goal_y=None):
    """
    LaserScan 데이터를 바탕으로 최적의 TauX, psi_e 값을 찾는 함수

    Args:
        Boat boat
        Float goal_x
        Float goal_y

    Returns:
        [psi_error, tauX]
    """
    global Goal_Psi, Goal_Distance
    print(Goal_Psi, Goal_Distance)
    if len(boat.scan) == 0:
        return [0, 0]

    safe_ld = calculate_safe_zone(boat.scan)
    psi_error = calculate_optimal_psi_d(boat.scan, safe_ld, int(Goal_Psi))
    
    
    ## TauX를 계산하는 부분
    if goal_check():
        psi_error = Goal_Psi
        if psi_error < 30:
            tauX = min((Goal_Distance ** 3) + 120, 500)
        else:
            tauX_dist = min(3 * boat.scan[0] ** 2, 300)
            tauX_psi = 200 / (abs(psi_error) + 1)
            tauX = min(tauX_dist + tauX_psi, 500)

        tauX = 450
        print("장애물 없음")
    else:
        TauX_Gain_Dist = 0.3
        TauX_Gain_Angle = 0.7
        a = 1500
        b = 4
        tauX_dist = 1 - exp(-(boat.scan[0] / b) ** 2)   # 1 - e^(-(x/4)^2)
        tauX_angle = exp(-(psi_error ** 2) / a)         # e^(-(x^2/1500))
        tauX = 100 + 300 * (TauX_Gain_Dist * tauX_dist + TauX_Gain_Angle * tauX_angle)
        tauX = min(tauX, 450)
        # tauX = 150

    # 목표 웨이포인트 데이터 표시
    if goal_x is not None and goal_y is not None:
        dx = goal_x - boat.position[0]
        dy = goal_y - boat.position[1]


        Goal_Psi = np.arctan2(dx, dy) * 180 / np.pi - boat.psi
        Goal_Psi = normalize_angle(Goal_Psi)
        Goal_Distance = np.sqrt(np.power(dx, 2) + np.power(dy, 2))


        return [psi_error, tauX]
    else:
        return [0, 0]


    

def goal_check(boat = Boat()):
    """
    목적지 까지 경로에 장애물이 있는지 판단하는 함수

    Args:
        Boat boat

    Returns:
        장애물이 있는지 판단 결과를 리턴 [Boolean]
    """
    l = Goal_Distance
    theta = ceil(np.degrees(np.arctan2(SETTINGS.BOAT_WIDTH/2, l)))

    check_ld = [0] * 360
    isAble = True

    for i in range(0, 90 - theta):
        angle = normalize_angle(int(Goal_Psi) - 90 + i)
        r = SETTINGS.BOAT_WIDTH /(2 *np.cos(np.radians(i)))
        check_ld[angle] = r
        if(boat.scan[angle] == 0):
            continue
        if(r > boat.scan[angle]):
            isAble = False

    for i in range(-theta, theta + 1):
        check_ld[normalize_angle(int(Goal_Psi) + i)] = l
        if(boat.scan[normalize_angle(int(Goal_Psi) + i)] < l):
            isAble = False

    for i in range(0, 90 - theta):
        angle = normalize_angle(int(Goal_Psi) + 90 - i)
        r = SETTINGS.BOAT_WIDTH /(2 *np.cos(np.radians(i)))
        check_ld[angle] = r
        if(boat.scan[angle] == 0):
            continue
        if(r > boat.scan[angle]):
            isAble = False

    return isAble

def goal_passed(boat = Boat(), goal_x = 0, goal_y = 0, goal_threshold = 2):
    """
    목적지에 도착했는지 판단하는 함수

    Args:
        Boat 객체
        목적지의 x, y 좌표

    Returns:
        도착 결과를 리턴 [Boolean]
    """
    isPassed = False
    if((boat.position[0] - goal_x)**2 + (boat.position[1] - goal_y)**2 < goal_threshold ** 2):
        isPassed = True
    return isPassed






def rotate(boat = Boat(), psi_d=None):
    """
    psi_error를 받고 tau_x는 0으로 해서 실행

    """
    psi_error=normalize_angle(psi_d - boat.psi)
    return [psi_error,0]