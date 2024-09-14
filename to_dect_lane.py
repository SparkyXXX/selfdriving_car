#!/usr/bin/env python
# -*- coding: UTF-8 -*-
from laneDetector import *
import cv2
import serial
import time

# import rospy

# from geometry_msgs.msg import Twist
# from std_msgs.msg import Int32

# import math

# ser = serial.Serial('COM3', 115200)
# if not ser.isOpen:
#     ser.open()
# time.sleep(0.5)


def servoTest(offset):
    if offset < 65: return 'A'
    if 65 <= offset < 75: return 'B'
    if 75 <= offset < 85: return 'C'
    if 85 <= offset < 95: return 'D'
    if 95 <= offset < 105: return 'E'
    if 105 <= offset < 115: return 'F'
    if 115 <= offset: return 'G'


width = 1280
height = 720

if __name__ == '__main__':
    # rospy.init_node('lane_vel', anonymous=True)
    # rate = rospy.Rate(32)
    # im = cv2.VideoCapture('/dev/video10')
    im = cv2.VideoCapture('test.mp4')
    # im = cv2.VideoCapture(0)
    im.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    im.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    # 创建检测器
    det = VehicleDetector()
    # 检测车道线

    #     - result：输出的车道线分�?#     - frame：可视化图像
    #     - offset：偏移的相对�?\

    k = 60
    # k  : 用来控制转弯的放大倍数

    # pub_lane_vel = rospy.Publisher('lane_vel', Twist, queue_size=1)
    # pub_lane_judge = rospy.Publisher('laneJudge', Int32, queue_size=1)
    # pub_ped_judge1 = rospy.Publisher('Pedestrain_Judge1', Int32, queue_size=1)
    ### pub_lim = rospy.Publisher('speedflag',Int32,queue_size=1)
    ### pub_ped_judge0=rospy.Publisher('Pedestrain_Judge0',Int32,queue_size=1)
    # cam_cmd = Twist()
    ### pub_imagePub = rospy.Publisher('images', Image, queue_size=1)
    offset_old = 0
    offset = 0

    # while not rospy.is_shutdown():
    while 1:
        ret, img = im.read()
        img = cv2.resize(img, None, fx=0.3, fy=0.3)
        offset_old = offset
        result, frame, offset, Pedestrain_Road, judge = det.feedCap(img)
        if offset * offset_old < 0 and abs(offset - offset_old) > 0.4:
            offset = offset_old
        laneJudge = 1
        flag = 0
        if offset > 0.2:
            flag = 1
        else:
            flag = 0
        # print("--------offset is ---------", offset)
        if offset < -0.2:
            k = 40
        elif offset < -0.12:
            k = 45
        elif offset < -0.02:
            k = 45
        elif offset < 0:
            k = 70
        elif offset < 0.02:
            k = 70
        elif offset < 0.12:
            k = 45
        elif offset < 0.2:
            k = 50
        else:
            k = 40
        if offset is not None:
            if not Pedestrain_Road[1]:
                pass
                # cam_cmd.angular.z = k * offset

        cv2.imshow('demo', frame)
        # cv2.imshow('a',result)
        cv2.waitKey(30)
        # print('dectection spinning')
        if offset is not None:
            print(90 - k * offset)
            # ser.write(str.encode(servoTest(90 - k * offset)))
            # time.sleep(0.1)
        Pedestrain_judge1 = 0
        Pedestrain_judge0 = 0
        if Pedestrain_Road[1]:
            Pedestrain_judge1 = 1

        ### print(result)
        # pub_lane_judge.publish(judge)
        # pub_lane_vel.publish(cam_cmd)
        # pub_ped_judge1.publish(Pedestrain_judge1)
        ### pub_ped_judge0.publish(Pedestrain_judge0)
        # rate.sleep()
