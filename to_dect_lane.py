#!/usr/bin/env python
# -*- coding: UTF-8 -*-
from laneDetector import *
import cv2
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
#import math

width = 1280
height = 720



if __name__ == '__main__':
    rospy.init_node('lane_vel', anonymous=True)
    rate = rospy.Rate(32)
    im = cv2.VideoCapture('/dev/video10')
    #im = cv2.VideoCapture(0)
    im.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    im.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    # 创建检测器
    det = VehicleDetector()
    # 检测车道线
    
#     - result：输出的车道线分�?#     - frame：可视化图像
#     - offset：偏移的相对�?\
   
    k=60
    #k  : 用来控制转弯的放大倍数

    pub_lane_vel=rospy.Publisher('lane_vel', Twist, queue_size=1)
    pub_lane_judge=rospy.Publisher('laneJudge',Int32,queue_size=1)
    pub_ped_judge1=rospy.Publisher('Pedestrain_Judge1',Int32,queue_size=1)
    #pub_lim = rospy.Publisher('speedflag',Int32,queue_size=1)
    #pub_ped_judge0=rospy.Publisher('Pedestrain_Judge0',Int32,queue_size=1)
    cam_cmd = Twist()
    #pub_imagePub = rospy.Publisher('images', Image, queue_size=1)
    offset_old = 0
    offset = 0
    
    while not rospy.is_shutdown():
        ret,img = im.read()
        img = cv2.resize(img, None, fx=0.3, fy=0.3)
	offset_old = offset
        result, frame, offset ,Pedestrain_Road,judge = det.feedCap(img)
	if offset * offset_old < 0 and abs(offset - offset_old) > 0.4:
	    offset = offset_old      
	laneJudge=1
	flag = 0
	if offset > 0.2:
	    flag = 1
	else:
	    flag = 0
        #print("--------offset is ---------", offset)
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
        if offset != None:
            if Pedestrain_Road[1] == False:
                cam_cmd.angular.z = k * offset 

        #可视化操�?        
        cv2.imshow('demo',frame)
        #cv2.imshow('a',result)
        cv2.waitKey(1)
        #print('dectection spinning')
        if offset != None:
            print(k*offset)
        Pedestrain_judge1 = 0
        Pedestrain_judge0 = 0
        if Pedestrain_Road[1] == True:
            Pedestrain_judge1 = 1
        
        #print(result)
        pub_lane_judge.publish(judge)
        pub_lane_vel.publish(cam_cmd)
        pub_ped_judge1.publish(Pedestrain_judge1)
        #pub_ped_judge0.publish(Pedestrain_judge0)
        rate.sleep()
