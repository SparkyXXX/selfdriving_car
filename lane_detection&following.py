#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import cv2
import os
import sys
import glob
import numpy as np
import math

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

# 距离映射
x_cmPerPixel = 90 / 665.00
y_cmPerPixel = 81 / 680.00
roadWidth = 665

y_offset = 50.0  # cm

# 轴间距
I = 58.0
# 摄像头坐标系与车中心间距
D = 18.0
# 计算cmdSteer的系数
k = -19


class camera:
    def __init__(self):
        self.camMat = []
        self.camDistortion = []

        #读取摄像头图像或视频
        # self.cap = cv2.VideoCapture('/dev/video10')
        self.cap = cv2.VideoCapture('challenge_video.mp4')
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)#设置宽和高
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        #定义publisher
        self.imagePub = rospy.Publisher('images', Image, queue_size=1)
        self.cmdPub = rospy.Publisher('lane_vel', Twist, queue_size=1)
        self.cam_cmd = Twist()
        self.cvb = CvBridge()

        #定义对应点（用于透视变换）
        src_points = np.array([[3, 570], [387, 460], [906, 452], [1041, 485]], dtype="float32")
        dst_points = np.array([[266., 686.], [266., 19.], [931., 20.], [931., 701.]], dtype="float32")
        self.M = cv2.getPerspectiveTransform(src_points, dst_points)#计算得到转换矩阵

        self.aP = [0.0, 0.0]#aimpoint
        self.lastP = [0.0, 0.0]#lastpoint
        self.Timer = 0#计算时间

    def __del__(self):
        self.cap.release()#释放空间


    def spin(self):
        ret, img = self.cap.read()
        if ret == True:
            image_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)#bgr转gray，opencv默认bgr
            kernel = np.ones((3, 3), np.uint8)#定义图像腐蚀内核
            image_gray = cv2.erode(image_gray, kernel, iterations=1)#图像腐蚀，iteration的值越高，模糊程度(腐蚀程度)就越高 呈正相关关系
            origin_thr = np.zeros_like(image_gray)
            origin_thr[(image_gray >= 125)] = 255#设置阈值进行二值化

            # 进行透视变换处理
            Warped_image= cv2.warpPerspective(origin_thr, self.M, (1280, 720), cv2.INTER_LINEAR)
            cv2.imshow('Warped_image', Warped_image)

            histogram_x = np.sum(Warped_image[int(Warped_image.shape[0] / 2):, :], axis=0)#image.shape[0]-图像高度  image.shape[1]-图像宽度  image.shape[2]-图像通道数
            #当axis为0时,是压缩行,即将每一列的元素相加,将矩阵压缩为一行
            lane_base = np.argmax(histogram_x)#取出a中元素最大值所对应的索引，此时最大值位6，其对应的位置索引值为4，（索引值默认从0开始）
            midpoint_x = int(histogram_x.shape[0] / 2)

            histogram_y = np.sum(Warped_image[0:Warped_image.shape[0], :], axis=1)
            midpoint_y = 320  # int(histogram.shape[0]/2)
            upper_half_histSum = np.sum(histogram_y[0:midpoint_y])
            lower_half_histSum = np.sum(histogram_y[midpoint_y:])
            try:
                hist_sum_y_ratio = (upper_half_histSum) / (lower_half_histSum)
            except:
                hist_sum_y_ratio = 1
            print(hist_sum_y_ratio)

            nwindows = 10
            window_height = int(Warped_image.shape[0] / nwindows)
            nonzero = Warped_image.nonzero()
            nonzeroy = np.array(nonzero[0])
            nonzerox = np.array(nonzero[1])
            lane_current = lane_base  #lane_base = np.argmax(histogram_x)#取出a中元素最大值所对应的索引，此时最大值位6，其对应的位置索引值为4，（索引值默认从0开始）
            margin = 100
            minpix = 25

            lane_inds = []

            #滑框取点
            for window in range(nwindows):
                win_y_low = Warped_image.shape[0] - (window + 1) * window_height
                win_y_high = Warped_image.shape[0] - window * window_height
                win_x_low = lane_current - margin
                win_x_high = lane_current + margin
                good_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_x_low) & (
                        nonzerox < win_x_high)).nonzero()[0]

                lane_inds.append(good_inds)
                if len(good_inds) > minpix:
                    lane_current = int(np.mean(nonzerox[good_inds]))  ####
                elif window >= 3:
                    break

            lane_inds = np.concatenate(lane_inds)#合并

            pixelX = nonzerox[lane_inds]
            pixelY = nonzeroy[lane_inds]

            # calculate the aimPoint
            if (pixelX.size == 0):#可以调整阈值以提高处理帧数
                return

            a2, a1, a0 = np.polyfit(pixelY, pixelX, 2)#二次拟合车道线，y=a2*x^2+a1*x+a0
            mean_X = np.average(pixelX)

            frontDistance = np.argsort(pixelY)[int(len(pixelY) / 8)]
            aimLaneP = [pixelX[frontDistance], pixelY[frontDistance]]

            # 计算aimLaneP处斜率，从而得到目标点的像素坐标
            lanePk = 2 * a2 * aimLaneP[0] + a1
            if (abs(lanePk) < 0.1):
                if lane_base >= midpoint_x:
                    LorR = -1.25
                else:
                    if hist_sum_y_ratio < 0.1:
                        LorR = -1.25
                    else:
                        LorR = 0.8
                self.aP[0] = aimLaneP[0] + LorR * roadWidth / 2
                self.aP[1] = aimLaneP[1]
            else:
                if (2 * a2 * mean_X + a1) > 0:  # 斜率大于0
                    if a2 > 0:
                        # x_intertcept = (-a1+(abs(a1*a1-4*a2*(a0 - 1280))**0.5))/(2*a2)
                        x_intertcept = (-a1 + (abs(a1 * a1 - 4 * a2 * (a0 - 1099.0)) ** 0.5)) / (2 * a2)  # 求截距

                    else:
                        x_intertcept = (-a1 - (abs(a1 * a1 - 4 * a2 * (a0 - 1099.0)) ** 0.5)) / (2 * a2)


                else:  # 斜率小于0
                    if a2 > 0:
                        # x_intertcept = (-a1-(abs(a1*a1-4*a2*(a0 - 1280))**0.5))/(2*a2)
                        x_intertcept = (-a1 - (abs(a1 * a1 - 4 * a2 * (a0 - 1099.0)) ** 0.5)) / (2 * a2)

                    else:
                        x_intertcept = (-a1 + (abs(a1 * a1 - 4 * a2 * (a0 - 1099.0)) ** 0.5)) / (2 * a2)

                if (x_intertcept > 599):
                    LorR = -1.4  # RightLane
                else:
                    LorR = 0.8  # LeftLane

                k_ver = - 1 / lanePk

                theta = math.atan(k_ver)
                self.aP[0] = aimLaneP[0] + math.cos(theta) * (LorR) * roadWidth / 2
                self.aP[1] = aimLaneP[1] + math.sin(theta) * (LorR) * roadWidth / 2
            #映射
            self.aP[0] = (self.aP[0] - 599) * x_cmPerPixel
            self.aP[1] = (680 - self.aP[1]) * y_cmPerPixel + y_offset

            # 计算目标点的真实坐标
            if (self.lastP[0] > 0.001 and self.lastP[1] > 0.001):
                if (((self.aP[0] - self.lastP[0]) ** 2 + (
                        self.aP[1] - self.lastP[1]) ** 2 > 2500) and self.Timer < 2):  # To avoid the mislead by walkers
                    self.aP = self.lastP[:]
                    self.Timer += 1
                else:
                    self.Timer = 0

            self.lastP = self.aP[:]
            steerAngle = math.atan(2 * I * self.aP[0] / (self.aP[0] * self.aP[0] + (self.aP[1] + D) * (self.aP[1] + D)))

            self.cam_cmd.angular.z = k * steerAngle
            print("steerAngle=", steerAngle)
            self.cmdPub.publish(self.cam_cmd)
            self.imagePub.publish(self.cvb.cv2_to_imgmsg(image_gray))  # image_gray
            cv2.imshow('image_gray', image_gray)
            cv2.waitKey(1)


if __name__ == '__main__':
    rospy.init_node('lane_vel', anonymous=True)
    rate = rospy.Rate(10)

    try:
        cam = camera()
        print(rospy.is_shutdown())  # FALSE
        while not rospy.is_shutdown():
            cam.spin()
            print('betweeen == cam.spin ==')
            rate.sleep()
    except rospy.ROSInterruptException:
        print(rospy.ROSInterruptException)
        pass
