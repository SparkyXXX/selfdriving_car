#!/usr/bin/env python
# -*- coding: UTF-8 -*-

# import imutils
import cv2
import numpy as np
from math import sqrt
import matplotlib.pyplot as plt


def Highlight_lowpass(value, min=0, max=0.5, ratio=1.0):
    # 0 <= value <= 0.5
    new = ratio * value + (1 - ratio) * max

    return new


class VehicleDetector():

    def __init__(self):

        self.bgr_seg = 0.6  # 画面截取比例
        self.binary_lane_seg = 0.4  # 车道线检测线比例
        self.binary_pedroad_seg_01 = 0.2  # 斑马线检测线01比例
        self.binary_pedroad_seg_02 = 0.8  # 斑马线检测线02比例

        self.lane_with_redroad_W_threshold = 0.7
        self.pedroad_threshold = 0.25  # 斑马线白点阈值
        self.single_lane_threshold = 0.5  # 偏移量过大阈值
        self.theta = 0.15  # 车道线检测线宽度

        self.size = None

    def feedCap(self, bgr_img):
        org_img = bgr_img
        lower_yellow = np.array([15, 5, 241])  # 黄色阈值下界
        higher_yellow = np.array([158, 255, 255])  # 黄色阈值上界
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))  # 返回指定形状和尺寸的结构元素
        kernel2 = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        # offset = None  # 车辆偏移量的估计值
        offset = 0
        PdeRoad_bool = [False, False]  # 斑马线检测线01、02是否检测到斑马线

        # 提取摄像头画面下半部分二值图
        H, W = bgr_img.shape[:2]
        # HSV
        img_hsv = cv2.cvtColor(bgr_img[int(H * self.bgr_seg):], cv2.COLOR_BGR2HSV)  # cvtcolor颜色空间转换
        mask_yellow = cv2.inRange(img_hsv, lower_yellow, higher_yellow)  # 处于阈值内的像素值设置为白色，否则黑色
        # plt.imshow(img_hsv)
        # plt.show()
        _, binary_img_gray = cv2.threshold(
            cv2.cvtColor(bgr_img[int(H * self.bgr_seg):], cv2.COLOR_RGB2GRAY), 230
            , 255, cv2.THRESH_BINARY
        )  # 去噪，去掉很小或者很大像素值的像素点
        # 开运算2
        mask_yellow = cv2.morphologyEx(mask_yellow, cv2.MORPH_CLOSE, kernel, iterations=1)  # 形态学变化函数
        mask_yellow = cv2.medianBlur(mask_yellow, ksize=5)  # 使用中值滤波平滑图像
        mask_yellow = cv2.morphologyEx(mask_yellow, cv2.MORPH_OPEN, kernel, iterations=1)
        # binary_img_gray = cv2.morphologyEx(binary_img_gray, cv2.MORPH_CLOSE, kernel2,iterations=1)
        binary_img_gray = cv2.medianBlur(binary_img_gray, ksize=5)
        cv2.imshow("test", mask_yellow)
        cv2.imshow('test2', binary_img_gray)
        cv2.line(bgr_img, (0, int(H * self.bgr_seg)),
                 (W, int(H * self.bgr_seg)), (0, 0, 255), 2)

        # 人行横道检测01
        binary_pedroad_seg_H_01 = int(
            binary_img_gray.shape[0] * self.binary_pedroad_seg_01)  # shape[0]图像高度
        bgr_pedroad_seg_H_01 = int(H * self.bgr_seg) + binary_pedroad_seg_H_01
        pedroad_seg_mask_01 = binary_img_gray[binary_pedroad_seg_H_01]

        # 人行横道检测02
        binary_pedroad_seg_H_02 = int(
            binary_img_gray.shape[0] * self.binary_pedroad_seg_02)
        bgr_pedroad_seg_H_02 = int(H * self.bgr_seg) + binary_pedroad_seg_H_02
        pedroad_seg_mask_02 = binary_img_gray[binary_pedroad_seg_H_02]

        # 是否有人行横道01
        pedroad_ratio_01 = float(np.sum(pedroad_seg_mask_01 == 255)) / float(pedroad_seg_mask_01.shape[0])
        if pedroad_ratio_01 > self.pedroad_threshold:
            cv2.line(bgr_img, (0, bgr_pedroad_seg_H_01), (W, bgr_pedroad_seg_H_01), (0, 255, 0), 2)
            PdeRoad_bool[1] = True
            cv2.putText(bgr_img, 'Pedestrain Road!', (10, 80), cv2.FONT_HERSHEY_COMPLEX, 0.7, (255, 0, 0), 2)
        else:
            cv2.line(bgr_img, (0, bgr_pedroad_seg_H_01), (W, bgr_pedroad_seg_H_01), (255, 0, 0), 2)

        # 是否有人行横道02
        PdeRoad_bool[0] = False
        # pedroad_ratio_02 = float(
        #    np.sum(pedroad_seg_mask_02 == 255))/float(pedroad_seg_mask_02.shape[0])
        # if pedroad_ratio_02 > self.pedroad_threshold:
        #    cv2.line(bgr_img, (0, bgr_pedroad_seg_H_02),
        #             (W, bgr_pedroad_seg_H_02), (0, 255, 0), 2)
        #    PdeRoad_bool[0] = True
        #    cv2.putText(bgr_img, 'Pedestrain Road!', (10, 80),
        #                cv2.FONT_HERSHEY_COMPLEX, 0.7, (255, 0, 0), 2)
        # else:
        #    cv2.line(bgr_img, (0, bgr_pedroad_seg_H_02),
        #             (W, bgr_pedroad_seg_H_02), (255, 0, 0), 2)

        # 车道线检测
        binary_lane_seg_H = int(mask_yellow.shape[0] * self.binary_lane_seg)
        bgr_lane_seg_H = int(H * self.bgr_seg) + binary_lane_seg_H

        start = -int(mask_yellow.shape[0] * self.theta) + binary_lane_seg_H
        end = int(mask_yellow.shape[0] * self.theta) + binary_lane_seg_H

        binary_results = np.vstack([mask_yellow, binary_img_gray])

        if self.size is None:
            self.size = (bgr_img.shape[1], bgr_img.shape[0])
            fourcc = cv2.VideoWriter_fourcc('m', 'p', '4', 'v')  # opencv3.0
            self.videoWriter = cv2.VideoWriter('/home/pi/result.mp4', fourcc, 5, self.size)  # 保存视频
            # self.videoWriter = cv2.VideoWriter(
            #   './result.mp4', fourcc, 5, self.size)

        # 是否有车道线
        if np.sum(mask_yellow[start:end] == 255) == 0:
            cv2.putText(bgr_img, 'No Lane Detected!', (10, 40), cv2.FONT_HERSHEY_COMPLEX, 0.7, (255, 0, 0), 2)
            cv2.rectangle(bgr_img, (0, start + int(H * self.bgr_seg)), (W, end + int(H * self.bgr_seg)), (0, 0, 200),
                          -1)
            # self.videoWriter.write(bgr_img)
            return binary_results, bgr_img, offset, PdeRoad_bool, 0

        if PdeRoad_bool[1] or PdeRoad_bool[0]:
            # 如果有斑马线，只检测右侧车道
            seg_W = int(mask_yellow.shape[1] * self.lane_with_redroad_W_threshold)
            lane_seg_mask = np.where(mask_yellow[start:end, seg_W:] == 255)[1]  # 返回符合条件的索引
            if np.sum(mask_yellow[start:end, seg_W:] == 255) == 0:
                cv2.putText(bgr_img, 'No Lane Detected!', (10, 40), cv2.FONT_HERSHEY_COMPLEX, 0.7, (255, 0, 0), 2)
                cv2.rectangle(bgr_img, (0, start + int(H * self.bgr_seg)), (W, end + int(H * self.bgr_seg)),
                              (0, 0, 200), -1)
                return binary_results, bgr_img, offset, PdeRoad_bool, 0
            lane_left = np.max(lane_seg_mask) + seg_W
            lane_right = lane_left
            lane_center = lane_left
            cv2.rectangle(bgr_img, (seg_W, start + int(H * self.bgr_seg)), (W, end + int(H * self.bgr_seg)),
                          (0, 255, 0), -1)
            cv2.rectangle(bgr_img, (0, start + int(H * self.bgr_seg)), (seg_W, end + int(H * self.bgr_seg)),
                          (0, 0, 200), -1)
        else:
            # if True:
            lane_seg_mask = np.where(mask_yellow[start:end] == 255)[1]
            lane_left = np.min(lane_seg_mask)
            lane_right = np.max(lane_seg_mask)
            lane_center = (lane_left + lane_right) / 2
            cv2.rectangle(bgr_img, (0, start + int(H * self.bgr_seg)), (W, end + int(H * self.bgr_seg)), (0, 255, 0),
                          -1)

        cv2.circle(bgr_img, (lane_left, bgr_lane_seg_H), 3, (0, 0, 255), 2)  # cv2.circle根据给定的圆心半径画圆
        cv2.circle(bgr_img, (lane_right, bgr_lane_seg_H), 3, (0, 0, 255), 2)
        cv2.circle(bgr_img, (int(lane_center), bgr_lane_seg_H), 5, (0, 0, 255), -1)

        if lane_right - lane_left < W * self.single_lane_threshold:
            offset = float(lane_center - W / 2) / float(W)
            if offset > 0:
                offset = Highlight_lowpass(0.5 - offset)
                # offset = sqrt(offset * 2) / 2
            else:
                offset = -(Highlight_lowpass(0.5 - abs(offset)))
                # offset = - sqrt(offset * 2) / 2
            if lane_center > W / 2:
                text = 'Too lane_right:{:.4f}'.format(offset)
            else:
                text = 'Too lane_left:{:.4f}'.format(offset)
            cv2.putText(bgr_img, text, (10, 50), cv2.FONT_HERSHEY_COMPLEX, 0.7, (255, 0, 0), 2)
        else:
            offset = float(lane_center - W / 2) / float(W)
            cv2.putText(bgr_img, 'lane_center Offset:{:.4f}'.format(offset), (10, 50), cv2.FONT_HERSHEY_COMPLEX, 0.7,
                        (255, 0, 0), 2)

        ratio_info = 'ratio_1:{:.3f} '.format(pedroad_ratio_01)
        cv2.putText(bgr_img, ratio_info, (10, 20), cv2.FONT_HERSHEY_COMPLEX, 0.7, (255, 0, 0), 2)

        # self.videoWriter.write(bgr_img)
        return binary_results, bgr_img, offset, PdeRoad_bool, 1


if __name__ == '__main__':

    name = 'demo'
    # path = 'output.avi'
    path = 'challenge_video.mp4'

    det = VehicleDetector()
    cap = cv2.VideoCapture(path)
    # cap = cv2.VideoCapture(0)
    fps = int(cap.get(5))  # 获取帧速率
    print('fps:', fps)
    t = int(1000 / fps)

    size = None
    car = None

    while True:
        # try:
        _, bgr_img = cap.read()
        if bgr_img is None:
            break

        bgr_img = cv2.resize(bgr_img, None, fx=0.5, fy=0.5)

        result, bgr_img, offset, _, _ = det.feedCap(bgr_img)
        result = cv2.merge([result, result, result])  # 图像通道合并
        result = np.vstack([bgr_img, result])  # 数据拼接
        cv2.imshow(name, result)
        cv2.waitKey(t)
        if cv2.getWindowProperty(name, cv2.WND_PROP_AUTOSIZE) < 1:
            break

    cap.release()
