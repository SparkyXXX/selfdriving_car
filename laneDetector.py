#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import cv2
import numpy as np


class VehicleDetector:
    def __init__(self):
        self.bgr_seg = 0.65  # 画面截取比例
        self.binary_lane_seg = 0.4  # 车道线检测线比例
        self.single_lane_threshold = 0.5  # 偏移量过大阈值
        self.theta = 0.1  # 车道线检测线宽度

    def feedCap(self, bgr_img):
        lower_yellow = np.array([0, 0, 221])
        higher_yellow = np.array([180, 30, 255])
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        offset = 0

        H, W = bgr_img.shape[:2]
        height = int(H * self.bgr_seg)
        img_hsv = cv2.cvtColor(bgr_img[height:], cv2.COLOR_BGR2HSV)
        mask_yellow = cv2.inRange(img_hsv, lower_yellow, higher_yellow)
        gray = cv2.cvtColor(bgr_img[height:], cv2.COLOR_RGB2GRAY)
        _, binary_img_gray = cv2.threshold(gray, 230, 255, cv2.THRESH_BINARY)
        mask_yellow = cv2.morphologyEx(mask_yellow, cv2.MORPH_CLOSE, kernel, iterations=1)
        mask_yellow = cv2.medianBlur(mask_yellow, ksize=5)
        mask_yellow = cv2.morphologyEx(mask_yellow, cv2.MORPH_OPEN, kernel, iterations=1)
        binary_img_gray = cv2.medianBlur(binary_img_gray, ksize=5)
        # cv2.line(bgr_img, (0, height), (W, height), (0, 0, 255), 2)

        # 车道线检测
        binary_lane_seg_H = int(mask_yellow.shape[0] * self.binary_lane_seg)
        bgr_lane_seg_H = height + binary_lane_seg_H
        start = -int(mask_yellow.shape[0] * self.theta) + binary_lane_seg_H
        end = int(mask_yellow.shape[0] * self.theta) + binary_lane_seg_H

        if np.sum(mask_yellow[start:end] == 255) == 0:
            cv2.putText(bgr_img, 'No Lane Detected!', (10, 40), cv2.FONT_HERSHEY_COMPLEX, 0.7, (255, 0, 0), 2)
            # cv2.rectangle(bgr_img, (0, start + height), (W, end + height), (0, 0, 200), -1)
            return binary_img_gray, bgr_img, offset

        lane_seg_mask = np.where(mask_yellow[start:end] == 255)
        lane_left = np.min(lane_seg_mask)
        lane_right = np.max(lane_seg_mask)
        lane_center = (lane_left + lane_right) / 2
        # cv2.rectangle(bgr_img, (0, start + height), (W, end + height), (0, 255, 0), -1)

        cv2.circle(bgr_img, (lane_left, bgr_lane_seg_H), 3, (0, 0, 255), 2)
        cv2.circle(bgr_img, (lane_right, bgr_lane_seg_H), 3, (0, 0, 255), 2)
        cv2.circle(bgr_img, (int(lane_center), bgr_lane_seg_H), 5, (0, 0, 255), -1)

        if lane_right - lane_left < W * self.single_lane_threshold:
            offset = float(lane_center - W / 2) / float(W)
            if offset > 0:
                offset = 0.5 - offset
            else:
                offset = -(0.5 - abs(offset))
            if lane_center > W / 2:
                text = 'Too lane_right:{:.4f}'.format(offset)
            else:
                text = 'Too lane_left:{:.4f}'.format(offset)
            cv2.putText(bgr_img, text, (10, 50), cv2.FONT_HERSHEY_COMPLEX, 0.7, (255, 0, 0), 2)
        else:
            offset = float(lane_center - W / 2) / float(W)
            cv2.putText(bgr_img, 'lane_center Offset:{:.4f}'.format(offset), (10, 50), cv2.FONT_HERSHEY_COMPLEX, 0.7, (255, 0, 0), 2)

        return binary_img_gray, bgr_img, offset


if __name__ == '__main__':
    fps = 60
    path = "test.mp4"
    detector = VehicleDetector()
    capture = cv2.VideoCapture(path)
    t = int(1000 / fps)

    while True:
        _, bgr_img = capture.read()
        if bgr_img is None:
            break
        bgr_img = cv2.resize(bgr_img, None, fx=0.5, fy=0.5)
        result, bgr_img, offset = detector.feedCap(bgr_img)
        result = cv2.merge([result, result, result])
        result = np.vstack([bgr_img, result])
        cv2.imshow('demo', result)
        cv2.waitKey(t)
    capture.release()
