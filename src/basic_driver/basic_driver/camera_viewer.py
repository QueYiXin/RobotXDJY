#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2

def main():
    # 打开摄像头 (0表示默认摄像头)
    cap = cv2.VideoCapture(1)

    # 检查摄像头是否成功打开
    if not cap.isOpened():
        print("无法打开摄像头")
        return

    print("摄像头已打开，按 'q' 键退出")

    while True:
        # 读取一帧
        ret, frame = cap.read()

        # 检查是否成功读取
        if not ret:
            print("无法接收帧，退出...")
            break

        # 显示图像
        cv2.imshow('Camera View', frame)

        # 按 'q' 键退出
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 释放摄像头资源
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
