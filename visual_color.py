# This work is licensed under the MIT license.
# Copyright (c) 2013-2023 OpenMV LLC. All rights reserved.
# https://github.com/openmv/openmv/blob/master/LICENSE
#
# Hello World Example
#
# Welcome to the OpenMV IDE! Click on the green run arrow button below to run the script!

import sensor
import time
import math

# 摄像头初始化
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)                 # 关闭自动增益
sensor.set_auto_whitebal(False)             # 关闭自动白平衡
#sensor.set_auto_exposure(False)
#sensor.set_manual_exposure(5000)
#手动曝光，根据环境调整 4.5.9固件版本不支持

# 红色灯条阈值（需现场用 IDE 阈值编辑器标定）
red_threshold = (99, 100, -76, 127, -30, 72)
clock = time.clock()

# 筛选灯条的函数
def is_light_strip(blob):
    # QVGA 下灯条面积较大
    if blob.area() < 200 or blob.pixels() < 100:
        return False
    # 密度放宽，适应倾斜/反光
    if blob.density() < 0.4:
        return False
    # 长宽比为 1.5 倍，避免漏检较宽灯条
    if blob.h() < blob.w() * 1.5:
        return False
    return True

while True:
    clock.tick()
    img = sensor.snapshot()

    # 查找红色色块
    blobs = img.find_blobs([red_threshold],
                           x_stride=2,
                           y_stride=1,
                           area_threshold=30,
                           pixels_threshold=20,
                           merge=True,
                           margin=5)

    strips = []
    for b in blobs:
        if is_light_strip(b):
            strips.append(b)
            img.draw_rectangle(b.rect(), color=(255, 0, 0))

# 两两配对，寻找装甲板
    best_pair = None
    best_score = 0
    for i in range(len(strips)):
        for j in range(i+1, len(strips)):
            b1, b2 = strips[i], strips[j]
            dx = abs(b1.cx() - b2.cx())          # 水平距离
            dy = abs(b1.cy() - b2.cy())          # 垂直错位
            h1 = b1.h()
            h2 = b2.h()
            avg_h = (h1 + h2) / 2
            h_ratio = abs(h1 - h2) / max(h1, h2)
            # 动态条件：水平距离约为灯条高度的 0.5~2.5 倍
            # 垂直错位小于平均高度的 30%，高度差比例小于 40%
            if (dx > 0.5 * avg_h and dx < 2.5 * avg_h and
                dy < 0.3 * avg_h and h_ratio < 0.4):
                # 简单评分，选择最优配对（当有多对候选时）
                ideal_dx = 1.2 * avg_h
                score = (max(0, 1 - abs(dx - ideal_dx) / ideal_dx) +
                         max(0, 1 - dy / (0.3 * avg_h)) +
                         max(0, 1 - h_ratio))
                if score > best_score:
                    best_pair = (b1, b2)
                    best_score = score
        if best_pair:
            break

    if best_pair:
        b1, b2 = best_pair
        # 计算合并后的装甲板外接矩形
        x = min(b1.x(), b2.x())
        y = min(b1.y(), b2.y())
        w = max(b1.x() + b1.w(), b2.x() + b2.w()) - x
        h = max(b1.y() + b1.h(), b2.y() + b2.h()) - y
        # 用白色框绘出装甲板整体
        img.draw_rectangle(x, y, w, h, color=(255, 255, 255), thickness=2)
        # 计算装甲板中心
        cx = (b1.cx() + b2.cx()) // 2
        cy = (b1.cy() + b2.cy()) // 2
        img.draw_cross(cx, cy, size=8, color=(0, 255, 0))
        img.draw_line(b1.cx(), b1.cy(), b2.cx(), b2.cy(), color=(0, 255, 0))
        print("Armor center: ({}, {}), box: x={} y={} w={} h={}".format(cx, cy, x, y, w, h))
    else:
        # 没有检测到装甲板时，可以清空输出或发送 N
        pass
    print("FPS: %.2f" % clock.fps())
