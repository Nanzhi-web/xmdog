#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from ultralytics import YOLO
import os
import cv2
import numpy as np
import time

# -----------------------------------------------------------------
# 1. 关键参数 (根据实际情况微调)
# -----------------------------------------------------------------
KNOWN_WIDTH_METERS = 0.2    # 灯牌实际宽度 (20cm)

# 将 700.0 改为 150.0 左右。
# 这样同样的 70 像素宽度，算出来的距离就会变成：(0.2 * 150) / 70 ≈ 0.42米
FOCAL_LENGTH_PX = 150.0
DISTANCE_THRESHOLD_METERS = 0.5  # 判定阈值 (50cm)
CONFIDENCE_THRESHOLD = 0.7  # YOLO 置信度

# -----------------------------------------------------------------
# 2. 模型加载
# -----------------------------------------------------------------
_THIS_SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
_MODEL_PATH = os.path.join(_THIS_SCRIPT_DIR, 'models', 'best.pt')

# 全局变量初始化
MODEL = None

def _get_model():
    global MODEL
    if MODEL is None:
        try:
            if not os.path.exists(_MODEL_PATH):
                print(f"[ERROR] 找不到模型文件: {_MODEL_PATH}")
                return None
            MODEL = YOLO(_MODEL_PATH)
        except Exception as e:
            print(f"[ERROR] 加载模型失败: {e}")
            return None
    return MODEL

# -----------------------------------------------------------------
# 3. 主检测逻辑 (直接使用 YOLO 框宽度)
# -----------------------------------------------------------------
def detect_yellow_light_state(cv_image: np.ndarray) -> str:
    model = _get_model()
    if model is None:
        return "Light Off"

    try:
        # 运行预测
        results = model(cv_image, conf=CONFIDENCE_THRESHOLD, verbose=False)
        result = results[0]
        
        # 调试信息：输出找到的框数量
        print(f"[YOLO_INTERNAL] 检测到灯牌数量: {len(result.boxes)}")
        
        # 保存一张带框的图方便调试
        debug_path = os.path.join(_THIS_SCRIPT_DIR, "yolo_all_boxes.jpg")
        cv2.imwrite(debug_path, result.plot())

        found_close_light = False
        
        for i, box in enumerate(result.boxes):
            # 获取 YOLO 框的坐标 [x1, y1, x2, y2]
            xyxy = box.xyxy[0].cpu().numpy().astype(int)
            x1, y1, x2, y2 = xyxy[0], xyxy[1], xyxy[2], xyxy[3]
            
            # 核心改进：直接用 YOLO 框的像素宽度计算距离
            pixel_width = float(x2 - x1)
            
            if pixel_width > 0:
                # 距离 = (实际宽度 * 焦距) / 像素宽度
                distance_m = (KNOWN_WIDTH_METERS * FOCAL_LENGTH_PX) / pixel_width
                
                print(f"[YOLO_INTERNAL] 目标 #{i}: 像素宽度={pixel_width}, 估算距离={distance_m:.2f}m")
                
                # 只要有一个灯牌距离符合条件，就判定为 "Light On"
                if distance_m < DISTANCE_THRESHOLD_METERS:
                    found_close_light = True
                    print(f"[YOLO_INTERNAL] >>> 距离满足条件: {distance_m:.2f}m < {DISTANCE_THRESHOLD_METERS}m")
                    break
            else:
                print(f"[YOLO_INTERNAL] 目标 #{i}: 像素宽度非法")

        if found_close_light:
            return "Light On"
        else:
            return "Light Off"

    except Exception as e:
        print(f"[ERROR] 检测过程出错: {e}")
        return "Light Off"

# -----------------------------------------------------------------
# 4. 命令行入口 (供 subprocess 调用)
# -----------------------------------------------------------------
if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1:
        img_path = sys.argv[1]
        frame = cv2.imread(img_path)
        if frame is not None:
            # 执行识别并打印最终结果 (stdout 的最后一行)
            print(detect_yellow_light_state(frame))
        else:
            print("Light Off")