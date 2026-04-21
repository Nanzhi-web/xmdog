#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
【视觉逻辑模块 - 黄灯检测 - light_logic.py】

职责:
1. 加载 YOLOv8 模型 (best.pt) 用于定位灯牌。
2. 提供 `detect_yellow_light_state` 函数：
   - 使用 YOLO 找到灯牌 (ROI)。
   - 在 ROI 内部，使用 CV (颜色+圆检测) 找到 20cm 的灯。
   - 计算这个圆灯的距离。
   - 仅当距离 < 50cm 时返回 "Light On"。
"""

from ultralytics import YOLO
import os
import cv2 # OpenCV 还是需要的
import numpy as np
import time

# -----------------------------------------------------------------
#  关键参数
# -----------------------------------------------------------------

# --- 1. 物理参数  ---
# 灯牌直径 20cm = 0.2 米
KNOWN_WIDTH_METERS = 0.2 

# TODO:运行 OpenCV 相机标定，获取相机的焦距

FOCAL_LENGTH_PX = 700.0

# --- 2. 逻辑阈值  ---
# 设定的 50cm 距离阈值
DISTANCE_THRESHOLD_METERS = 0.5 

# YOLOv8 模型检测的置信度阈值 (0.0 到 1.0)
CONFIDENCE_THRESHOLD = 0.9

# --- 3. 传统CV (内部测量) 参数 ---

# 使用一个更宽的范围来容忍光照变化
LOWER_YELLOW_HSV = np.array([25, 150, 150])
UPPER_YELLOW_HSV = np.array([35, 255, 255]) # 稍微放宽H通道

# 霍夫圆检测的参数 (可能需要微调)
HOUGH_DP = 1          # 累加器分辨率 (1 = 与原图相同)
HOUGH_MIN_DIST = 40   # 圆心之间的最小距离
HOUGH_PARAM1 = 50     # Canny 边缘检测的高阈值
HOUGH_PARAM2 = 30     # 圆心检测的累加器阈值 (越小越容易检测到圆)
HOUGH_MIN_RADIUS = 5  # 最小半径
HOUGH_MAX_RADIUS = 0  # 最大半径 (0 = 不限制)

# -----------------------------------------------------------------
#模型加载设置
# -----------------------------------------------------------------

_THIS_SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
_MODEL_PATH = os.path.join(_THIS_SCRIPT_DIR, 'models', 'best.pt')
MODEL = None
MODEL_LOADED_SUCCESSFULLY = False

def _load_model_once():
    global MODEL, MODEL_LOADED_SUCCESSFULLY
    if MODEL_LOADED_SUCCESSFULLY:
        return
    if MODEL is not None and not MODEL_LOADED_SUCCESSFULLY:
        return

    try:
        if not os.path.exists(_MODEL_PATH):
            print("="*50)
            print(f"CRITICAL ERROR in light_logic: Model file not found!")
            print(f"Expected path: {_MODEL_PATH}")
            print("="*50)
            MODEL = "LOAD_FAILED" 
            return

        print(f"[light_logic] Loading YOLOv8 model from {_MODEL_PATH}...")
        MODEL = YOLO(_MODEL_PATH)
        MODEL_LOADED_SUCCESSFULLY = True
        print("[light_logic] YOLOv8 light model loaded successfully.")
        
        if FOCAL_LENGTH_PX == 700.0:
            print("="*50)
            print("WARNING in light_logic: Using default FOCAL_LENGTH_PX (700.0)!")
            print("This is a PLACEHOLDER. Your distance calculation will be WRONG.")
            print("You MUST calibrate your camera and update FOCAL_LENGTH_PX.")
            print("="*50)

    except Exception as e:
        print(f"[light_logic] Error loading model: {e}")
        MODEL = "LOAD_FAILED" # 标记为失败

# -----------------------------------------------------------------
#内部CV测量函数
# -----------------------------------------------------------------
def _find_light_and_distance_in_roi(roi: np.ndarray) -> float:
    """
    (CV 步骤) 在YOLO给的ROI中，找到黄灯圆并计算距离。
    返回: 距离(米)。如果找不到，返回 float('inf')
    """
    
   
    hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask_yellow = cv2.inRange(hsv_roi, LOWER_YELLOW_HSV, UPPER_YELLOW_HSV)
    
 
    kernel = np.ones((3,3), np.uint8)
    mask_yellow = cv2.morphologyEx(mask_yellow, cv2.MORPH_OPEN, kernel)
    mask_yellow = cv2.morphologyEx(mask_yellow, cv2.MORPH_CLOSE, kernel)

   
    circles = cv2.HoughCircles(mask_yellow, 
                               cv2.HOUGH_GRADIENT, 
                               dp=HOUGH_DP, 
                               minDist=HOUGH_MIN_DIST,
                               param1=HOUGH_PARAM1, 
                               param2=HOUGH_PARAM2, 
                               minRadius=HOUGH_MIN_RADIUS, 
                               maxRadius=HOUGH_MAX_RADIUS)
    
    if circles is not None:
        # 找到了一个或多个圆
        circles = np.uint16(np.around(circles))
        
        best_circle = circles[0, 0]
        pixel_radius = best_circle[2]
        pixel_width = float(pixel_radius * 2) # 直径

        if pixel_width > 0:
            # 3. 计算距离
            # 公式: Distance = (Known_Width * Focal_Length) / Pixel_Width
            distance_m = (KNOWN_WIDTH_METERS * FOCAL_LENGTH_PX) / pixel_width
            return distance_m
            
    # 如果霍夫圆检测失败, 尝试用旧的 "最大轮廓法" 作为备选
    contours, _ = cv2.findContours(mask_yellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        best_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(best_contour)
        if area > 10: # 至少有10个像素
             # 用边界矩形的宽度作为近似值
            _x, _y, pixel_width, _h = cv2.boundingRect(best_contour)
            pixel_width = float(pixel_width)
            
            if pixel_width > 0:
                distance_m = (KNOWN_WIDTH_METERS * FOCAL_LENGTH_PX) / pixel_width
                return distance_m

    # 如果 CV 两种方法都找不到，返回无限大距离
    return float('inf')


# -----------------------------------------------------------------
#  主检测函数 
# -----------------------------------------------------------------

def detect_yellow_light_state(cv_image: np.ndarray) -> str:
    """
    (YOLO + CV)
    检测图像中是否有黄灯亮起，并且距离小于 50cm。
    """
    global MODEL, MODEL_LOADED_SUCCESSFULLY
    
    # 1. 确保模型已加载
    _load_model_once()

    # 2. 检查模型是否加载失败
    if not MODEL_LOADED_SUCCESSFULLY:
        print("[light_logic] ERROR: YOLO model is not loaded. Diagnostic info:")
        print(f"  time: {time.strftime('%Y-%m-%d %H:%M:%S')}")
        print(f"  _MODEL_PATH: {_MODEL_PATH}")
        print(f"  model file exists: {os.path.exists(_MODEL_PATH)}")
        print(f"  MODEL value: {repr(MODEL)}")
        print(f"  MODEL_LOADED_SUCCESSFULLY: {MODEL_LOADED_SUCCESSFULLY}")
        print("  Action: verify model file, permissions, and that ultralytics is installed.")
        return "Light Off" 

    # 3. 运行YOLOv8预测 (步骤一：定位灯牌)
    try:
        results = MODEL(cv_image, conf=CONFIDENCE_THRESHOLD, verbose=False)
    except Exception as e:
        print(f"[light_logic] Error during model prediction: {e}")
        return "Light Off"

    # 4. 分析结果
    result = results[0]
    # --- 【调试代码】显示YOLO画出的所有框 ---
    
    # 调用 .plot() 方法生成带框的图像
    # im_with_boxes 是一个新的 BGR numpy 数组
    im_with_boxes = result.plot() 

  
    
    # 将图片保存到文件 
    
    debug_image_path = os.path.join(_THIS_SCRIPT_DIR, "yolo_all_boxes.jpg")
    
    cv2.imwrite(debug_image_path, im_with_boxes)
    print(f"[light_logic] 调试：YOLO的完整检测结果已保存到 {debug_image_path}")
    # 5. (步骤二：CV测量) 遍历YOLO找到的每个灯牌
    found_close_light = False
    
    for box in result.boxes:
        # 1. 获取灯牌的坐标 [x1, y1, x2, y2]
        xyxy = box.xyxy[0].cpu().numpy().astype(int) 
        x1, y1, x2, y2 = xyxy[0], xyxy[1], xyxy[2], xyxy[3]

        # 2. 提取灯牌的 ROI
        if x1 >= x2 or y1 >= y2:
            continue # 无效的框
        roi = cv_image[y1:y2, x1:x2]

        # 3. 在这个 ROI 内部，调用CV函数来查找圆灯并计算距离
        distance_m = _find_light_and_distance_in_roi(roi)
        
        # 4. 检查是否满足 < 50cm 阈值
        if distance_m < DISTANCE_THRESHOLD_METERS:
            # 找到了！
            found_close_light = True
            
            #  打印日志用于调试
            print(f"[light_logic] YOLO found board, CV found light at {distance_m:.2f} m. (PASSED < {DISTANCE_THRESHOLD_METERS}m)")
            
            break # 既然已找到一个，就退出循环
        else:
             found_close_light = True
            #  打印日志用于调试
             print(f"[light_logic] YOLO found board, CV found light at {distance_m:.2f} m. (FAILED < {DISTANCE_THRESHOLD_METERS}m)")
            
    # 6. 根据最终结果返回状态
    if found_close_light:
        return "Light On"
    else:
        return "Light Off"