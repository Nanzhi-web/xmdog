import cv2
import numpy as np

def _get_yellow_mask(frame):
    """私有方法：提取黄色掩膜并进行形态学闭合，抗干扰"""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # 放宽黄色阈值，适应仿真光线
    lower_yellow = np.array([15, 50, 50])
    upper_yellow = np.array([45, 255, 255])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    
    # 消除孤立噪点，连结断裂的线
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask

def check_horizontal(frame):
    """
    逻辑 A：检测远处平行横线
    通过宽高比判定是否为横线，并用拟合直线确保水平
    """
    res = {"is_horizontal": False, "center_offset": 0.0}
    if frame is None: return res

    mask = _get_yellow_mask(frame)
    height, width = mask.shape
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    horizontal_cnts = []
    for cnt in contours:
        x, y, w, h = cv2.boundingRect(cnt)
        # 条件：宽度是高度的2.5倍以上，且占据了屏幕至少20%宽度
        if w > h * 2.5 and w > width * 0.2:
            horizontal_cnts.append(cnt)

    if not horizontal_cnts: return res

    # 找到最宽的那条横线
    best_cnt = max(horizontal_cnts, key=lambda c: cv2.boundingRect(c)[2])
    
    # 拟合直线计算真实角度，允许 15 度以内的偏斜
    [vx, vy, x, y] = cv2.fitLine(best_cnt, cv2.DIST_L2, 0, 0.01, 0.01)
    angle = np.abs(np.arctan2(vy, vx) * 180 / np.pi)
    if angle > 90: angle = 180 - angle
    
    res["is_horizontal"] = bool(angle < 15.0)
    
    # 计算重心偏移
    M = cv2.moments(best_cnt)
    if M["m00"] != 0:
        cx = int(M["m10"] / M["m00"])
        res["center_offset"] = float((cx - width / 2.0) / (width / 2.0))

    return res

def detect_yellow_lines(frame):
    """
    逻辑 B：双侧黄线居中对齐
    将屏幕劈成两半，分别找左边线和右边线，计算几何中点
    """
    res = {"found": False, "center_offset": 0.0}
    if frame is None: return res

    mask = _get_yellow_mask(frame)
    height, width = mask.shape
    half_w = int(width / 2)
    
    left_mask = mask[:, :half_w]
    right_mask = mask[:, half_w:]

    def get_line_center(sub_mask):
        cnts, _ = cv2.findContours(sub_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts: return None
        c = max(cnts, key=cv2.contourArea)
        if cv2.contourArea(c) > 200: # 面积够大才算线
            M = cv2.moments(c)
            if M['m00'] != 0: return int(M['m10']/M['m00'])
        return None

    lx = get_line_center(left_mask)
    rx = get_line_center(right_mask)

    if lx is not None and rx is not None:
        # 两条线都在：取几何中心
        res["found"] = True
        real_rx = rx + half_w
        mid_point = (lx + real_rx) / 2.0
        res["center_offset"] = float((mid_point - half_w) / half_w)
    elif lx is not None:
        # 只看到左线：目标定在左线偏右
        res["found"] = True
        target = lx + half_w / 2.0
        res["center_offset"] = float((target - half_w) / half_w)
    elif rx is not None:
        # 只看到右线：目标定在右线偏左
        res["found"] = True
        real_rx = rx + half_w
        target = real_rx - half_w / 2.0
        res["center_offset"] = float((target - half_w) / half_w)

    return res