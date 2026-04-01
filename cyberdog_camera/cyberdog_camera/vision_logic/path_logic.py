import cv2
import numpy as np

def detect_yellow_path_info(img):
    def to_value(x):
        return x if x is not None else 9999

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([30, 255, 255])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    kernel = np.ones((7, 7), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    h, w = mask.shape
    center_x = w // 2

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return {
            "left_distance": 9999,
            "right_distance": 9999,
            "front_distance": 9999,
            "right_slope_angle": 9999,
            "front_slope_angle": 9999
        }
    all_points = np.vstack([cnt.reshape(-1, 2) for cnt in contours if cnt.shape[1] == 1])

    # 离狗最近的黄线（扩大到下半幅）
    bottom_half_mask = mask[h//2:, :]
    ylist, xlist = np.where(bottom_half_mask > 0)
    if len(ylist) > 0:
        max_y = ylist.max()
        real_y = max_y + h // 2
        points_at_max_y = xlist[ylist == max_y]
        front_distance = np.min(np.abs(points_at_max_y - center_x)) if len(points_at_max_y) > 0 else 9999
    else:
        front_distance = 9999

    # front_slope_angle（下半部分所有黄线点拟合直线夹角）
    front_slope_angle = 9999
    if len(xlist) >= 10:
        fit_points = np.column_stack((xlist, ylist + h // 2))  # 转回原图坐标
        vx, vy, x0, y0 = cv2.fitLine(fit_points, cv2.DIST_L2, 0, 0.01, 0.01)
        angle = np.degrees(np.arctan2(vy, vx))
        front_slope_angle = float(angle)

    # 左距离
    center_y = h - 10
    scan_line = mask[center_y:center_y+1, :]
    white_x = np.where(scan_line[0] > 0)[0]
    left_distance = None
    left_x = white_x[white_x < center_x]
    if len(left_x) > 0:
        left_distance = center_x - left_x.max()

    # 右距离+右角度
    right_distance = None
    right_slope_angle = None
    right_x = white_x[white_x > center_x]
    if len(right_x) > 0:
        right_distance = right_x.min() - center_x
        right_mask = np.zeros_like(mask)
        right_mask[center_y:, center_x:] = mask[center_y:, center_x:]
        right_points = np.column_stack(np.where(right_mask > 0))
        if len(right_points) >= 10:
            vx, vy, x0, y0 = cv2.fitLine(right_points, cv2.DIST_L2, 0, 0.01, 0.01)
            angle = np.degrees(np.arctan2(vy, vx))
            right_slope_angle = float(angle)
        else:
            right_slope_angle = None

    return {
        "left_distance": to_value(left_distance),
        "right_distance": to_value(right_distance),
        "front_distance": to_value(front_distance),
        "right_slope_angle": to_value(right_slope_angle),
        "front_slope_angle": to_value(front_slope_angle)
    }

