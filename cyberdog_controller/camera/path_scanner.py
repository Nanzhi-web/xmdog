import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import time

# ===== 基础类 =====
class PathScannerBase(Node):
    def __init__(self, node_name, done_callback):
        super().__init__(node_name)
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(Image, '/rgb_camera/image_raw', self.image_callback, qos)
        self.bridge = CvBridge()
        self.done_callback = done_callback
        self.center_x = 0
        self.img_height = 0

        # 预处理参数
        self.lower_yellow = np.array([20, 100, 100])
        self.upper_yellow = np.array([30, 255, 255])
        self.kernel_open = np.ones((5,5), np.uint8)
        self.kernel_close = np.ones((10,10), np.uint8)

    def image_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.img_height, self.center_x = cv_img.shape[0], cv_img.shape[1]//2
            
            # 预处理流水线
            hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel_close)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel_open)
            

            result = self.specialized_detect(cv_img , mask)
            self.done_callback(result)

        except Exception as e:
            self.get_logger().error(f"Processing error: {str(e)}")

    def specialized_detect(self, img, mask):
        raise NotImplementedError()

# ===== 专用检测器 =====
class FarDistanceScanner(PathScannerBase):
    """最远点距离检测"""
    def __init__(self, done_callback=lambda x: None):
        super().__init__("far_distance_scanner", done_callback)
    def specialized_detect(self, img, mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours: return {'far_distance': None}
        
        all_points = np.vstack([c.reshape(-1,2) for c in contours])
        min_y = np.min(all_points[:,1]) if all_points.size > 0 else self.img_height
        return {'far_distance': self.img_height - min_y}

class BottomLeftScanner(PathScannerBase):
    """底部左边界检测"""
    def __init__(self, done_callback=lambda x: None):
        super().__init__("bottom_left_scanner", done_callback)
    def specialized_detect(self, img, mask):
        img_height = mask.shape[0]
        bottom_row = img_height - 1
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        leftmost_x = None
        for contour in contours:
            for point in contour[:,0,:]:
                x, y = point
                if y == bottom_row:
                    leftmost_x = x if (leftmost_x is None or x < leftmost_x) else leftmost_x
        return {'left_bottom_distance': leftmost_x}

class SlopeScanner(PathScannerBase):
    def __init__(self, done_callback=lambda x: None):
        super().__init__("slope_scanner", done_callback)
        
    def specialized_detect(self, img, msk):
        """
        识别灰色斜坡的坡顶线并计算中心偏移
        
        参数:
            mask: 输入图像掩膜（灰度或二值）
            blank_threshold: 空白区域的像素占比阈值 (0-1)
            min_ramp_height: 最小有效斜坡高度（像素）
            
        返回:
            {
                'ramp_top_line': (y_position, (left_x, right_x)),  # 坡顶线信息
                'ramp_top_mid': (x, y),      # 坡顶线中点坐标
                'center_offset': offset,     # 中心线到坡顶线中点的水平偏移
                'ramp_height': height        # 斜坡高度（从坡底到坡顶）
            }
        """
        gray_low=70
        gray_high=120
        blank_threshold=0.03
        min_ramp_height=20
        # 确保输入是彩色图像
        if len(img.shape) == 2:  # 如果已经是灰度图
            gray = img
        else:
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        h, w = gray.shape
        center_x = w // 2
        
        # 创建灰色掩膜
        mask = np.zeros((h, w), dtype=np.uint8)
        mask[(gray >= gray_low) & (gray <= gray_high)] = 255
        # 调试：显示灰色掩膜
        cv2.imshow("Gray Mask", mask)
        cv2.waitKey(1)
        
        # 1. 垂直扫描寻找坡顶线（从顶部向下扫描）
        ramp_top_y = None
        ramp_bottom_y = h - 1
        
        for y in range(0, h):
            row = mask[y, :]
            non_zero_count = np.count_nonzero(row)
            non_zero_ratio = non_zero_count / w
            
            # 寻找灰色区域（非空白）
            if non_zero_ratio > blank_threshold:
                ramp_top_y = y
                break
        
        if ramp_top_y is None or ramp_top_y < min_ramp_height:
            return {
                'ramp_top_line': None,
                'ramp_top_mid': (center_x, 0),
                'center_offset': 0,
                'ramp_height': 0
            }
        
        # 2. 寻找坡底线（从坡顶线向下扫描）
        for y in range(ramp_top_y, h):
            row = mask[y, :]
            non_zero_count = np.count_nonzero(row)
            
            if non_zero_count / w < 0.2:  # 找到空白区域作为坡底
                ramp_bottom_y = y
                break
        
        # 3. 在坡顶线位置确定左右边界
        top_row = mask[ramp_top_y, :]
        gray_pixels = np.where(top_row > 0)[0]
        
        if gray_pixels.size == 0:
            left_edge = 0
            right_edge = w - 1
        else:
            left_edge = gray_pixels.min()
            right_edge = gray_pixels.max()
        
        # 4. 计算坡顶线中点
        mid_x = (left_edge + right_edge) // 2
        offset = mid_x - center_x
        
        # 5. 计算斜坡高度
        ramp_height = ramp_bottom_y - ramp_top_y
        
        return {
            'ramp_top_line': (ramp_top_y, (left_edge, right_edge)),
            'ramp_top_mid': (mid_x, ramp_top_y),
            'center_offset': offset,
            'ramp_height': ramp_height
        }

class RightBoundaryScanner(PathScannerBase):
    """右边界距离检测"""
    def __init__(self, done_callback=lambda x: None):
        super().__init__("right_boundary_scanner", done_callback)
    def specialized_detect(self,img, mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours: return {'right_boundary': None}
        
        all_points = np.vstack([c.reshape(-1,2) for c in contours])
        right_points = all_points[all_points[:,0] > self.center_x]
        if right_points.size == 0: return {'right_boundary': None}
        
        min_x = np.min(right_points[:,0])
        return {'right_boundary': mask.shape[1] - min_x}

class CenterOffsetScanner(PathScannerBase):
    """中心线偏移检测器（精准识别最近两侧赛道）"""
    def __init__(self, done_callback=lambda x: None):
        super().__init__("center_offset_scanner", done_callback)
        self.max_gap = 30      # 最大允许轮廓间距
        self.scan_y = 0

    def specialized_detect(self,img, mask):
        """简化版中心偏移检测"""
        if len(mask.shape) == 3:
            mask = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
        h, w = mask.shape
        self.center_x = w // 2
        self.scan_y = h * 5 //6 # 扫描线位置（可调）

        # 步骤1：在固定Y轴扫描赛道边界
        scan_line = mask[self.scan_y, :]
        white_pixels = np.where(scan_line == 255)[0]

        # 步骤2：分离左右边界点
        left_points = white_pixels[white_pixels < self.center_x]
        right_points = white_pixels[white_pixels > self.center_x]

        # 步骤3：取最近的左右边界
        left_edge = left_points.max() if left_points.size > 0 else None
        right_edge = right_points.min() if right_points.size > 0 else None

        # 步骤4：计算前视中点
        if left_edge and right_edge:
            mid = (left_edge + right_edge) // 2
            offset = mid - self.center_x
        elif left_points.size >= 2:  # 只有左边有边界且至少有两个点
            # 取左边距离中心线最近的两个点（最大的两个值）
            sorted_left = np.sort(left_points)
            right_edge = sorted_left[-1]  # 最近边界（最大值）设为右边界
            left_edge = sorted_left[-2]  # 第二近边界（次大值）设为左边界
            mid = (left_edge + right_edge) // 2
            offset = mid - self.center_x
        elif right_points.size >= 2:  # 只有右边有边界且至少有两个点
            # 取右边距离中心线最近的两个点（最小的两个值）
            sorted_right = np.sort(right_points)
            left_edge = sorted_right[0]  # 最近边界（最小值）设为左边界
            right_edge = sorted_right[1]  # 第二近边界（次小值）设为右边界
            mid = (left_edge + right_edge) // 2
            offset = mid - self.center_x
        else:
            mid, offset = self.center_x, 0

        # 步骤5：检测中心线交点（保持原有逻辑）
        center_intersection, center_dist = self._find_center_end(img,mask)

        return {
            'mid_point': (mid, self.scan_y),
            'center_offset': offset,
            'left_edge': left_edge,
            'right_edge': right_edge,
            'center_intersection': center_intersection,
            'center_distance': center_dist
        }

    def _find_center_end(self,img, mask):
        """中心线终点检测"""
        center_line = mask[:, self.center_x]
        white_pixels = np.where(center_line == 255)[0]
        if white_pixels.size == 0:
            return (self.center_x, mask.shape[0]-1), 0
        return (self.center_x, white_pixels[-1]), mask.shape[0]-1 - white_pixels[-1]


class HorizontalLineScanner(PathScannerBase):
    """增强版水平线检测器，排除底部短横线"""
    def __init__(self, done_callback=lambda x: None):
        super().__init__("horizontal_line_scanner", done_callback)
    def specialized_detect(self,img, mask):
        """
        基于轮廓线段的横向检测（高效直接版）
        """
        try:
            # 输入校验
            if mask is None or mask.size == 0:
                return {'is_horizontal_line': -1, 'angle': None, 'points': None}

            # 预处理
            if len(mask.shape) == 3:
                mask = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
            mask = mask.astype(np.uint8)
            img_h, img_w = mask.shape

        
            # 关键参数
            min_length = img_w * 0.25       # 线段最短长度
            max_angle = 15                  # 最大允许角度
            bottom_margin = 10              # 底边避让距离（像素）

            # 形态学处理
            kernel = np.ones((15,3), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            # 轮廓提取
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            best_segment = None
            best_angle = 90  # 初始化为最大角度
            max_length = 0

            for cnt in contours:
                # 多边形近似（减少点数）
                epsilon = 0.01 * cv2.arcLength(cnt, True)
                approx = cv2.approxPolyDP(cnt, epsilon, True)

                # 遍历所有线段
                for i in range(len(approx)):
                    # 获取线段端点
                    pt1 = approx[i][0]
                    pt2 = approx[(i+1)%len(approx)][0]
                    x1, y1 = pt1
                    x2, y2 = pt2

                    # 计算线段特征
                    dx = x2 - x1
                    dy = y2 - y1
                    length = np.hypot(dx, dy)
                    angle = np.degrees(np.arctan2(dy, dx))

                    # 角度标准化到-90~90度
                    if angle > 90:
                        angle -= 180
                    elif angle < -90:
                        angle += 180

                    # 筛选条件
                    if (abs(angle) > max_angle or
                        length < min_length or
                        max(y1, y2) > img_h - bottom_margin):
                        continue

                    # 更新最佳线段（优先角度，其次长度）
                    if abs(angle) < abs(best_angle) or (abs(angle) == abs(best_angle) and length > max_length):
                        best_angle = angle
                        max_length = length
                        best_segment = (x1, y1, x2, y2)

            # 结果处理
            if best_segment:
                x1, y1, x2, y2 = best_segment
                
                # 分类结果
                code = 3 if abs(best_angle) < 1 else (2 if best_angle > 0 else 1)
                
                if abs(best_angle) > 4:
                    code = -1
                
                return {
                    'is_horizontal_line': code,
                    'angle': float(best_angle),
                    'points': best_segment
                }
            else:
                return {'is_horizontal_line': -1, 'angle': None, 'points': None}

        except Exception as e:
            self.get_logger().error(f"检测异常: {str(e)}")
            return {'is_horizontal_line': -1, 'angle': None, 'points': None}

class BlackBarScanner(PathScannerBase):
    """黑色限高杆识别，只判断是否为水平杆"""
    def __init__(self, done_callback=lambda x: None):
        super().__init__("black_bar_scanner", done_callback)
        # 黑色阈值（可根据实际情况调整）
        self.lower_black = np.array([0, 0, 0])
        self.upper_black = np.array([180, 255, 60])

    def image_callback(self,img, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.img_height, self.center_x = cv_img.shape[0], cv_img.shape[1]//2

            hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, self.lower_black, self.upper_black)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel_open)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel_close)

            result = self.specialized_detect(mask)
            self.done_callback(result)
        except Exception as e:
            self.get_logger().error(f"Processing error: {str(e)}")

    def specialized_detect(self,img, mask):
        try:
            if mask is None or mask.size == 0:
                return {'is_horizontal_bar': -1}

            hough_params = {
                'rho': 1,
                'theta': np.pi/180,
                'threshold': 40,
                'minLineLength': int(mask.shape[1]*0.2),
                'maxLineGap': 15
            }

            lines = cv2.HoughLinesP(mask, **hough_params)
            if lines is None:
                return {'is_horizontal_bar': -1}

            candidates = []
            for line in lines:
                x1, y1, x2, y2 = line[0]
                if x1 > x2:
                    x1, x2 = x2, x1
                    y1, y2 = y2, y1
                dx = x2 - x1
                dy = y2 - y1
                angle = np.degrees(np.arctan2(dy, dx))
                if not (-10 <= angle <= 10):
                    continue
                y_avg = (y1 + y2) / 2
                candidates.append({'angle': angle, 'y_avg': y_avg})

            if not candidates:
                return {'is_horizontal_bar': -1}

            best = max(candidates, key=lambda x: x['y_avg'])
            angle = best['angle']

            if abs(angle) < 1.1:
                return {'is_horizontal_bar': 3 , 'angle': angle}
            elif angle >= 1.1:
                return {'is_horizontal_bar': 2 , 'angle': angle}
            else:
                return {'is_horizontal_bar': 1 , 'angle': angle}

        except Exception as e:
            self.get_logger().error(f"黑色限高杆检测异常: {str(e)}")
            return {'is_horizontal_bar': -1}

class CenterIntersectionScanner(PathScannerBase):
    """专用中心线交点检测器（检测赛道与图像中心线的交点）"""
    def __init__(self, done_callback=lambda x: None):
        super().__init__("center_intersection_scanner", done_callback)
        self.cluster_threshold = 5    # 纵向聚类间距阈值
        self.min_cluster_size = 3      # 有效聚类最小点数
        self.default_bottom = True    # 无检测时是否返回底部坐标

    def specialized_detect(self,img, mask):
        """返回格式：
        {
            'center_intersection': (x, y),  # 交点坐标
            'center_distance': int          # 距图像底部的距离
        }
        """
        # 预处理
        if len(mask.shape) == 3:
            mask = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
        img_h, img_w = mask.shape
        center_x = img_w // 2

        # 核心检测逻辑
        result = {
            'center_intersection': (center_x, img_h-1),  # 默认底部坐标
            'center_distance': 0
        }

        # 获取中心线像素
        center_line = mask[:, center_x]
        white_pixels = np.where(center_line == 255)[0]
        if white_pixels.size == 0:
            return result

        # 聚类处理（从下往上扫描）
        clusters = []
        current_cluster = []
        for y in reversed(np.sort(white_pixels)):  # 降序排列
            if not current_cluster:
                current_cluster.append(y)
            else:
                if current_cluster[-1] - y <= self.cluster_threshold:
                    current_cluster.append(y)
                else:
                    if len(current_cluster) >= self.min_cluster_size:
                        clusters.append(current_cluster)
                    current_cluster = [y]
        if current_cluster:  # 处理最后一个聚类
            clusters.append(current_cluster)

        # 选择有效聚类
        for cluster in clusters:
            if len(cluster) >= self.min_cluster_size:
                cluster_y = int(np.mean(cluster))
                result['center_intersection'] = (center_x, cluster_y)
                result['center_distance'] = img_h - 1 - cluster_y
                break  # 取第一个有效聚类

        return result

# ===== 新版主函数 =====
def path_scanner_main(scanner_type: type, timeout=5.0):
    """
    参数:
        scanner_type: 检测器类（必须继承自PathScannerBase）
        timeout: 超时时间（秒）
    """
    result = {}
    event = threading.Event()

    def on_detection(data):
        result.update(data)
        event.set()

    # rclpy.init()
    scanner = scanner_type(done_callback=on_detection)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(scanner)

    try:
        start_time = time.time()
        while not event.is_set() and (time.time() - start_time) < timeout:
            executor.spin_once(timeout_sec=0.1)
    except KeyboardInterrupt:
        scanner.get_logger().info('Scanner interrupted')
    finally:
        executor.remove_node(scanner)
        scanner.destroy_node()
        # rclpy.shutdown()
    
    if result:
        print("从路径扫描器获取的数据:", result)
        return result
    else:
        print("未识别到路径信息。")
        return None
    

if __name__ == '__main__':
    result = path_scanner_main()
    if result:
        # 在这里使用 QR 码内容
        print("正在开始调整:", result)