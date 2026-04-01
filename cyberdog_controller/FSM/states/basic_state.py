from abc import ABC
import time
from locomotion import LocomotionController
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
import threading
from camera.path_scanner import *

class SimulationClock(Node):
    def __init__(self):
        super().__init__('simulation_clock_listener')
        self._sim_time = None
        self._lock = threading.Lock()
        self._sub = self.create_subscription(Clock, '/clock', self._clock_callback, 10)

    def _clock_callback(self, msg):
        with self._lock:
            self._sim_time = msg.clock

    def get_sim_time(self):
        with self._lock:
            return self._sim_time


class Basic_State(ABC):
    def __init__(self):
        self.name = "Basic_State"
        self.motion_id = 0
        self.duration = -1

        self.locomotion = LocomotionController()

    def execute(self):
        rclpy.init()
        sim_clock = SimulationClock()

        # 启动一个线程运行 ROS node（非阻塞）
        thread = threading.Thread(target=rclpy.spin, args=(sim_clock,), daemon=True)
        thread.start()

        time.sleep(1)  # 等待订阅者连接

        try:
            print(f"Executing {self.name} with motion ID {self.motion_id} for duration {self.duration}")
            # start_time = time.time()
            _start_time = sim_clock.get_sim_time()
            start_time = _start_time.nanosec / 1e9 + _start_time.sec
            current_time = start_time
            # print(f"Start time: {start_time}")
            while True:
                self.locomotion.set_motion(self.motion_id)
                time.sleep(0.005)  # 发布频率
                # if time.time() - start_time >= self.duration:
                _current_time = sim_clock.get_sim_time()
                current_time = _current_time.nanosec / 1e9 + _current_time.sec
                # print(f"Current simulation time: {current_time}")
                if current_time - start_time >= self.duration:
                    break
            print(f"Finished executing {self.name} in {current_time - start_time:.2f} seconds")
        finally:
            sim_clock.destroy_node()
            rclpy.shutdown()



class Standing(Basic_State):
    """
    站立
    """
    def __init__(self, duration=0):
        super().__init__()
        self.name = "Standing"
        self.motion_id = 1
        self.duration = duration


class Laying(Basic_State):
    """
    趴下
    """
    def __init__(self, duration=0):
        super().__init__()
        self.name = "Laying"
        self.motion_id = 3
        self.duration = duration


class Walking_Forward(Basic_State):
    """
    前进
    """
    def __init__(self, duration=0):
        super().__init__()
        self.name = "Walking Forward"
        self.motion_id = 5
        self.duration = duration

class Walking_Forward_Slow(Basic_State):
    """
    前进，很慢很慢，正常速度1/2
    """
    def __init__(self, duration=0):
        super().__init__()
        self.name = "Walking Forward"
        self.motion_id = 16
        self.duration = duration

class Walking_Forward_by_px(Basic_State):
    '''
    动态的根据目前视野最远点的距离，来判断停下的时间，传入参数是停下时最远点距离。
    @param duration: 最大值，达不到要求时作用
    @param px: 距离最远点的距离
    '''
    def __init__(self, duration, px):
        super().__init__()
        self.name = "Walking Forward by px"
        self.duration = duration
        self.px = px
        self.flag = 0
        

    def execute(self):
        print(f"Executing {self.name} with px threshold: {self.px}")
        rclpy.init()
        sim_clock = SimulationClock()

        # 启动一个线程运行 ROS node（非阻塞）
        thread = threading.Thread(target=rclpy.spin, args=(sim_clock,), daemon=True)
        thread.start()

        time.sleep(1)  # 等待订阅者连接
        # 动态获取最新检测方法
        
        try:    
            _start_time = sim_clock.get_sim_time()
            start_time = _start_time.nanosec / 1e9 + _start_time.sec
            current_time = start_time
            while True:
                time.sleep(0.01)
                _current_time = sim_clock.get_sim_time()
                current_time = _current_time.nanosec / 1e9 + _current_time.sec
                    
                if current_time - start_time >= self.duration:
                    break
                try:
                    data = path_scanner_main(FarDistanceScanner, timeout=0.5)  # 动态调用
                    distance = data.get('far_distance', 0)
                    
                    if distance > self.px:
                        self.locomotion.set_motion(5)
                        time.sleep(0.01)
                    elif self.flag == 1:
                        print(f"Finished executing {self.name} in {current_time - start_time:.2f} seconds")
                        break

                    if distance < self.px:
                        self.flag = 1
                except Exception as e:
                    print(f"检测异常: {str(e)}")
                    break
        finally:
            sim_clock.destroy_node()
            rclpy.shutdown()


class Walking_Forward_by_rightbound(Basic_State):
    '''
    前进，检测右边界，前进至右边界小于设定值。这个也是曲线入口专用，
    根据视野内右边界与赛道的距离判断,
    注意不要和下面的Walking_Forward_by_bottom混淆，
    这个是根据整个赛道判断的，下面的是根据底边相接赛道处判断的
    '''
    def __init__(self, duration,right_bound):
        super().__init__()
        self.name = "Walking Forward by rightbound"
        self.duration = duration  
        self.flag = 0
        self.right_bound = right_bound

    def execute(self):
        print(f"Executing {self.name} with distance threshold: {self.duration}")
        rclpy.init()
        sim_clock = SimulationClock()

        # 启动一个线程运行 ROS node（非阻塞）
        thread = threading.Thread(target=rclpy.spin, args=(sim_clock,), daemon=True)
        thread.start()

        time.sleep(1)  # 等待订阅者连接

        try:
            _start_time = sim_clock.get_sim_time()
            start_time = _start_time.nanosec / 1e9 + _start_time.sec
            current_time = start_time
            while True:
                time.sleep(0.01)
                _current_time = sim_clock.get_sim_time()
                current_time = _current_time.nanosec / 1e9 + _current_time.sec
                if current_time - start_time >= self.duration:
                    break
                try:
                    data = path_scanner_main(RightBoundaryScanner, timeout=0.5)  # 动态调用
                    right_bound= data.get('right_boundary', 0)
                    if self.right_bound <= right_bound:
                        self.locomotion.set_motion(5)
                        time.sleep(0.01)
                    else : 
                        print(f"Finished executing {self.name} in {current_time - start_time:.2f} seconds")
                        break
                except Exception as e:
                    print(f"检测异常: {str(e)}")
                    break
        
        finally:
            sim_clock.destroy_node()
            rclpy.shutdown()

class Walking_Stone_by_distance(Basic_State):
    '''
    限高杆->石板路的石板路
    '''
    def __init__(self, duration,distance):
        super().__init__()
        self.name = "Walking Forward by distance"
        self.duration = duration  
        self.flag = 0
        self.distance = distance

    def execute(self):
        print(f"Executing {self.name} with distance threshold: {self.duration}")
        rclpy.init()
        sim_clock = SimulationClock()

        # 启动一个线程运行 ROS node（非阻塞）
        thread = threading.Thread(target=rclpy.spin, args=(sim_clock,), daemon=True)
        thread.start()

        time.sleep(1)  # 等待订阅者连接

        try:
            _start_time = sim_clock.get_sim_time()
            start_time = _start_time.nanosec / 1e9 + _start_time.sec
            current_time = start_time
            while True:
                time.sleep(0.01)
                _current_time = sim_clock.get_sim_time()
                current_time = _current_time.nanosec / 1e9 + _current_time.sec
                if current_time - start_time >= self.duration:
                    break
                try:
                    data = path_scanner_main(CenterOffsetScanner, timeout=0.5)  # 动态调用
                    distance= data.get('center_distance', 0)
                    if distance <= self.distance :
                        self.locomotion.set_motion(16)
                        time.sleep(0.01)
                    else : 
                        print(f"Finished executing {self.name} in {current_time - start_time:.2f} seconds")
                        break
                except Exception as e:
                    print(f"检测异常: {str(e)}")
                    break
        
        finally:
            sim_clock.destroy_node()
            rclpy.shutdown()

class Walking_Forward_by_distance_z(Basic_State):
    '''
    使用离得最近的边线的距离，
    @param duration: 最大值，达不到要求时作用
    @param distance: 距离最近赛道的距离
    '''
    def __init__(self, duration,distance):
        super().__init__()
        self.name = "Walking Forward by distance"
        self.duration = duration  
        self.flag = 0
        self.distance = distance

    def execute(self):
        print(f"Executing {self.name} with distance threshold: {self.duration}")
        rclpy.init()
        sim_clock = SimulationClock()

        # 启动一个线程运行 ROS node（非阻塞）
        thread = threading.Thread(target=rclpy.spin, args=(sim_clock,), daemon=True)
        thread.start()

        time.sleep(1)  # 等待订阅者连接

        try:
            _start_time = sim_clock.get_sim_time()
            start_time = _start_time.nanosec / 1e9 + _start_time.sec
            current_time = start_time
            while True:
                time.sleep(0.01)
                _current_time = sim_clock.get_sim_time()
                current_time = _current_time.nanosec / 1e9 + _current_time.sec
                if current_time - start_time >= self.duration:
                    break
                try:
                    data = path_scanner_main(CenterOffsetScanner, timeout=0.5)  # 动态调用
                    distance= data.get('center_distance', 0)
                    if  self.distance < distance:
                        self.locomotion.set_motion(5)
                        time.sleep(0.01)
                    elif distance == 0 : 
                        print(f"Finished executing {self.name} in {current_time - start_time:.2f} seconds")
                        break
                    else : 
                        print(f"Finished executing {self.name} in {current_time - start_time:.2f} seconds")
                        break
                except Exception as e:
                    print(f"检测异常: {str(e)}")
                    break
        
        finally:
            sim_clock.destroy_node()
            rclpy.shutdown()


class Walking_Slope_by_distance(Basic_State):
    '''
    走斜坡，动态调整走坡时间
    '''
    def __init__(self, duration,distance):
        super().__init__()
        self.name = "Walking Forward by distance"
        self.duration = duration  
        self.flag = 0
        self.distance = distance

    def execute(self):
        print(f"Executing {self.name} with distance threshold: {self.duration}")
        rclpy.init()
        sim_clock = SimulationClock()

        # 启动一个线程运行 ROS node（非阻塞）
        thread = threading.Thread(target=rclpy.spin, args=(sim_clock,), daemon=True)
        thread.start()

        time.sleep(1)  # 等待订阅者连接

        try:
            _start_time = sim_clock.get_sim_time()
            start_time = _start_time.nanosec / 1e9 + _start_time.sec
            current_time = start_time
            while True:
                time.sleep(0.01)
                _current_time = sim_clock.get_sim_time()
                current_time = _current_time.nanosec / 1e9 + _current_time.sec
                if current_time - start_time >= self.duration:
                    break
                try:
                    data = path_scanner_main(CenterOffsetScanner, timeout=0.5)  # 动态调用
                    distance= data.get('center_distance', 0)
                    offset = data.get('center_offset', 0)
                    if distance <= self.distance or distance == None:
                        self.locomotion.set_motion(19)
                        time.sleep(0.01)
                        self.flag = 0
                    elif self.flag == 1: 
                        print(f"Finished executing {self.name} in {current_time - start_time:.2f} seconds")
                        break
                    if distance > self.distance and offset != 0 and offset < 50 and offset > -50:
                        self.flag = 1
                except Exception as e:
                    print(f"检测异常: {str(e)}")
                    break
        
        finally:
            sim_clock.destroy_node()
            rclpy.shutdown()



class Spin_by_line(Basic_State):
    '''
    原地旋转，根据眼前横向（小于10度）线条判断，然后决定旋转方向，顺、逆旋转至横线小于1.5度
    '''
    def __init__(self, duration):
        super().__init__()
        self.name = "Spin by line"
        self.duration = duration
        self.flag = 0

    def execute(self):
        print(f"Executing {self.name} with distance threshold: {self.duration}")
        
        rclpy.init()
        sim_clock = SimulationClock()

        # 启动一个线程运行 ROS node（非阻塞）
        thread = threading.Thread(target=rclpy.spin, args=(sim_clock,), daemon=True)
        thread.start()

        time.sleep(1)  # 等待订阅者连接
        try:    
            _start_time = sim_clock.get_sim_time()
            start_time = _start_time.nanosec / 1e9 + _start_time.sec
            current_time = start_time
            while True:
                time.sleep(0.02)
                _current_time = sim_clock.get_sim_time()
                current_time = _current_time.nanosec / 1e9 + _current_time.sec
                if current_time - start_time >= self.duration:
                    break
                try:
                    data = path_scanner_main(HorizontalLineScanner, timeout=0.5)  # 动态调用
                    is_horizontal_line= data.get('is_horizontal_line', 0)
                    if is_horizontal_line == 2:
                        self.locomotion.set_motion(12)
                    elif is_horizontal_line == 1:
                        self.locomotion.set_motion(11)
                    elif is_horizontal_line == 3:
                        print(f"Finished executing {self.name} in {current_time - start_time:.2f} seconds")
                        break
                    else : 
                        print(f"Finished executing {self.name} in {current_time - start_time:.2f} seconds")
                        break
                except Exception as e:
                    print(f"检测异常: {str(e)}")
                    break
        finally:
            sim_clock.destroy_node()
            rclpy.shutdown()   


class Walking_by_black_bar(Basic_State):
    '''
    石板路->限高杆，根据限高杆走
    '''
    def __init__(self, duration):
        super().__init__()
        self.name = "Spin by black bar"
        self.duration = duration
        self.flag = 0

    def execute(self):
        print(f"Executing {self.name} with distance threshold: {self.duration}")
        
        rclpy.init()
        sim_clock = SimulationClock()

        # 启动一个线程运行 ROS node（非阻塞）
        thread = threading.Thread(target=rclpy.spin, args=(sim_clock,), daemon=True)
        thread.start()

        time.sleep(1)  # 等待订阅者连接
        try:    
            _start_time = sim_clock.get_sim_time()
            start_time = _start_time.nanosec / 1e9 + _start_time.sec
            current_time = start_time
            while True:
                time.sleep(0.01)
                _current_time = sim_clock.get_sim_time()
                current_time = _current_time.nanosec / 1e9 + _current_time.sec
                if current_time - start_time >= self.duration:
                    break
                try:
                    data = path_scanner_main(BlackBarScanner, timeout=0.5)  # 动态调用
                    is_horizontal_line= data.get('is_horizontal_bar', 0)
                    if is_horizontal_line == -1:
                        print(f"Finished executing {self.name} in {current_time - start_time:.2f} seconds")
                        break
                    else :
                        self.locomotion.set_motion(5)
                    
                except Exception as e:
                    print(f"检测异常: {str(e)}")
                    break
        finally:
            sim_clock.destroy_node()
            rclpy.shutdown()   

class Walking_Backward(Basic_State):
    """
    后退
    """
    def __init__(self, duration=0):
        super().__init__()
        self.name = "Walking Backward"
        self.motion_id = 6
        self.duration = duration


class Walking_Backward_by_distance(Basic_State):
    '''
    后退，根据目前视野最远点的距离，来判断停下的时间，传入参数是停下时最远点距离。
    '''
    def __init__(self, duration,distance):
        super().__init__()
        self.name = "Walking Forward by distance"
        self.duration = duration  
        self.flag = 0
        self.distance = distance

    def execute(self):
        print(f"Executing {self.name} with distance threshold: {self.duration}")
        rclpy.init()
        sim_clock = SimulationClock()

        # 启动一个线程运行 ROS node（非阻塞）
        thread = threading.Thread(target=rclpy.spin, args=(sim_clock,), daemon=True)
        thread.start()

        time.sleep(1)  # 等待订阅者连接

        try:
            _start_time = sim_clock.get_sim_time()
            start_time = _start_time.nanosec / 1e9 + _start_time.sec
            current_time = start_time
            while True:
                time.sleep(0.01)
                _current_time = sim_clock.get_sim_time()
                current_time = _current_time.nanosec / 1e9 + _current_time.sec
                if current_time - start_time >= self.duration:
                    break
                try:
                    data = path_scanner_main(CenterOffsetScanner, timeout=0.5)  # 动态调用
                    distance= data.get('center_distance', 0)
                    offset = data.get('center_offset', 0)
                    if distance <= self.distance or distance == None:
                        self.locomotion.set_motion(6)
                        time.sleep(0.01)
                        self.flag = 0
                    elif self.flag == 1: 
                        print(f"Finished executing {self.name} in {current_time - start_time:.2f} seconds")
                        break
                    if distance > self.distance and offset != 0 and offset < 50 and offset > -50:
                        self.flag = 1
                except Exception as e:
                    print(f"检测异常: {str(e)}")
                    break
        
        finally:
            sim_clock.destroy_node()
            rclpy.shutdown()

class Shift_Left(Basic_State):
    """
    向左平移，非前进左转
    """
    def __init__(self, duration=0):
        super().__init__()
        self.name = "Shift Left"
        self.motion_id = 7
        self.duration = duration
    
class Shift_Left_by_bottom(Basic_State):
    def __init__(self, duration , distance ):
        """
        向左平移，非前进左转，目前是曲线入口专用，根据视野内底边赛道与左边界的距离判断
        """
        super().__init__()
        self.name = "Shift Left by bottom"
        self.motion_id = 7
        self.duration = duration
        self.distance = distance

    def execute(self):
        print(f"Executing {self.name} with distance threshold: {self.distance}")
        
        rclpy.init()
        sim_clock = SimulationClock()

        # 启动一个线程运行 ROS node（非阻塞）
        thread = threading.Thread(target=rclpy.spin, args=(sim_clock,), daemon=True)
        thread.start()

        time.sleep(1)  # 等待订阅者连接
        try:
            _start_time = sim_clock.get_sim_time()
            start_time = _start_time.nanosec / 1e9 + _start_time.sec
            current_time = start_time
            while True:
                time.sleep(0.01)
                _current_time = sim_clock.get_sim_time()
                current_time = _current_time.nanosec / 1e9 + _current_time.sec
                if current_time - start_time >= self.duration:
                    break
                try:
                    data = path_scanner_main(BottomLeftScanner  , timeout=0.5)  # 动态调用
                    distance = data.get('left_bottom_distance', 0)
                    if distance <= self.distance and distance != 0:
                        self.locomotion.set_motion(13)
                        time.sleep(0.01)
                    else : 
                        print(f"Finished executing {self.name} in {current_time - start_time:.2f} seconds")
                        break
                except Exception as e:
                    print(f"检测异常: {str(e)}")
                break
        finally:
            sim_clock.destroy_node()
            rclpy.shutdown()
            
class Shift_Right_by_bottom(Basic_State):
    def __init__(self, duration , distance):
        """
        向右平移，非前进右转，目前是曲线入口专用，根据视野内底边赛道与右边界的距离判断
        """
        super().__init__()
        self.name = "Shift Right by bottom"
        self.motion_id = 8
        self.duration = duration
        self.distance = distance

    def execute(self):
        print(f"Executing {self.name} with distance threshold: {self.distance}")
       
        rclpy.init()
        sim_clock = SimulationClock()

        # 启动一个线程运行 ROS node（非阻塞）
        thread = threading.Thread(target=rclpy.spin, args=(sim_clock,), daemon=True)
        thread.start()

        time.sleep(1)  # 等待订阅者连接
        try:
            _start_time = sim_clock.get_sim_time()
            start_time = _start_time.nanosec / 1e9 + _start_time.sec
            current_time = start_time
            while True:
                time.sleep(0.01)
                _current_time = sim_clock.get_sim_time()
                current_time = _current_time.nanosec / 1e9 + _current_time.sec
                if current_time - start_time >= self.duration:
                    break
                try:
                    data = path_scanner_main(BottomLeftScanner  , timeout=0.5)  # 动态调用
                    distance = data.get('left_bottom_distance', 0)
                    if distance >= self.distance:
                        self.locomotion.set_motion(8)
                    else: 
                        print(f"Finished executing {self.name} in {current_time - start_time:.2f} seconds")
                        break
                except Exception as e:
                    print(f"检测异常: {str(e)}")
                    break
        finally:
            sim_clock.destroy_node()
            rclpy.shutdown()

class Shift_Right(Basic_State):
    """
    向右平移，非前进右转
    """
    def __init__(self, duration=0):
        super().__init__()
        self.name = "Shift Right"
        self.motion_id = 8
        self.duration = duration

class Shift_by_angle(Basic_State):
    '''
    有调整的原地向右转
    '''
    def __init__(self, duration):
        super().__init__()
        self.name = "Shift by angle"
        self.motion_id = 7
        self.duration = duration
        self.flag = 0

    def execute(self):
        print(f"Executing {self.name}")
        rclpy.init()
        sim_clock = SimulationClock()

        # 启动一个线程运行 ROS node（非阻塞）
        thread = threading.Thread(target=rclpy.spin, args=(sim_clock,), daemon=True)
        thread.start()

        time.sleep(1)  # 等待订阅者连接
        try:
            _start_time = sim_clock.get_sim_time()
            start_time = _start_time.nanosec / 1e9 + _start_time.sec
            current_time = start_time
            while True:
                time.sleep(0.01)
                _current_time = sim_clock.get_sim_time()
                current_time = _current_time.nanosec / 1e9 + _current_time.sec
                if current_time - start_time >= self.duration:
                    break
                try:
                    data = path_scanner_main(CenterOffsetScanner, timeout=0.5)  # 动态调用
                    distance_to_center = data.get('center_offset', 0)
                    if distance_to_center <= 3 and distance_to_center >= -3:
                        print(f"Finished executing {self.name} in {current_time - start_time:.2f} seconds")
                        break
                    if distance_to_center < -3 :
                        self.locomotion.set_motion(13)
                        time.sleep(0.01)
                    if distance_to_center > 3:
                        self.locomotion.set_motion(14)
                        time.sleep(0.01)
                except Exception as e:
                    print(f"检测异常: {str(e)}")
                    break
        finally:
            sim_clock.destroy_node()
            rclpy.shutdown()

class Spin_Left(Basic_State):
    """
    原地向左转
    """
    def __init__(self, duration=0):
        super().__init__()
        self.name = "Spin Left"
        self.motion_id = 9
        self.duration = duration


class Spin_Right(Basic_State):
    """
    原地向右转
    """
    def __init__(self, duration=0):
        super().__init__()
        self.name = "Spin Right"
        self.motion_id = 10
        self.duration = duration

class Spin_by_angle(Basic_State):
    '''
    有调整的原地向右转
    '''
    def __init__(self, duration):
        super().__init__()
        self.name = "Spin Right by angle"
        self.motion_id = 10
        self.duration = duration
        self.flag = 0

    def execute(self):
        print(f"Executing {self.name}")
        rclpy.init()
        sim_clock = SimulationClock()

        # 启动一个线程运行 ROS node（非阻塞）
        thread = threading.Thread(target=rclpy.spin, args=(sim_clock,), daemon=True)
        thread.start()

        time.sleep(1)  # 等待订阅者连接
        try:
            _start_time = sim_clock.get_sim_time()
            start_time = _start_time.nanosec / 1e9 + _start_time.sec
            current_time = start_time
            while True:
                time.sleep(0.01)
                _current_time = sim_clock.get_sim_time()
                current_time = _current_time.nanosec / 1e9 + _current_time.sec
                if current_time - start_time >= self.duration:
                    break
                try:
                    data = path_scanner_main(CenterOffsetScanner, timeout=0.5)  # 动态调用
                    distance_to_center = data.get('center_offset', 0)

                    if distance_to_center <= 5 and distance_to_center >= -5:
                        print(f"Finished executing {self.name} in {current_time - start_time:.2f} seconds")
                        break
                    if distance_to_center <= -4 :
                        self.locomotion.set_motion(11)
                        time.sleep(0.01)
                    if distance_to_center >= 4:
                        self.locomotion.set_motion(12)
                except Exception as e:
                    print(f"检测异常: {str(e)}")
                    break
        finally:
            sim_clock.destroy_node()
            rclpy.shutdown()

class Spin_by_slope(Basic_State):
    '''
    坡底使用，旋转调整
    '''
    def __init__(self, duration):
        super().__init__()
        self.name = "Spin Slope by angle"
        self.motion_id = 10
        self.duration = duration
        self.flag = 0

    def execute(self):
        print(f"Executing {self.name}")
        rclpy.init()
        sim_clock = SimulationClock()

        # 启动一个线程运行 ROS node（非阻塞）
        thread = threading.Thread(target=rclpy.spin, args=(sim_clock,), daemon=True)
        thread.start()

        time.sleep(1)  # 等待订阅者连接
        try:
            _start_time = sim_clock.get_sim_time()
            start_time = _start_time.nanosec / 1e9 + _start_time.sec
            current_time = start_time
            while True:
                time.sleep(0.01)
                _current_time = sim_clock.get_sim_time()
                current_time = _current_time.nanosec / 1e9 + _current_time.sec
                if current_time - start_time >= self.duration:
                    break
                try:
                    data = path_scanner_main(SlopeScanner, timeout=0.5)  # 动态调用
                    distance_to_center = data.get('center_offset', 0)

                    if distance_to_center < 9 and distance_to_center > -9:
                        print(f"Finished executing {self.name} in {current_time - start_time:.2f} seconds")
                        break
                    if distance_to_center <= -8 :
                        self.locomotion.set_motion(11)
                        time.sleep(0.01)
                    if distance_to_center >= 8:
                        self.locomotion.set_motion(12)
                except Exception as e:
                    print(f"检测异常: {str(e)}")
                    break
        finally:
            sim_clock.destroy_node()
            rclpy.shutdown()

class Shift_by_slope(Basic_State):
    '''
    坡底使用，平移调整
    '''
    def __init__(self, duration):
        super().__init__()
        self.name = "Shift Slope by angle"
        self.motion_id = 10
        self.duration = duration
        self.flag = 0

    def execute(self):
        print(f"Executing {self.name}")
        rclpy.init()
        sim_clock = SimulationClock()

        # 启动一个线程运行 ROS node（非阻塞）
        thread = threading.Thread(target=rclpy.spin, args=(sim_clock,), daemon=True)
        thread.start()

        time.sleep(1)  # 等待订阅者连接
        try:
            _start_time = sim_clock.get_sim_time()
            start_time = _start_time.nanosec / 1e9 + _start_time.sec
            current_time = start_time
            while True:
                time.sleep(0.01)
                _current_time = sim_clock.get_sim_time()
                current_time = _current_time.nanosec / 1e9 + _current_time.sec
                if current_time - start_time >= self.duration:
                    break
                try:
                    data = path_scanner_main(SlopeScanner, timeout=0.5)  # 动态调用
                    distance_to_center = data.get('center_offset', 0)

                    if distance_to_center < 12 and distance_to_center > -12:
                        print(f"Finished executing {self.name} in {current_time - start_time:.2f} seconds")
                        break
                    if distance_to_center <= -13 :
                        self.locomotion.set_motion(13)
                        time.sleep(0.01)
                    if distance_to_center >= 13:
                        self.locomotion.set_motion(14)
                except Exception as e:
                    print(f"检测异常: {str(e)}")
                    break
        finally:
            sim_clock.destroy_node()
            rclpy.shutdown()

class Walking_Left_Turn(Basic_State):
    """
    前进左转（曲线）
    """
    def __init__(self, duration=0):
        super().__init__()
        self.name = "Walking Left Turn"
        self.motion_id = 18
        self.duration = duration


class Walking_Right_Turn(Basic_State):
    """
    前进右转（曲线）
    """
    def __init__(self, duration=0):
        super().__init__()
        self.name = "Walking Right Turn"
        self.motion_id = 15
        self.duration = duration


class Walking_Slope(Basic_State):
    """
    上坡下坡
    """
    def __init__(self, duration=0):
        super().__init__()
        self.name = "Walking Slope"
        self.motion_id = 19
        self.duration = duration


class Walking_High_Limiter(Basic_State):
    """
    限高杆行走
    """
    def __init__(self, duration=0):
        super().__init__()
        self.name = "Walking High Limiter"
        self.motion_id = 20
        self.duration = duration

# class Check_forward_point(Basic_State):
#     def __init__(self, duration=0):
#         super().__init__()
#         self.name = "Checking_forward_point"
#         self.scanner = None
#         self.result = None
#         self.duration = 5
    
    
#     def get_motion(self,duration):
#         while True:
#             data = shared_data.get()
            
#             # 检查数据时效性（1秒内有效）
#             if time.time() - data['timestamp'] > 1.0:
#                 print("等待有效数据...")
#                 time.sleep(0.1)
#                 continue
                
#             farthest_info = data['farthest_info']
#             if farthest_info and farthest_info[1] < duration:
#                 self.motion_id = 1  # 到达位置
#             else:
#                 self.motion_id = 5  # 继续前进
        

# class Walking_Forward_by_distance(Basic_State):
#     '''
    
#     '''
#     def __init__(self, duration,distance):
#         super().__init__()
#         self.name = "Walking Forward by distance"
#         self.duration = duration  
#         self.flag = 0
#         self.distance = distance

#     def execute(self):
#         print(f"Executing {self.name} with distance threshold: {self.duration}")
#         rclpy.init()
#         sim_clock = SimulationClock()

#         # 启动一个线程运行 ROS node（非阻塞）
#         thread = threading.Thread(target=rclpy.spin, args=(sim_clock,), daemon=True)
#         thread.start()

#         time.sleep(1)  # 等待订阅者连接

#         try:
#             _start_time = sim_clock.get_sim_time()
#             start_time = _start_time.nanosec / 1e9 + _start_time.sec
#             current_time = start_time
#             while True:
#                 time.sleep(0.01)
#                 _current_time = sim_clock.get_sim_time()
#                 current_time = _current_time.nanosec / 1e9 + _current_time.sec
#                 if current_time - start_time >= self.duration:
#                     break
#                 try:
#                     data = path_scanner_main(CenterOffsetScanner, timeout=0.5)  # 动态调用
#                     distance= data.get('center_distance', 0)
#                     if self.distance <= distance or distance <= 10:
#                         self.locomotion.set_motion(5)
#                         time.sleep(0.01)
#                     elif distance == 0 : 
#                         print(f"Finished executing {self.name} in {current_time - start_time:.2f} seconds")
#                         break
#                 except Exception as e:
#                     print(f"检测异常: {str(e)}")
#                     break
        
#         finally:
#             sim_clock.destroy_node()
#             rclpy.shutdown()

# class Walking_Forward_by_distance(Basic_State):
#     '''
#     这个函数写的不是很好，我本意是静态的获得目前的距离，然后根据距离/速度，
#     换算出接下来要走的时间。但是发现算的差错太大，目前是不太好用。
#     '''
#     def __init__(self, duration, check_method):
#         super().__init__()
#         self.name = "Walking Forward by distance"
#         self.duration = duration
#         self.check_method = check_method  # 直接存储检测方法引用
#         self.flag = 0

#     def execute(self):
#         print(f"Executing {self.name} with distance threshold: {self.duration}")
        
#         # 动态获取最新检测方法
#         if not callable(self.check_method):
#             raise ValueError("无效的检测方法")
            
#         start_time = time.time()
#         data = self.check_method()  # 动态调用
#         distance = data['far_distance']
#         threshold = 12.5
#         if distance >= 30 :
#             threshold = 14
#         elif distance >= 25 :
#             threshold = 13
#         elif distance >= 20 :
#             threshold = 12.5
#         while time.time() - start_time < threshold :
#             try:
#                 self.locomotion.set_motion(5)
#             except Exception as e:
#                 print(f"检测异常: {str(e)}")
#                 break
#         print(f"Finished executing {self.name} in {time.time() - start_time:.2f} seconds")

# class Walking_Forward_by_bottom(Basic_State):
#     def __init__(self, duration , distance):
#         """
#         向前走，目前是曲线入口专用，根据视野内底边赛道与左边界的距离判断
#         """
#         super().__init__()
#         self.name = "Walking forward by bottom"
#         self.motion_id = 5
#         self.duration = duration
#         self.distance = distance

#     def execute(self):
#         print(f"Executing {self.name} with distance threshold: {self.distance}")
        
#         rclpy.init()
#         sim_clock = SimulationClock()

#         # 启动一个线程运行 ROS node（非阻塞）
#         thread = threading.Thread(target=rclpy.spin, args=(sim_clock,), daemon=True)
#         thread.start()

#         time.sleep(1)  # 等待订阅者连接
#         try:
#             _start_time = sim_clock.get_sim_time()
#             start_time = _start_time.nanosec / 1e9 + _start_time.sec
#             current_time = start_time
#             while True:
#                 time.sleep(0.01)
#                 _current_time = sim_clock.get_sim_time()
#                 current_time = _current_time.nanosec / 1e9 + _current_time.sec
#                 if current_time - start_time >= self.duration:
#                     break
#                 try:
#                     data = path_scanner_main(BottomLeftScanner, timeout=0.5)  # 动态调用
#                     distance = data.get('left_bottom_distance', 0)
#                     if distance <= self.distance:
#                         self.locomotion.set_motion(16)
#                         time.sleep(0.01)
#                     else : 
#                         print(f"Finished executing {self.name} in {current_time - start_time:.2f} seconds")
#                         break
#                 except Exception as e:
#                     print(f"检测异常: {str(e)}")
#                     break
#         finally:
#             sim_clock.destroy_node()
#             rclpy.shutdown()

# class Spin_by_black_bar(Basic_State):
#     '''
#     原地旋转，根据眼前限高杆（小于10度）线条判断，然后决定旋转方向，顺、逆旋转至横线小于1.5度
#     '''
#     def __init__(self, duration):
#         super().__init__()
#         self.name = "Spin by black bar"
#         self.duration = duration
#         self.flag = 0

#     def execute(self):
#         print(f"Executing {self.name} with distance threshold: {self.duration}")
        
#         rclpy.init()
#         sim_clock = SimulationClock()

#         # 启动一个线程运行 ROS node（非阻塞）
#         thread = threading.Thread(target=rclpy.spin, args=(sim_clock,), daemon=True)
#         thread.start()

#         time.sleep(1)  # 等待订阅者连接
#         try:    
#             _start_time = sim_clock.get_sim_time()
#             start_time = _start_time.nanosec / 1e9 + _start_time.sec
#             current_time = start_time
#             while True:
#                 time.sleep(0.01)
#                 _current_time = sim_clock.get_sim_time()
#                 current_time = _current_time.nanosec / 1e9 + _current_time.sec
#                 if current_time - start_time >= self.duration:
#                     break
#                 try:
#                     data = path_scanner_main(BlackBarScanner, timeout=0.5)  # 动态调用
#                     is_horizontal_line= data.get('is_horizontal_bar', 0)
#                     if is_horizontal_line == 2:
#                         self.locomotion.set_motion(11)
#                     elif is_horizontal_line == 1:
#                         self.locomotion.set_motion(12)
#                     elif is_horizontal_line == 3:
#                         print(f"Finished executing {self.name} in {current_time - start_time:.2f} seconds")
#                         break
#                     else : 
#                         print(f"Finished executing {self.name} in {current_time - start_time:.2f} seconds")
#                         break
#                 except Exception as e:
#                     print(f"检测异常: {str(e)}")
#                     break
#         finally:
#             sim_clock.destroy_node()
#             rclpy.shutdown()   