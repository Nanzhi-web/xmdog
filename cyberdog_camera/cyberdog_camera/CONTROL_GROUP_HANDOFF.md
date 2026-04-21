# 2026 视觉节点交接说明

这份文档给控制组使用，目标是说明当前视觉节点如何接入、如何启动、以及控制组需要同步哪些代码。

## 1. 当前视觉能力

当前版本包含：

- 默认黄线检测
- 第二阶段球类检测
- 第三阶段限高/障碍检测
- 第六阶段基于 D435 深度图的足球检测

当前版本**不包含**：

- YOLO
- 特殊目标识别

第三阶段的 `target_logic.py` 现在是占位实现，固定返回：

```python
{"found": False, "type": "none", "distance": 99.9}
```

控制组不要依赖：

- `target_found == True`
- `target_type != "none"`

## 2. 控制组需要同步的代码

至少同步以下目录：

- `/home/cyberdog_sim/src/cyberdog_camera/cyberdog_camera`
- `/home/cyberdog_sim/src/cyberdog_camera/package.xml`
- `/home/cyberdog_sim/src/cyberdog_camera/setup.py`
- `/home/cyberdog_sim/src/xmdog_msgs`

如果控制组自己维护仿真模型，还需要同步：

- `/home/cyberdog_sim/src/cyberdog_simulator/cyberdog_robot/cyberdog_description/xacro/robot.xacro`

原因：

- D435 相机角度已经永久修改为向地面下俯约 60 度
- 具体改动在 `D435_camera_joint`
- 原始角度：`rpy="0 0 0"`
- 现用角度：`rpy="0 1.0472 0"`
- 也就是沿 pitch 方向额外下俯约 `60°`

## 3. 视觉节点启动

编译：

```bash
cd /home/cyberdog_sim
source /opt/ros/galactic/setup.bash
colcon build --packages-select xmdog_msgs cyberdog_camera --merge-install
source install/setup.bash
```

启动视觉节点：

```bash
ros2 run cyberdog_camera vision_service
```

## 4. 对外接口

### 服务

- `/vision/set_mode`
- `/get_soccer_pixel_pos`
- `/get_height_limit_distance`

### 话题

- `/vision/perception_state`

## 5. 模式说明

### `default`

用途：

- 黄线对齐

主要字段：

- `line_found`
- `line_center_offset`

### `stage_2`

用途：

- 第二阶段寻球

输入相机：

- 头部 RGB 相机 `/rgb_sensor/image_raw`

主要字段：

- `line_is_horizontal`
- `ball_found`
- `ball_color`
- `ball_x_offset`
- `ball_size_ratio`

### `stage_3`

用途：

- 黄线 + 限高 + 障碍

输入相机：

- 头部 RGB 相机 `/rgb_sensor/image_raw`

主要字段：

- `line_found`
- `line_center_offset`
- `height_limit_danger`
- `obstacle_danger`
- `obstacle_distance`
- `target_found` 固定为 `False`
- `target_type` 固定为 `"none"`
- `target_distance` 固定为 `99.9`

### `stage_6`

用途：

- 足球追踪

输入相机：

- D435 深度图 `/camera/depth/image_raw`

主要字段：

- `ball_found`
- `ball_color`
- `ball_x_offset`

说明：

- `ball_color` 检测到时固定填 `"soccer"`

## 6. 仿真启动顺序

必须按这个顺序启动：

1. `race_gazebo.launch.py`
2. 等待 Gazebo 完成模型加载、共享内存初始化
3. `cyberdog_control_launch.py`
4. 再启动 `vision_service`

推荐命令：

```bash
cd /home/cyberdog_sim
source /opt/ros/galactic/setup.bash
source install/setup.bash
ros2 launch cyberdog_gazebo race_gazebo.launch.py
```

等待稳定后：

```bash
ros2 launch cyberdog_gazebo cyberdog_control_launch.py
```

再开一个终端：

```bash
ros2 run cyberdog_camera vision_service
```

## 7. 判断仿真是否真正就绪

至少确认这些话题真正有消息，而不是只出现在 `topic list`：

- `/rgb_sensor/image_raw`
- `/rgb_sensor/camera_info`
- `/camera/depth/image_raw`
- `/camera/depth/camera_info`

如果控制链路没有完全拉活，Gazebo 可能停在等待状态，这时会出现：

- 话题名存在
- 但实际没有图像帧

这不是视觉代码错误，而是仿真底座还没完全起来。

## 8. 控制组怎么用

控制组进入状态时，调用：

```bash
/vision/set_mode
```

切换为：

- `default`
- `stage_2`
- `stage_3`
- `stage_6`
- `idle`

然后持续订阅：

```bash
/vision/perception_state
```

根据当前模式读取对应字段即可。
