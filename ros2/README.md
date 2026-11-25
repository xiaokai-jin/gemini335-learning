# Orbbec Gemini 335 ROS2 使用说明

## 1. 目录作用

本目录包含项目自己的 ROS2 启动配置：

```text
ros2/
├── gemini335_bringup/
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── launch/
│       └── gemini335.launch.py
├── build/                 # colcon 构建目录
├── install/               # colcon 安装目录
└── log/                   # colcon 日志目录
```

`gemini335_bringup` 是一个轻量的 bringup 包。它不重复安装 Orbbec SDK，也不包含相机驱动源码；实际驱动来自已经构建好的：

```text
/home/xiaokai/orbbec_ws/install
```

因此，运行项目中的 launch 文件前，需要先 source ROS2 和 Orbbec 工作空间。

## 2. Launch 文件做什么

[gemini335.launch.py](./gemini335_bringup/launch/gemini335.launch.py) 启动已安装的：

```text
orbbec_camera/orbbec_camera_node
```

并为它设置以下参数：

- 相机命名空间：默认 `gemini335`
- 彩色流：启用，`1280 x 720 @ 30 FPS`
- 深度流：默认启用，`1280 x 800 @ 30 FPS`
- 左目 IR 流：默认启用，`1280 x 800 @ 30 FPS`
- 右目 IR 流：默认启用，`1280 x 800 @ 30 FPS`
- 点云：默认启用
- 彩色点云：默认关闭
- 点云降采样：`1`（不降采样）
- 点云 QoS：`default`
- 有序点云：默认关闭
- 深度对齐：默认使用软件对齐（`align_mode:=SW`）
- 加速度计：关闭
- 陀螺仪：关闭
- 帧同步：开启
- 深度注册：关闭

驱动启动后，RGB 和深度数据会发布为 ROS2 话题。

## 3. `serial_number` 参数

`serial_number` 用于指定要连接的相机。默认值为空字符串，表示使用驱动找到的设备。

当前 Gemini 335 的序列号为：

```text
CP0F463000XE
```

连接多台相机时，建议明确指定：

```bash
serial_number:=CP0F463000XE
```

这样可以避免启动错误的相机。

## 4. 构建项目包

首次构建或修改 launch 包后执行：

```bash
source /opt/ros/humble/setup.bash
source /home/xiaokai/orbbec_ws/install/setup.bash

cd /home/xiaokai/gemini335-learning
colcon build \
  --base-paths ros2 \
  --build-base ros2/build \
  --install-base ros2/install
```

构建完成后加载项目工作空间：

```bash
source /home/xiaokai/gemini335-learning/ros2/install/setup.bash
```

## 5. 启动相机

每次打开新终端：

```bash
source /opt/ros/humble/setup.bash
source /home/xiaokai/orbbec_ws/install/setup.bash
source /home/xiaokai/gemini335-learning/ros2/install/setup.bash
```

启动：

```bash
ros2 launch gemini335_bringup gemini335.launch.py
```

指定序列号：

```bash
ros2 launch gemini335_bringup gemini335.launch.py \
  serial_number:=CP0F463000XE
```

只启动 RGB：

```bash
ros2 launch gemini335_bringup gemini335.launch.py \
  enable_depth:=false
```

只启动深度：

```bash
ros2 launch gemini335_bringup gemini335.launch.py \
  enable_color:=false
```

调整 RGB 分辨率和帧率：

```bash
ros2 launch gemini335_bringup gemini335.launch.py \
  color_width:=1280 color_height:=720 color_fps:=30
```

调整深度分辨率和帧率（左右目开启时，必须与左右目使用相同的配置）：

```bash
ros2 launch gemini335_bringup gemini335.launch.py \
  depth_width:=1280 depth_height:=800 depth_fps:=30 \
  left_ir_width:=1280 left_ir_height:=800 left_ir_fps:=30 \
  right_ir_width:=1280 right_ir_height:=800 right_ir_fps:=30
```

分辨率和帧率必须使用 Gemini 335 支持的组合。修改后需要先停止当前相机节点，再重新启动；如果使用安装工作空间中的 launch 文件，还需要重新构建并 source：

```bash
cd /home/xiaokai/gemini335-learning
colcon build --base-paths ros2 --build-base ros2/build --install-base ros2/install
source ros2/install/setup.bash
```

如果只需要原来的 `1280 x 800` 深度配置，必须同时关闭左右目：

```bash
ros2 launch gemini335_bringup gemini335.launch.py \
  enable_left_ir:=false enable_right_ir:=false \
  depth_width:=1280 depth_height:=800 depth_fps:=30
```

## 6. 查看 ROS2 话题

查看节点和话题：

```bash
ros2 node list
ros2 topic list
```

按默认配置，RGB 图像通常位于：

```text
/gemini335/color/image_raw
```

左右目 IR 图像位于：

```text
/gemini335/left_ir/image_raw
/gemini335/right_ir/image_raw
```

左右目图像的完整实验步骤和双目深度原理见
[阶段四：双目图像](../docs/04-双目图像.md)。

点云话题通常位于：

```text
/gemini335/depth/points
```

查看点云话题类型和发布者：

```bash
ros2 topic info /gemini335/depth/points
ros2 topic hz /gemini335/depth/points
```

点云相关 launch 参数：

```text
enable_point_cloud
enable_colored_point_cloud
point_cloud_decimation_filter_factor（1~8）
point_cloud_qos
cloud_frame_id
ordered_pc
depth_registration
align_mode（`SW` 或 `HW`）
```

例如，开启彩色点云并将点云降采样为 2：

```bash
ros2 launch gemini335_bringup gemini335.launch.py \
  enable_colored_point_cloud:=true \
  depth_registration:=true \
  align_mode:=SW \
  point_cloud_decimation_filter_factor:=2
```

Gemini 335 当前彩色点云建议使用软件对齐。`HW` 硬件对齐只适用于设备支持的彩色/深度 profile 组合；如果日志出现
`Current stream profile is not support hardware d2c process`，应改为
`align_mode:=SW`。

彩色点云话题为：

```text
/gemini335/depth_registered/points
```

查看 RGB 图像：

```bash
ros2 run rqt_image_view rqt_image_view
```

在窗口中选择 `/gemini335/color/image_raw`。

也可以查看图像信息：

```bash
ros2 topic info /gemini335/color/image_raw
ros2 topic hz /gemini335/color/image_raw
```

## 7. 依赖关系

项目 bringup 包依赖：

- ROS2 Humble 的 `launch`
- ROS2 Humble 的 `launch_ros`
- 已安装的 `orbbec_camera`
- `rclpy`、`sensor_msgs` 和 `std_msgs`
- `cv_bridge`

Orbbec 的 deb 依赖、SDK 动态库和 udev 规则仍由 `/home/xiaokai/orbbec_ws` 及系统负责。项目目录只管理自己的启动文件，便于后续增加图像处理、深度处理和其他实验节点。

## 8. 绿色罐子颜色分割

在源码工作区重新构建并加载环境：

```bash
source /opt/ros/humble/setup.bash
source /home/xiaokai/orbbec_ws/install/setup.bash
cd /home/xiaokai/gemini335-learning
colcon build \
  --base-paths ros2 \
  --build-base ros2/build \
  --install-base ros2/install \
  --packages-select gemini335_bringup
source ros2/install/setup.bash
```

启动相机和绿色罐子检测：

```bash
ros2 launch gemini335_bringup gemini335.launch.py \
  enable_green_jar_detection:=true \
  green_jar_minimum_area:=500
```

已增加暗光增强：检测前会用 Gamma 和 CLAHE 提亮 HSV 的亮度分量。默认暗光参数为：

```text
H: 35~85
S: 40~255
V: 20~255
Gamma x100: 70
CLAHE clip x10: 20
```

界面中可拖动滑条实时调整：

- `H min/max`：调整目标的色相范围。绿色通常在 35~85；相机偏绿或偏黄时移动整个范围。
- `S min/max`：调整饱和度。暗处绿色容易变灰，降低 `S min` 可帮助识别，但可能增加背景误检。
- `V min/max`：调整亮度。暗处罐子仍亮度较低时降低 `V min`；环境反光过强时提高 `V min` 或降低 `V max`。
- `Gamma x100`：数值除以 100 得到 Gamma 倍数。默认 70 表示约 0.7，会增强暗部。若画面过亮或噪点增多，调到 100 或更高。
- `CLAHE clip x10`：数值除以 10 得到局部对比度。默认 2.0；设为 0 可关闭 CLAHE。

检测窗口只显示原图和绿色罐子的外接框，不再显示 HSV Mask 窗口。在检测窗口按：

- `s` 保存成功案例图；
- `f` 保存失败案例图；
- `q` 退出。

案例保存在 `results/images/`。节点发布：

```text
/gemini335/detection/image
/gemini335/detection/green_jar
```

其中检测话题为包含外接框、中心像素、面积和时间戳的 JSON 字符串。也可直接运行脚本而不启动相机：

```bash
python3 python/object_detection_color.py
```

## 9. ArUco 标记检测

ArUco 节点会检测标记编号和角点，并利用 Gemini 335 发布的彩色相机内参估计标记相对于相机光学坐标系的位姿。生成标记时使用：

```text
字典：DICT_4X4_50
标记编号：0~49
当前打印标记外框：48.5 mm
```

启动相机和 ArUco 检测：

```bash
source /opt/ros/humble/setup.bash
source /home/xiaokai/orbbec_ws/install/setup.bash
source ros2/install/setup.bash

ros2 launch gemini335_bringup gemini335.launch.py \
  enable_aruco_detection:=true \
  aruco_dictionary:=DICT_4X4_50 \
  aruco_marker_length_m:=0.0485
```

`aruco_marker_length_m` 必须填写标记黑色外框的实际边长，单位为米；此处使用实测的 `48.5 mm = 0.0485 m`。`aruco_camera_info_topic` 默认为 `/gemini335/color/camera_info`，必须对应正在检测的彩色图像及其分辨率。

节点会在彩色图像上绘制标记边框、编号、中心点和 XYZ 坐标轴，并发布：

```text
/gemini335/detection/aruco/image
/gemini335/detection/aruco
```

同时会通过 `/tf` 发布相机到标记的实时变换，子坐标系默认为：

```text
aruco_marker_0
aruco_marker_1
...
```

变换的父坐标系来自彩色图像的 `header.frame_id`，子坐标系原点位于
ArUco 标记中心，平移和完整三维旋转均可在 TF 中查看。

检测话题中的每个标记包含：

```text
id：标记编号
corners：四个角点像素坐标
center：四角的中心像素坐标
pose.translation_m：标记原点在相机光学坐标系中的 [X, Y, Z]，单位米
pose.rotation_vector_rad：标记相对于相机的 Rodrigues 旋转向量，单位弧度
pose.rotation_matrix：对应的 3×3 旋转矩阵
pose.distance_m：相机光学中心到标记原点的直线距离，单位米
pose.quaternion_xyzw：标记相对相机的姿态四元数，顺序为 x/y/z/w
camera_info_valid：当前图像是否具有有效且匹配的相机内参
pose_valid：当前帧是否成功估计了至少一个标记的位姿
```

相机坐标遵循 ROS optical frame：X 向图像右方、Y 向图像下方、Z 向相机前方。角点顺序为左上、右上、右下、左下。
`camera_info_valid` 表示相机内参是否可用于当前图像；`pose_valid` 表示当前帧是否成功估计了至少一个标记的位姿。CameraInfo 尚未到达、内参为零、畸变模型不支持或分辨率不匹配时，`camera_info_valid` 和 `pose_valid` 会是 `false`，检测结果仍包含编号和角点，但各标记的 `pose` 为 `null`，不会用不匹配的内参估算位姿。

检测窗口按 `s` 保存成功案例，
按 `f` 保存失败案例，按 `q` 退出。

查看检测结果：

```bash
ros2 topic echo /gemini335/detection/aruco
```

### 在 RViz2 查看相机与标记坐标系

同时启动 ArUco 检测和 RViz2：

```bash
ros2 launch gemini335_bringup gemini335.launch.py \
  enable_aruco_detection:=true \
  enable_aruco_rviz:=true \
  aruco_dictionary:=DICT_4X4_50 \
  aruco_marker_length_m:=0.0485 \
  aruco_pose_smoothing_alpha:=0.7
```

ArUco 角点会受像素量化、轻微模糊和反光影响，因此单帧 PnP 位姿可能跳动。
节点对每个 ID 的平移使用指数移动平均，对旋转使用四元数球面插值（SLERP），
TF 和 JSON 均发布平滑后的位姿。默认系数已调高至 `0.7`，以减少目标移动后的跟随延迟。
`aruco_pose_smoothing_alpha` 范围为 `(0, 1]`：

- `0.3`：更平滑，但目标移动时滞后较明显；
- `0.7`：默认折中，跟随更及时；
- `0.9`：响应很快，但残留抖动较多；
- `1.0`：关闭平滑，使用每帧原始位姿。

若标记连续丢失超过 `aruco_pose_filter_reset_sec`（默认 `0.5` 秒），下次检测会
重新初始化滤波，避免旧位姿拖拽。滤波不能替代相机固定、清晰对焦、避免反光、
标记贴平和准确填写标记边长；如需进一步减少抖动，应先确保标记在图像中足够大，
并检查相机是否稳固及 `CameraInfo` 是否与输入图像匹配。

RViz 配置会显示 TF 坐标轴、箭头和 frame 名称。默认固定坐标系为
`gemini335_color_optical_frame`；若相机发布的图像 `header.frame_id`
不同，在 RViz 左侧 `Global Options → Fixed Frame` 中改成实际 frame。

可在另一个终端查看相机到目标的变换：

```bash
ros2 run tf2_ros tf2_echo gemini335_color_optical_frame aruco_marker_0
```

只有检测到有效标记且收到有效相机内参时，标记 TF 才会出现。目标移动或旋转时，
标记坐标系也会随图像帧持续更新。若同一个 ID 的标记在画面中出现多次，它们会
共享同一个 TF 子 frame；因此一个 ID 应只对应一个同时可见的实体标记。

## 10. ArUco 节点退出原因说明

一次启动日志中的 `exit code 1` 只表示检测进程在启动约 9 秒后退出，
但该日志没有保留对应进程的 Python Traceback，因此不能据此确定唯一原因。

已做的复现检查：

- 直接启动节点：节点正常运行，并持续发布检测状态。
- 使用完整 launch 启动：节点正常启动，重新运行时没有再次退出。
- 离线检测：字典、角点输出和检测逻辑可以正常工作。
- 源码与 ROS2 安装目录中的节点已核对一致。

因此，当前证据不足以确认原进程是由代码逻辑直接导致退出；
更可能的情况包括启动时使用了旧版已安装节点、环境变量或相机
初始化状态异常、或 OpenCV 窗口创建异常。相机初始化日志中的
`uvc_open failed` 同时说明另一侧相机节点曾无法打开设备，
但已启动的 ArUco 节点本身没有从日志中看到检测或转换异常。

为提高可复现性，节点现在会：

- 跳过单帧图像转换异常，不让整个节点退出；
- 跳过单帧 ArUco 检测异常，不让整个节点退出；
- 将角点数组整理为固定的四行二列形式；
- 提供 `aruco_show_windows` 参数，可关闭 OpenCV 窗口；
- 保持检测异常时仍可继续等待下一帧。
