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
- 深度流：默认启用
- 点云：默认启用
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

调整深度分辨率和帧率：

```bash
ros2 launch gemini335_bringup gemini335.launch.py \
  depth_width:=1280 depth_height:=800 depth_fps:=30
```

分辨率和帧率必须使用 Gemini 335 支持的组合。修改后需要先停止当前相机节点，再重新启动；如果使用安装工作空间中的 launch 文件，还需要重新构建并 source：

```bash
cd /home/xiaokai/gemini335-learning
colcon build --base-paths ros2 --build-base ros2/build --install-base ros2/install
source ros2/install/setup.bash
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

点云话题通常位于：

```text
/gemini335/depth/points
```

查看点云话题类型和发布者：

```bash
ros2 topic info /gemini335/depth/points
ros2 topic hz /gemini335/depth/points
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

Orbbec 的 deb 依赖、SDK 动态库和 udev 规则仍由 `/home/xiaokai/orbbec_ws` 及系统负责。项目目录只管理自己的启动文件，便于后续增加图像处理、深度处理和其他实验节点。
