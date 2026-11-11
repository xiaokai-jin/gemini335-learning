# Orbbec Gemini 335 C++ 示例

## 1. 示例功能

本示例使用 Orbbec C++ SDK 连接第一台 Orbbec 相机，并使用 OpenCV 显示 RGB 图像。

程序启动后会：

1. 输出 Orbbec SDK 版本。
2. 查询已连接的设备。
3. 获取第一台设备。
4. 输出设备型号、序列号、固件版本、PID 和 VID。
5. 使用默认配置启动数据流。
6. 等待帧数据并提取彩色帧。
7. 将 MJPEG、RGB、BGR、RGBA 或 BGRA 转换为 OpenCV 的 BGR 图像。
8. 显示实时 RGB 画面。

按 `q` 或 `Esc` 可以退出程序。

## 2. 文件说明

```text
c++/
├── CMakeLists.txt
├── main.cpp
└── build/                 # CMake 构建目录，生成后出现
```

### `main.cpp`

核心代码使用以下 SDK 对象：

- `ob::Context`：创建 SDK 上下文并查询设备。
- `ob::Device`：访问设备信息。
- `ob::Pipeline`：启动数据流和等待帧数据。
- `FrameSet::colorFrame()`：从一组帧中获取彩色帧。

`toBgr()` 函数根据 Orbbec 返回的像素格式转换图像。由于 OpenCV 默认显示 BGR 格式，RGB、RGBA 等格式需要转换后再调用 `cv::imshow()`。

### `CMakeLists.txt`

CMake 配置完成以下工作：

- 使用 C++17。
- 查找 OpenCV 的 `core`、`highgui`、`imgproc` 和 `imgcodecs` 模块。
- 引入 Orbbec SDK 头文件。
- 链接 Orbbec SDK 动态库和 OpenCV。
- 设置运行时动态库搜索路径。

当前 SDK 路径配置为：

```text
/home/xiaokai/orbbec_ws/src/OrbbecSDK_ROS2/orbbec_camera/SDK
```

## 3. 编译和运行

```bash
cd /home/xiaokai/gemini335-learning/c++

cmake -S . -B build
cmake --build build -j2
./build/orbbec_rgb_demo
```

运行前需要：

- 连接 Orbbec 相机。
- 已安装 Orbbec SDK。
- 已安装 OpenCV C++ 开发文件。
- 用户具有访问相机 USB 设备的权限。

## 4. 运行结果

程序会输出类似信息：

```text
Orbbec SDK version: 2.7.6
Device: Orbbec Gemini 335
Serial number: CP0F463000XE
Firmware version: 1.8.10
RGB 预览已启动，按 q 或 ESC 退出。
```

## 5. 错误处理

- 没有检测到相机时返回错误。
- SDK 异常会输出 `Orbbec SDK error`。
- OpenCV 或其他标准异常会输出 `Error`。
- 不支持的彩色帧格式会被明确报告。

这个示例直接使用 SDK，不依赖 ROS2，也不产生 ROS2 话题。
