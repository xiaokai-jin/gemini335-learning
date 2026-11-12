# 阶段二：RGB 图像

## 目标

获取 Gemini 335 的彩色图像，查看图像内容，并了解分辨率、帧率和图像编码。

## RGB 话题

默认的彩色图像话题是：

```text
/gemini335/color/image_raw
```

查看话题类型和发布频率：

```bash
ros2 topic info /gemini335/color/image_raw
ros2 topic hz /gemini335/color/image_raw
```

## 查看实时图像

安装图像查看工具：

```bash
sudo apt install ros-humble-rqt-image-view
```

启动：

```bash
ros2 run rqt_image_view rqt_image_view
```

在窗口中选择：

```text
/gemini335/color/image_raw
```

## 调整分辨率和帧率

启动相机时传入参数：

```bash
ros2 launch gemini335_bringup gemini335.launch.py \
  color_width:=1280 \
  color_height:=720 \
  color_fps:=30
```

分辨率和帧率需要使用相机支持的组合。修改后要先停止旧的相机节点，再重新启动。

## 保存一张 RGB 图片

可以使用 ROS 2 的图像保存工具保存原始图像。若系统尚未安装：

```bash
sudo apt install ros-humble-image-view
```

保存节点启动后，根据终端提示调用保存服务：

```bash
ros2 run image_view image_saver --ros-args \
  -r image:=/gemini335/color/image_raw
```

## 小结

这一阶段重点是确认：

- 相机能够发布 RGB 图像；
- 能够在窗口中看到实时画面；
- 能够检查图像话题的帧率；
- 能够根据需要调整分辨率和帧率。
