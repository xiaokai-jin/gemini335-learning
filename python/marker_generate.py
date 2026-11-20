import os

import cv2

# OpenCV 4.5.4 使用旧版 ArUco API，标记生成函数是 drawMarker，
# 新版 OpenCV 中可能改名为 generateImageMarker。
dictionary = cv2.aruco.getPredefinedDictionary(
    cv2.aruco.DICT_4X4_50
)

# 将标记图保存到项目的实验图片目录。
output_dir = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "results", "images", "marker")
)
os.makedirs(output_dir, exist_ok=True)

# 生成 ID 0~5 的标记，便于后续同时识别多个罐子或位置。
for marker_id in range(0, 6):
    marker = cv2.aruco.drawMarker(
        dictionary,
        id=marker_id,
        sidePixels=800,
        borderBits=1,
    )
    output_path = os.path.join(
        output_dir,
        "aruco_4x4_id%d.png" % marker_id,
    )

    if not cv2.imwrite(output_path, marker):
        raise RuntimeError("无法保存 ArUco 标记图片：%s" % output_path)

    print("ArUco 标记已保存到：%s" % output_path)