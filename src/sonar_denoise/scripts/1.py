# 读取前1190秒图像，每秒读一张，并展示

import rosbag
import cv2
import cv_bridge
import time

# === 参数设置 ===
bag_file = '/home/hzr/rosbag/aurora.bag'
topic_name = '/rexrov/blueview_m450/sonar_image'
max_duration = 1190  # 保留前1190秒
display_interval = 0.1  # 每张图间隔0.1秒

# === 初始化 CvBridge ===
bridge = cv_bridge.CvBridge()

# === 打开 rosbag 文件 ===
with rosbag.Bag(bag_file, 'r') as bag:
    first_time = None
    last_display_time = -1

    print("开始播放前1190秒图像... (按 Ctrl+C 退出)")

    for topic, msg, t in bag.read_messages(topics=[topic_name]):
        # 记录起始时间
        if first_time is None:
            first_time = t

        elapsed = (t - first_time).to_sec()

        # 超过1190秒则停止
        if elapsed > max_duration:
            print("播放结束（超过1190秒）")
            break

        # 只取每秒一帧（即时间戳秒数变化时显示）
        if int(elapsed) == last_display_time:
            continue
        last_display_time = int(elapsed)

        try:
            # 转换 ROS 图像为 OpenCV 图像
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # 转灰度图
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            # 显示图像
            cv2.imshow("Sonar Image", gray)
            cv2.waitKey(int(display_interval * 1000))

        except Exception as e:
            print(f"解码错误: {e}")
            continue

# 播放结束关闭窗口
cv2.destroyAllWindows()
