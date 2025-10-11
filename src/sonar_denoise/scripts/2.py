# 间隔一秒保存声纳图像

import rosbag
import cv2
import cv_bridge
import os
import numpy as np
from tqdm import tqdm
import time

# === 路径配置 ===
bag_file = '/home/hzr/rosbag/aurora.bag'
output_folder = '/home/hzr/rosbag/aurora_dikaer'

# 确保输出文件夹存在
os.makedirs(output_folder, exist_ok=True)

# 初始化 cv_bridge
bridge = cv_bridge.CvBridge()

# 参数
last_saved_sec = -1
max_time = 1190  # 只处理前 1190 秒
window_name = "Sonar Preview"
start_time = None

cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

# === 打开 bag 文件 ===
with rosbag.Bag(bag_file, 'r') as bag:
    total_msgs = bag.get_message_count('/rexrov/blueview_m450/sonar_image')
    print(f"总共检测到 {total_msgs} 条消息")

    with tqdm(total=total_msgs, desc="提取与展示图像", unit="帧") as pbar:
        for topic, msg, t in bag.read_messages(topics=['/rexrov/blueview_m450/sonar_image']):
            try:
                time_sec = msg.header.stamp.secs
                time_nsec = msg.header.stamp.nsecs

                if start_time is None:
                    start_time = time_sec  # 记录 bag 开始时间

                # 超过1190秒就停止
                if (time_sec - start_time) > max_time:
                    print("✅ 已到达1190秒，停止读取。")
                    break

                # 每秒只保存一次
                if time_sec != last_saved_sec:
                    image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                    if image is None:
                        print(f"⚠️ 无法解码消息 {msg.header.stamp}")
                        pbar.update(1)
                        continue

                    # 转灰度
                    image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

                    # 保存图像
                    filename = f"{output_folder}/{time_sec}_{time_nsec}.png"
                    cv2.imwrite(filename, image_gray)

                    # 展示当前图像
                    # cv2.imshow(window_name, image_gray)
                    # cv2.waitKey(1)  # 刷新窗口
                    # time.sleep(0.1)

                    print(f"已保存并展示: {filename}")
                    last_saved_sec = time_sec

                pbar.update(1)

            except Exception as e:
                print(f"❌ 处理消息 {msg.header.stamp} 时出错: {str(e)}")
                pbar.update(1)
                continue

cv2.destroyAllWindows()
print("🎉 处理完成！")
