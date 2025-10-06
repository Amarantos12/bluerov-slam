import rosbag
import cv2
import cv_bridge
import os
import numpy as np
from tqdm import tqdm

# 定义路径
bag_file = '/home/hzr/rosbag/ARACATI_2017_8bits_full.bag'
output_folder = '/home/hzr/rosbag/Aracati_full'

# 确保输出文件夹存在
if not os.path.exists(output_folder):
    os.makedirs(output_folder)

# 初始化 cv_bridge
bridge = cv_bridge.CvBridge()

# 打开 bag 文件并统计消息数量以设置进度条
with rosbag.Bag(bag_file, 'r') as bag:
    total_msgs = bag.get_message_count('/son/compressed')
    print(f"总共检测到 {total_msgs} 条消息")

    # 使用 tqdm 创建进度条
    with tqdm(total=total_msgs, desc="处理图像", unit="帧") as pbar:
        # 遍历指定话题的消息
        for topic, msg, t in bag.read_messages(topics=['/son/compressed']):
            try:
                # 将压缩图像转换为 OpenCV 格式
                np_arr = np.frombuffer(msg.data, np.uint8)
                image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                
                # 确保图像有效
                if image is None:
                    print(f"警告: 无法解码消息 {msg.header.stamp}")
                    pbar.update(1)
                    continue
                
                # 转换为灰度图
                image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                
                # 获取时间戳
                time_sec = msg.header.stamp.secs
                time_nsec = msg.header.stamp.nsecs
                
                # 创建文件名，确保时间戳完整
                filename = f"{output_folder}/{time_sec}_{time_nsec}.png"
                
                # 保存图像
                cv2.imwrite(filename, image)
                # print(f"已保存: {filename}")
                
                # 更新进度条
                pbar.update(1)
                
            except Exception as e:
                print(f"处理消息 {msg.header.stamp} 时出错: {str(e)}")
                pbar.update(1)
                continue