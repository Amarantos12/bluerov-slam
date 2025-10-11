import os
import pickle
import numpy as np
import cv2
import rosbag
from scipy.spatial.transform import Rotation
from tqdm import tqdm

# 路径配置
polar_dir = "/home/hzr/rosbag/aurora_polar"  # 极坐标图像目录
bag_file = "/home/hzr/rosbag/aurora.bag"    # ROS bag 文件
output_dir = "/home/hzr/catkin_ws/src/neusis/data/aurora/Data"  # 输出 PKL 文件目录

# 确保输出目录存在
os.makedirs(output_dir, exist_ok=True)

# 步骤1: 列出并按时间戳排序极坐标图像（secs_nsecs.png）
image_files = [f for f in os.listdir(polar_dir) if f.endswith('.png')]
image_files.sort(key=lambda f: tuple(map(int, f.split('.')[0].split('_'))))  # 按(secs, nsecs)排序

print(f"找到 {len(image_files)} 个极坐标图像。")

# 步骤2: 从bag文件中收集所有/rexrov/pose_gt消息及其时间戳
pose_msgs = []
with rosbag.Bag(bag_file, 'r') as bag:
    for topic, msg, t in bag.read_messages(topics=['/rexrov/pose_gt']):
        timestamp = msg.header.stamp.to_sec()  # secs + nsecs / 1e9
        pose_msgs.append((timestamp, msg))

print(f"收集到 {len(pose_msgs)} 个位姿消息。")

# 步骤3: 按顺序处理每个图像
for idx, filename in enumerate(tqdm(image_files, desc="生成PKL文件")):
    # 从文件名解析时间戳
    secs_str, nsecs_str = filename.split('.')[0].split('_')
    timestamp_str = f"{secs_str}_{nsecs_str}"  # 保持完整时间戳，如 459_580000000
    timestamp_float = int(secs_str) + int(nsecs_str) / 1e9  # 用于匹配位姿的浮点时间戳

    # 以灰度float32加载极坐标图像并归一化
    image_path = os.path.join(polar_dir, filename)
    image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if image is None:
        print(f"加载 {filename} 失败，跳过。")
        continue
    image = image.astype(np.float32) / 255.0  # 归一化到 [0, 1]

    # 找到时间戳最接近的位姿
    closest_pose_msg = min(pose_msgs, key=lambda x: abs(x[0] - timestamp_float))[1]

    # 提取位置和方向
    pos = closest_pose_msg.pose.pose.position
    ori = closest_pose_msg.pose.pose.orientation
    quat = [ori.x, ori.y, ori.z, ori.w]

    # 将四元数转换为旋转矩阵
    rot = Rotation.from_quat(quat).as_matrix().astype(np.float32)

    # 构建4x4 PoseSensor矩阵
    pose_sensor = np.eye(4, dtype=np.float32)
    pose_sensor[:3, :3] = rot
    pose_sensor[:3, 3] = [pos.x, pos.y, pos.z]

    # 构建PKL字典，仅保留必要键
    state = {
        't': timestamp_str,  # 时间戳（字符串，如 459_580000000）
        'ImagingSonar': image,  # 极坐标图像（np.float32数组，归一化到[0,1]）
        'PoseSensor': pose_sensor  # 4x4矩阵
    }

    # 保存为PKL
    pkl_filename = f"{idx + 1}.pkl"
    pkl_path = os.path.join(output_dir, pkl_filename)
    with open(pkl_path, 'wb') as f:
        pickle.dump(state, f)

    print(f"保存 {pkl_path}")

print("所有PKL文件生成完成。")