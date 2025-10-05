#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
generate_slam_bag_final_optimized.py

这个脚本用于处理 ARACATI 数据集的 ROS bag 文件，将其转换为 Bruce SLAM 系统所需的格式。
它基于一个经过验证可以正常工作的版本进行优化，增加了注释和选择性赋值。

核心转换逻辑:
1. 声呐图像处理: /son/compressed -> /sonar_oculus_node/ping
2. 运动数据处理:
   - 适配 DeadReckoningNode 的内部实现，通过逆向工程构造输入数据。
   - DVL速度: (x,y,z)_twist -> (x, -y, -z)_dvl
   - 深度: 从 /pose_gt 提取
   - IMU姿态: 构造一个特定的 (roll, pitch, yaw) 组合，以抵消 DeadReckoningNode 的硬编码旋转。
   - IMU角速度: 进行坐标轴交换和变号，以匹配 DeadReckoningNode 内部姿态变换后的坐标系。
"""

import rosbag
import cv_bridge
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage, Image, Imu
from sonar_oculus.msg import OculusPingUncompressed, OculusFire
from rti_dvl.msg import DVL
from bar30_depth.msg import Depth
from geometry_msgs.msg import TwistStamped, PoseStamped, Vector3, Quaternion
from std_msgs.msg import Header
import tf.transformations as tf_trans
from tqdm import tqdm
import rospy

# --- 全局配置 ---
INPUT_BAG_PATH = '/home/hzr/rosbag/ARACATI_2017_8bits_full.bag'
OUTPUT_BAG_PATH = '/home/hzr/rosbag/ARACATI_2017_8bits_full_slam.bag'
TIME_SYNC_THRESHOLD = 0.1  # /cmd_vel 和 /pose_gt 之间允许的最大时间差 (秒)

# --- 声呐图像转换函数 (保留原逻辑) ---
def cartesian_to_polar(image, max_range, bearing_range, num_beams, bearing_resolution, origin, reverse_z=1):
    height, width = image.shape[:2]
    polar_width = num_beams
    polar_height = height
    range_resolution = max_range / polar_height
    x, y = np.arange(polar_width), np.arange(polar_height)
    X, Y = np.meshgrid(x, y)
    r = Y * range_resolution
    theta = bearing_range[0] + X * bearing_resolution * reverse_z
    x1, y1 = r * np.sin(theta), r * np.cos(theta)
    map_x = np.clip(x1 / range_resolution + origin[0], 0, width - 1).astype(np.float32)
    map_y = np.clip(origin[1] - y1 / range_resolution, 0, height - 1).astype(np.float32)
    polar_image = cv2.remap(image, map_x, map_y, cv2.INTER_CUBIC, borderValue=0)
    if polar_image.dtype != np.uint8:
        polar_image = np.clip(polar_image, 0, 255).astype(np.uint8)
    return polar_image

def process_son_compressed(msg, ping_id_counter):
    try:
        bridge = cv_bridge.CvBridge()
        image = bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        if len(image.shape) == 3:
            image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        max_range = 50.0
        bearing_range_deg = [-65.0, 65.0]
        bearing_range = [np.deg2rad(d) for d in bearing_range_deg]
        num_beams = 768
        bearing_resolution = np.deg2rad(130.0 / num_beams)
        origin = (776.5, 848)
        
        polar_image = cartesian_to_polar(image, max_range, bearing_range, num_beams, bearing_resolution, origin)
        
        oculus_msg = OculusPingUncompressed(
            header=msg.header,
            fire_msg=OculusFire(header=msg.header, mode=1, gamma=128, flags=0, range=max_range, gain=0.0, speed_of_sound=1500.0, salinity=35.0),
            ping_id=ping_id_counter,
            part_number=1032,
            start_time=msg.header.stamp.secs,
            bearings=(np.linspace(bearing_range_deg[0], bearing_range_deg[1], num_beams) * 100).astype(np.int16).tolist(),
            range_resolution=max_range / polar_image.shape[0],
            num_ranges=polar_image.shape[0],
            num_beams=polar_image.shape[1],
            ping=bridge.cv2_to_imgmsg(polar_image, "mono8")
        )
        oculus_msg.ping.header = msg.header
        return oculus_msg, ping_id_counter + 1
    
    except Exception as e:
        rospy.logwarn("Error processing sonar image: %s", str(e))
        return None, ping_id_counter

def main():
    rospy.loginfo("Starting ROS bag processing...")
    rospy.loginfo("Input bag: %s", INPUT_BAG_PATH)
    rospy.loginfo("Output bag: %s", OUTPUT_BAG_PATH)

    # --- 步骤 1: 预处理，缓存 /pose_gt 消息 ---
    rospy.loginfo("Pass 1: Caching all /pose_gt messages...")
    pose_gt_dict = {}
    with rosbag.Bag(INPUT_BAG_PATH, 'r') as input_bag:
        total_pose_gt = input_bag.get_message_count(topic_filters=['/pose_gt'])
        with tqdm(total=total_pose_gt, desc="Caching pose_gt", unit="msg") as pbar:
            for _, msg, t in input_bag.read_messages(topics=['/pose_gt']):
                pose_gt_dict[t.to_nsec()] = msg
                pbar.update(1)
    sorted_pose_gt_times = sorted(pose_gt_dict.keys())

    # --- 步骤 2: 主处理循环 ---
    rospy.loginfo("Pass 2: Processing all topics and writing to new bag...")
    ping_id_counter = 0
    total_messages = rosbag.Bag(INPUT_BAG_PATH).get_message_count()
    
    with rosbag.Bag(OUTPUT_BAG_PATH, 'w') as outbag:
        with tqdm(total=total_messages, desc="Processing ROS bag", unit="msg") as pbar:
            for topic, msg, t in rosbag.Bag(INPUT_BAG_PATH).read_messages():
                if topic == '/son/compressed':
                    oculus_msg, ping_id_counter = process_son_compressed(msg, ping_id_counter)
                    if oculus_msg:
                        outbag.write('/sonar_oculus_node/ping', oculus_msg, t)
                
                elif topic == '/cmd_vel':
                    import bisect
                    idx = bisect.bisect_left(sorted_pose_gt_times, t.to_nsec())
                    candidates = []
                    if idx > 0: candidates.append(sorted_pose_gt_times[idx - 1])
                    if idx < len(sorted_pose_gt_times): candidates.append(sorted_pose_gt_times[idx])
                    
                    min_time_diff, best_candidate_time = float('inf'), None
                    for time_nsec in candidates:
                        diff = abs(t.to_nsec() - time_nsec)
                        if diff < min_time_diff:
                            min_time_diff, best_candidate_time = diff, time_nsec
                    
                    if best_candidate_time and min_time_diff < TIME_SYNC_THRESHOLD * 1e9:
                        closest_pose_gt = pose_gt_dict[best_candidate_time]
                        header = Header(stamp=msg.header.stamp, frame_id='base_link')
                        
                        # --- 1. DVL 消息 (增加了选择性赋值) ---
                        dvl_msg = DVL(header=header, temperature=-1.0, altitude=-1.0)
                        vel = msg.twist.linear
                        # 使用经过验证的坐标系映射: (x,y,z)_twist -> (x, -y, -z)_dvl
                        dvl_msg.velocity = Vector3(x=vel.x, y=-vel.y, z=-vel.z)
                        outbag.write('/rti/body_velocity/raw', dvl_msg, t)
                        
                        # --- 2. Depth 消息 ---
                        depth_msg = Depth(header=header, time=header.stamp.to_sec())
                        depth_msg.depth = -closest_pose_gt.pose.position.z
                        outbag.write('/bar30/depth/raw', depth_msg, t)
                        
                        # --- 3. IMU 消息 ---
                        imu_msg = Imu(header=header)
                        
                        # --- 姿态修正 ---
                        # 逆向工程 DeadReckoningNode 的行为：提供一组特定的 (roll, pitch, yaw)
                        # 原材料，使其内部重建后的姿态是我们想要的 (yaw, 0, 0)。
                        q_orig = closest_pose_gt.pose.orientation
                        euler_orig = tf_trans.euler_from_quaternion([q_orig.x, q_orig.y, q_orig.z, q_orig.w])
                        real_yaw = euler_orig[2]
                        
                        # 我们需要让 DeadReckoningNode 内部计算出的 roll 为 0。
                        # 计算公式: output_roll = np.radians(90) + input_roll
                        # 所以, 我们需要让 input_roll = -np.radians(90)
                        final_roll = -np.pi / 2.0
                        final_pitch = 0.0
                        final_yaw = real_yaw
                        
                        quat_final = tf_trans.quaternion_from_euler(final_roll, final_pitch, final_yaw)
                        imu_msg.orientation = Quaternion(*quat_final)

                        # --- 角速度修正 (增加了选择性赋值) ---
                        ang_vel = msg.twist.angular
                        imu_angular_velocity = Vector3()
                        # 使用经过验证的坐标系映射
                        if abs(ang_vel.x) > 1e-6: imu_angular_velocity.x = ang_vel.x   # Roll -> Roll
                        if abs(ang_vel.z) > 1e-6: imu_angular_velocity.y = -ang_vel.z  # Yaw -> -Pitch
                        if abs(ang_vel.y) > 1e-6: imu_angular_velocity.z = ang_vel.y   # Pitch -> Yaw
                        imu_msg.angular_velocity = imu_angular_velocity
                        
                        # 设置协方差
                        imu_msg.orientation_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
                        imu_msg.angular_velocity_covariance = [1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6]
                        imu_msg.linear_acceleration_covariance[0] = -1.0 # 标记线加速度无效
                        
                        outbag.write('/vn100/imu/raw', imu_msg, t)
                
                elif topic == '/pose_gt':
                    outbag.write(topic, msg, t)

                pbar.update(1)

    rospy.loginfo("Processing complete. Output bag saved to: %s", OUTPUT_BAG_PATH)

if __name__ == '__main__':
    main()