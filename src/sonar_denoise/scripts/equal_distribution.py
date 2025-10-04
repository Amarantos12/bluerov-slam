#!/usr/bin/env python
import rosbag
from tqdm import tqdm
import rospy

# 输入和输出 bag 文件路径
input_bag_path = '/home/hzr/rosbag/ARACATI_2017_8bits_full_slam.bag'
output_bag_paths = [
    '/home/hzr/rosbag/ARACATI_2017_8bits_full_slam_part1.bag',
    '/home/hzr/rosbag/ARACATI_2017_8bits_full_slam_part2.bag',
    '/home/hzr/rosbag/ARACATI_2017_8bits_full_slam_part3.bag'
]

def split_bag_by_time():
    # 打开输入 bag 文件，获取时间范围
    with rosbag.Bag(input_bag_path) as in_bag:
        # 获取 bag 文件的开始和结束时间
        info = in_bag.get_type_and_topic_info()
        start_time = in_bag.get_start_time()  # 最早消息时间戳（秒）
        end_time = in_bag.get_end_time()      # 最晚消息时间戳（秒）
        total_duration = end_time - start_time
        print(f"Bag start time: {start_time}, end time: {end_time}, duration: {total_duration} seconds")

        # 计算三个时间段的边界
        segment_duration = total_duration / 3.0
        time_splits = [
            start_time,
            start_time + segment_duration,
            start_time + 2 * segment_duration,
            end_time
        ]
        print(f"Time segments: {time_splits}")

        # 获取总消息数，用于进度条
        total_messages = in_bag.get_message_count()

        # 打开三个输出 bag 文件
        out_bags = [rosbag.Bag(path, 'w') for path in output_bag_paths]
        
        # 遍历输入 bag 文件的消息
        with tqdm(total=total_messages, desc="Splitting ROS bag", unit="msg") as pbar:
            for topic, msg, t in in_bag.read_messages():
                # 获取消息时间戳（秒）
                msg_time = t.to_sec()
                
                # 根据时间戳分配到对应的输出 bag
                if start_time <= msg_time < time_splits[1]:
                    out_bags[0].write(topic, msg, t)
                elif time_splits[1] <= msg_time < time_splits[2]:
                    out_bags[1].write(topic, msg, t)
                elif time_splits[2] <= msg_time <= end_time:
                    out_bags[2].write(topic, msg, t)
                
                pbar.update(1)
        
        # 关闭输出 bag 文件
        for out_bag in out_bags:
            out_bag.close()

    print(f"Split complete. Output bags saved to:")
    for path in output_bag_paths:
        print(f"  {path}")

if __name__ == '__main__':
    try:
        split_bag_by_time()
    except Exception as e:
        print(f"Error: {e}")