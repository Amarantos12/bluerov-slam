# 记录/pose_gt，类型为geometry_msgs/PoseStamped

import rosbag
from geometry_msgs.msg import PoseStamped
from tqdm import tqdm

# 定义rosbag路径、话题和输出txt路径
bag_path = '/home/hzr/rosbag/ARACATI_2017_8bits_full.bag'
topic_name = '/pose_gt'
output_txt = '/home/hzr/rosbag/ARACATI_2017_8bits_full.txt'

# 打开rosbag文件
bag = rosbag.Bag(bag_path)

# 获取该话题的消息总数
total_msgs = bag.get_message_count(topic_name)
print(f"总消息数: {total_msgs}")

# 打开输出文件
with open(output_txt, 'w') as f:
    # 使用tqdm显示进度条
    for topic, msg, t in tqdm(bag.read_messages(topics=[topic_name]), total=total_msgs, desc="处理进度"):
        # 提取时间戳（转换为浮点秒）
        timestamp = t.to_sec()
        
        # 提取位置和四元数
        position = msg.pose.position
        orientation = msg.pose.orientation
        
        # 写入TUM格式: timestamp tx ty tz qx qy qz qw
        f.write(f"{timestamp:.9f} {position.x} {position.y} {position.z} {orientation.x} {orientation.y} {orientation.z} {orientation.w}\n")

# 关闭bag文件
bag.close()
print(f"轨迹数据已保存到: {output_txt}")