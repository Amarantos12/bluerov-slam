#!/usr/bin/env python

import rospy
from bruce_slam.utils.io import *
from bruce_slam.slam_ros import SLAMNode
from bruce_slam.utils.topics import *

def offline(args)->None:
    """运行离线 SLAM 系统，处理 ROS bag 文件中的录制数据

    Args:
        args (Any): 运行系统所需的参数（如文件路径、起始时间、持续时间等）
    """

    # 导入离线模式所需的额外模块
    from rosgraph_msgs.msg import Clock
    from dead_reckoning_node import DeadReckoningNode
    from feature_extraction_node import FeatureExtraction
    from gyro_node import GyroFilter
    from mapping_node import MappingNode
    from bruce_slam.utils import io

    # 设置离线模式参数
    io.offline = True  # 标记为离线模式
    node.save_fig = False  # 不保存图像
    node.save_data = False  # 不保存数据

    # 初始化所需的节点
    dead_reckoning_node = DeadReckoningNode()  # 死 reckoning 节点，用于位姿估计
    dead_reckoning_node.init_node(SLAM_NS + "localization/")  # 初始化死 reckoning 节点，命名空间为 localization
    feature_extraction_node = FeatureExtraction()  # 特征提取节点
    feature_extraction_node.init_node(SLAM_NS + "feature_extraction/")  # 初始化特征提取节点
    gyro_node = GyroFilter()  # 陀螺仪融合节点
    gyro_node.init_node(SLAM_NS + "gyro/")  # 初始化陀螺仪节点
    """mp_node = MappingNode()
    mp_node.init_node(SLAM_NS + "mapping/")"""  # 地图构建节点（已注释，未使用）
    clock_pub = rospy.Publisher("/clock", Clock, queue_size=100)  # 创建时钟发布者，发布模拟时间

    # 遍历 ROS bag 文件中的消息
    for topic, msg in read_bag(args.file, args.start, args.duration, progress=True):
        while not rospy.is_shutdown():
            if callback_lock_event.wait(1.0):  # 等待回调锁，确保数据处理同步
                break

        if rospy.is_shutdown():  # 如果 ROS 节点关闭，退出循环
            break

        # 根据话题分发消息到对应的节点处理
        if topic == IMU_TOPIC or topic == IMU_TOPIC_MK_II:  # 处理 IMU 数据
            dead_reckoning_node.imu_sub.callback(msg)  # 调用死 reckoning 节点的 IMU 回调
        elif topic == DVL_TOPIC:  # 处理 DVL（多普勒测速仪）数据
            dead_reckoning_node.dvl_sub.callback(msg)  # 调用死 reckoning 节点的 DVL 回调
        elif topic == DEPTH_TOPIC:  # 处理深度传感器数据
            dead_reckoning_node.depth_sub.callback(msg)  # 调用死 reckoning 节点的深度回调
        elif topic == SONAR_TOPIC or SONAR_TOPIC_UNCOMPRESSED:  # 处理声呐数据
            feature_extraction_node.sonar_sub.callback(msg)  # 调用特征提取节点的声呐回调
        elif topic == GYRO_TOPIC:  # 处理陀螺仪数据
            gyro_node.gyro_sub.callback(msg)  # 调用陀螺仪节点的回调

        # 使用 IMU 数据驱动时钟
        if topic == IMU_TOPIC or topic == IMU_TOPIC_MK_II:
            clock_pub.publish(Clock(msg.header.stamp))  # 发布 IMU 消息的时间戳作为模拟时钟

            # 发布 map 到 world 的坐标变换，以便在 RViz 中以 z 轴向下的坐标系进行可视化
            # 【位姿变换】：这里定义了一个静态的 map 到 world 的变换
            node.tf.sendTransform((0, 0, 0), [1, 0, 0, 0], msg.header.stamp, "map", "world")

# def aracati_offline(args)->None:
#     from rosgraph_msgs.msg import Clock
#     from feature_extraction_node import FeatureExtraction
#     from bruce_slam.utils import io

#     io.offline = True
#     node.save_fig = False
#     node.save_data = False

#     # 只初始化 FeatureExtraction（用于点云处理）和 SLAMNode
#     feature_extraction_node = FeatureExtraction()
#     feature_extraction_node.init_node(SLAM_NS + "feature_extraction/")
#     clock_pub = rospy.Publisher("/clock", Clock, queue_size=100)

#     # 自定义声呐图像到点云转换（参考上一对话）
#     from your_conversion_node import SonarImageToPC  # 假设您已实现
#     sonar_converter = SonarImageToPC()  # 初始化转换节点

#     # 消息缓冲区（手动同步 odom 和 feature）
#     odom_buffer = {}  # 时间戳: Odometry msg
#     feature_buffer = {}  # 时间戳: PointCloud2 msg

#     for topic, msg in read_bag(args.file, args.start, args.duration, progress=True):
#         while not rospy.is_shutdown():
#             if callback_lock_event.wait(1.0):
#                 break
#         if rospy.is_shutdown():
#             break

#         # 处理 Aracati 话题
#         if topic == ARACATI_SONAR_TOPIC:  # /son/compressed
#             feature_msg = sonar_converter.convert_to_pc(msg)  # 转换为 PointCloud2
#             if feature_msg:
#                 feature_buffer[msg.header.stamp.to_sec()] = feature_msg
#         elif topic == ARACATI_ODOM_TOPIC:  # /odom_pose
#             odom_buffer[msg.header.stamp.to_sec()] = msg

#         # 手动同步并调用 SLAM_callback
#         if topic in [ARACATI_SONAR_TOPIC, ARACATI_ODOM_TOPIC]:
#             current_time = msg.header.stamp.to_sec()
#             matched_odom = min(odom_buffer, key=lambda t: abs(t - current_time), default=None) if odom_buffer else None
#             matched_feature = min(feature_buffer, key=lambda t: abs(t - current_time), default=None) if feature_buffer else None
#             if matched_odom and matched_feature and abs(matched_odom - matched_feature) < 0.5:  # slop 0.5s
#                 node.SLAM_callback(feature_buffer[matched_feature], odom_buffer[matched_odom])
#                 # 可选：清理缓冲区
#                 del feature_buffer[matched_feature]
#                 del odom_buffer[matched_odom]

#         # 发布时钟（用消息时间戳）
#         clock_pub.publish(Clock(msg.header.stamp))

#         # 发布静态 TF（map 到 world）
#         node.tf.sendTransform((0, 0, 0), [1, 0, 0, 0], msg.header.stamp, "map", "world")

if __name__ == "__main__":

    # 初始化 ROS 节点，命名为 "slam"
    rospy.init_node("slam", log_level=rospy.INFO)

    # 创建并初始化 SLAM 节点
    node = SLAMNode()
    node.init_node()

    # 解析命令行参数
    args, _ = common_parser().parse_known_args()

    # 根据是否有文件参数，决定运行在线或离线模式
    if not args.file:
        loginfo("Start online slam...")  # 启动在线 SLAM 模式
        rospy.spin()  # 进入 ROS 事件循环，处理实时数据
    else:
        loginfo("Start offline slam...")  # 启动离线 SLAM 模式
        offline(args)  # 调用离线模式处理函数