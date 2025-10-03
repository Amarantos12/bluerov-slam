#!/usr/bin/env python

import rospy
import threading  # 添加threading导入，用于Event
from bruce_slam.utils.io import *
from bruce_slam.slam_ros import SLAMNode
from bruce_slam.utils.topics import *
from bruce_slam.slam_ros_aracati import SLAMNode_aracati

# 定义callback_lock_event（如果原代码中未定义）
callback_lock_event = threading.Event()

def offline(args)->None:
    """运行离线 SLAM 系统，处理 ROS bag 文件中的录制数据

    Args:
        args (Any): 运行系统所需的参数（如文件路径、起始时间、持续时间等）
    """

    # 导入离线模式所需的额外模块
    from rosgraph_msgs.msg import Clock
    from feature_extraction_node import FeatureExtraction
    from bruce_slam.utils import io
    from nav_msgs.msg import Odometry  # 确保导入Odometry

    # 设置离线模式参数
    io.offline = True  # 标记为离线模式
    node.save_fig = False  # 不保存图像
    node.save_data = False  # 不保存数据

    # 初始化所需的节点（只保留feature_extraction，因为sonar已修改）
    feature_extraction_node = FeatureExtraction()  # 特征提取节点
    feature_extraction_node.init_node(SLAM_NS + "feature_extraction/")  # 初始化特征提取节点

    # 创建时钟发布者
    clock_pub = rospy.Publisher("/clock", Clock, queue_size=100)  # 创建时钟发布者，发布模拟时间

    # 创建odometry发布者，用于将/odom_pose直接转发到LOCALIZATION_ODOM_TOPIC
    odom_pub = rospy.Publisher(LOCALIZATION_ODOM_TOPIC, Odometry, queue_size=100)

    # 遍历 ROS bag 文件中的消息
    for topic, msg in read_bag(args.file, args.start, args.duration, progress=True):
        while not rospy.is_shutdown():
            if callback_lock_event.wait(1.0):  # 等待回调锁，确保数据处理同步
                break

        if rospy.is_shutdown():  # 如果 ROS 节点关闭，退出循环
            break

        # 根据话题分发消息到对应的节点处理
        if topic == SONAR_TOPIC or topic == SONAR_TOPIC_UNCOMPRESSED:  # 处理声呐数据（您已修改为/sonar_oculus_node/ping）
            feature_extraction_node.sonar_sub.callback(msg)  # 调用特征提取节点的声呐回调
            clock_pub.publish(Clock(msg.header.stamp))  # 使用sonar时间戳发布clock（备用）
            # 发布 map 到 world 的坐标变换（使用sonar时间戳，如果没有odom）
            node.tf.sendTransform((0, 0, 0), [1, 0, 0, 0], msg.header.stamp, "map", "world")

        elif topic == ARACATI_ODOM_TOPIC:  # 处理/odom_pose，直接发布到LOCALIZATION_ODOM_TOPIC
            # print("*********odom_pub*************")
            odom_pub.publish(msg)  # 直接发布作为odometry输入
            clock_pub.publish(Clock(msg.header.stamp))  # 使用/odom_pose时间戳发布clock（优先）
            # 发布 map 到 world 的坐标变换（使用/odom_pose时间戳）
            node.tf.sendTransform((0, 0, 0), [1, 0, 0, 0], msg.header.stamp, "map", "world")


if __name__ == "__main__":

    # 初始化 ROS 节点，命名为 "slam"
    # print("slam_aracati start")
    rospy.init_node("slam", log_level=rospy.INFO)

    # 创建并初始化 SLAM 节点
    node = SLAMNode_aracati()
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