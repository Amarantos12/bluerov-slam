#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
aracati_bridge_final.py

这个脚本是一个ROS节点，其功能是作为 Bruce SLAM 系统和 Aracati 数据集之间的桥梁。
它订阅 Aracati 数据集中的 /cmd_vel (视为真实速度) 和 /pose_gt (真实位姿) 话题，
然后将这些数据转换并发布为 Bruce SLAM 所需的传感器话题格式和坐标系约定。

核心转换逻辑:
1. 坐标系映射:
   - 源 (Aracati, 标准Twist/Pose): 
     - 线性: x-前进, y-左, z-上
     - 角度: 绕z轴为偏航(Yaw)
   - 目标 (Bruce SLAM, base_link):
     - 线性: X-前进, Y-下, Z-右
     - 角度: 绕Y轴为偏航(Yaw)
2. 数据源:
   - DVL速度来源于 /cmd_vel 的线性速度分量.
   - 深度来源于 /pose_gt 的z坐标.
   - IMU姿态来源于 /pose_gt 的姿态，并进行坐标系旋转以匹配目标约定.
   - IMU角速度来源于 /cmd_vel 的角速度分量，并进行坐标轴映射.
"""

import rospy
from geometry_msgs.msg import TwistStamped, PoseStamped, Quaternion, Vector3
from std_msgs.msg import Header
from message_filters import ApproximateTimeSynchronizer, Subscriber

# 确保你的ROS工作空间能够找到这些自定义消息类型。
# 如果编译时提示找不到，请确认你的环境已经 source 了包含这些消息定义的包。
from rti_dvl.msg import DVL
from bar30_depth.msg import Depth
from sensor_msgs.msg import Imu
import tf.transformations as tf_trans

class AracatiBridgeFinal:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('aracati_bridge_final', anonymous=True)

        # 订阅源话题
        self.cmd_vel_sub = Subscriber('/cmd_vel', TwistStamped)
        self.pose_gt_sub = Subscriber('/pose_gt', PoseStamped)
        
        # 使用时间同步器来确保收到的 cmd_vel 和 pose_gt 消息时间戳大致对齐
        self.ts = ApproximateTimeSynchronizer(
            [self.cmd_vel_sub, self.pose_gt_sub], 
            queue_size=10, 
            slop=0.1  # 允许最大0.1秒的时间戳偏差
        )
        self.ts.registerCallback(self.callback)

        # 初始化目标话题的发布者
        self.dvl_pub = rospy.Publisher('/rti/body_velocity/raw', DVL, queue_size=10)
        self.depth_pub = rospy.Publisher('/bar30/depth/raw', Depth, queue_size=10)
        self.imu_pub = rospy.Publisher('/vn100/imu/raw', Imu, queue_size=10)

        rospy.loginfo("Aracati Final Bridge Initialized.")
        rospy.loginfo("Mapping from standard Twist/Pose to a Y-Down vehicle frame for Bruce SLAM.")

    def callback(self, cmd_vel_msg, pose_gt_msg):
        # 创建一个统一的消息头，使用 cmd_vel 的时间戳
        header = Header()
        header.stamp = cmd_vel_msg.header.stamp
        header.frame_id = 'base_link'  # 所有发布的数据都相对于机器人本体坐标系

        # === 1. DVL 速度转换与发布 ===
        dvl_msg = DVL()
        dvl_msg.header = header
        
        # 坐标系映射:
        # Twist.linear.x (前进) -> base_link.X (前进) -> dvl.velocity.x
        # Twist.linear.y (向左) -> base_link.Z (向右) -> dvl.velocity.z (方向相反，需*-1)
        # Twist.linear.z (向上) -> base_link.Y (向下) -> dvl.velocity.y (方向相反，需*-1)
        dvl_msg.velocity = Vector3(
            x = cmd_vel_msg.twist.linear.x,
            y = -cmd_vel_msg.twist.linear.z,
            z = -cmd_vel_msg.twist.linear.y
        )
        
        # DVL消息的其他字段，由于源数据没有，我们填充为无效值
        dvl_msg.temperature = -1.0
        dvl_msg.altitude = -1.0
        self.dvl_pub.publish(dvl_msg)

        # === 2. 深度信息转换与发布 ===
        depth_msg = Depth()
        depth_msg.header = header
        
        # 深度是沿垂直轴(Y轴, 向下为正)的距离。
        # pose_gt.position.z 是高度(向上为正)。
        # 因此，depth = -height。
        depth_msg.depth = -pose_gt_msg.pose.position.z
        
        # Depth消息的其他字段，填充为默认值
        depth_msg.time = header.stamp.to_sec()
        self.depth_pub.publish(depth_msg)

        # === 3. IMU 数据转换与发布 ===
        imu_msg = Imu()
        imu_msg.header = header

        # --- 姿态 (Orientation) ---
        # 目标: 将一个标准的Z-up姿态，转换为一个Y-down姿态。
        # 核心思想: 提取原始姿态中的偏航角(Yaw)，然后创建一个新的姿态，
        #           该姿态表示绕着目标坐标系的垂直轴(Y轴)旋转相同的偏航角。
        
        # 从/pose_gt的四元数中提取欧拉角 (roll, pitch, yaw)
        euler = tf_trans.euler_from_quaternion([
            pose_gt_msg.pose.orientation.x,
            pose_gt_msg.pose.orientation.y,
            pose_gt_msg.pose.orientation.z,
            pose_gt_msg.pose.orientation.w
        ])
        yaw_angle = euler[2]  # 获取原始的偏航角 (绕标准Z轴)
        
        # 创建一个新的四元数，它代表绕Y轴旋转 yaw_angle。
        # tf.transformations的euler_from_quaternion参数顺序是(roll, pitch, yaw)，
        # 分别对应绕(x, y, z)的旋转。所以我们将yaw_angle放在pitch的位置。
        quat = tf_trans.quaternion_from_euler(0.0, yaw_angle, 0.0)
        imu_msg.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])

        # --- 角速度 (Angular Velocity) ---
        # 目标: 将标准的Twist角速度，映射到base_link坐标系下的角速度。
        # Twist.angular.z (标准偏航，绕z轴) -> base_link偏航 (绕Y轴) -> imu.angular_velocity.y
        # 其他分量(roll, pitch)通常为0，但也进行映射以保持完整性。
        imu_msg.angular_velocity = Vector3(
            x = cmd_vel_msg.twist.angular.x,    # roll -> roll (绕X轴)
            y = cmd_vel_msg.twist.angular.z,    # yaw -> yaw (绕Y轴)
            z = cmd_vel_msg.twist.angular.y     # pitch -> pitch (绕Z轴)
        )

        # --- 协方差和线加速度 ---
        # 为我们有信心的测量值设置一个很小的协方差。
        # 对于没有数据的测量值，将协方差的第一个元素设为-1，这是ROS EKF等节点的惯例，表示“此数据无效”。
        imu_msg.orientation_covariance = [
            0.01, 0, 0,
            0, 0.01, 0,
            0, 0, 0.01
        ]
        imu_msg.angular_velocity_covariance = [
            1e-6, 0, 0,
            0, 1e-6, 0,
            0, 0, 1e-6
        ]
        imu_msg.linear_acceleration.x = 0.0
        imu_msg.linear_acceleration.y = 0.0
        imu_msg.linear_acceleration.z = 0.0
        imu_msg.linear_acceleration_covariance[0] = -1.0  # 标记线加速度数据无效

        self.imu_pub.publish(imu_msg)

if __name__ == '__main__':
    try:
        bridge = AracatiBridgeFinal()
        rospy.spin()  # 保持节点运行，等待消息
    except rospy.ROSInterruptException:
        rospy.loginfo("Aracati bridge node shut down.")
        pass