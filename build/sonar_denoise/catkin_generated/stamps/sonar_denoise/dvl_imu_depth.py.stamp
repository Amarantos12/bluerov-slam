#!/usr/bin/env python

import rospy
from geometry_msgs.msg import TwistStamped, PoseStamped, Quaternion, Vector3
from std_msgs.msg import Header
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rti_dvl.msg import DVL
from bar30_depth.msg import Depth
from sensor_msgs.msg import Imu
import tf.transformations as tf_trans  # For quaternion from euler

class AracatiBridge:
    def __init__(self):
        # 初始化 ROS 节点
        rospy.init_node('aracati_bridge', anonymous=True)

        # 订阅者
        self.cmd_vel_sub = Subscriber('/cmd_vel', TwistStamped)
        self.pose_gt_sub = Subscriber('/pose_gt', PoseStamped)
        # 使用 ApproximateTimeSynchronizer 同步话题，允许 0.1 秒偏差
        self.ts = ApproximateTimeSynchronizer([self.cmd_vel_sub, self.pose_gt_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.callback)

        # 发布者
        self.dvl_pub = rospy.Publisher('/rti/body_velocity/raw', DVL, queue_size=10)
        self.depth_pub = rospy.Publisher('/bar30/depth/raw', Depth, queue_size=10)
        self.imu_pub = rospy.Publisher('/vn100/imu/raw', Imu, queue_size=10)

        rospy.loginfo("Aracati 2017 bridge node initialized. Assuming 2D motion: z=0, roll/pitch=0.")

    def callback(self, cmd_vel_msg, pose_gt_msg):
        # 统一使用 /cmd_vel 的 header 作为时间戳
        header = Header()
        header.stamp = cmd_vel_msg.header.stamp
        header.frame_id = 'base_link'  # 假设 frame_id 为 base_link，可根据需要调整

        # 发布 DVL (/rti/body_velocity/raw)
        dvl_msg = DVL()
        dvl_msg.header = header
        dvl_msg.velocity = Vector3(
            x=cmd_vel_msg.twist.linear.x,
            y=cmd_vel_msg.twist.linear.y,
            z=0.0  # 2D 运动，z 速度设为 0
        )
        dvl_msg.temperature = 0.0  # 默认值，无数据可用
        dvl_msg.altitude = 0.0  # 默认值，无数据可用
        dvl_msg.time = header.stamp.to_sec()
        self.dvl_pub.publish(dvl_msg)

        # 发布 Depth (/bar30/depth/raw)
        depth_msg = Depth()
        depth_msg.header = header
        depth_msg.time = header.stamp.to_sec()  # 使用时间戳作为 time
        depth_msg.pressure_abs = 0.0  # 默认值，无数据可用
        depth_msg.pressure_diff = 0.0  # 默认值，无数据可用
        depth_msg.temperature = 0.0  # 默认值，无数据可用
        depth_msg.depth = 0.0  # 2D 运动，深度设为 0（或使用 pose_gt.position.z 但假设为 0）
        self.depth_pub.publish(depth_msg)

        # 发布 Imu (/vn100/imu/raw)
        imu_msg = Imu()
        imu_msg.header = header

        # 姿态：从 /pose_gt 的 orientation 取 quaternion（假设仅 yaw，非零 roll/pitch 会被保留，但根据 2D 假设可强制 roll/pitch=0）
        # 若要强制 roll/pitch=0，仅保留 yaw
        euler = tf_trans.euler_from_quaternion([
            pose_gt_msg.pose.orientation.x,
            pose_gt_msg.pose.orientation.y,
            pose_gt_msg.pose.orientation.z,
            pose_gt_msg.pose.orientation.w
        ])
        # 强制 roll=0, pitch=0，仅 yaw
        quat = tf_trans.quaternion_from_euler(0.0, 0.0, euler[2])
        imu_msg.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
        imu_msg.orientation_covariance = [0.0] * 9  # 默认协方差为 0

        # 角速度：从 /cmd_vel 的 twist.angular.z 取 yaw rate，其他设为 0
        imu_msg.angular_velocity = Vector3(x=0.0, y=0.0, z=cmd_vel_msg.twist.angular.z)
        imu_msg.angular_velocity_covariance = [0.0] * 9  # 默认协方差为 0

        # 线加速度：无数据可用，设为 0
        imu_msg.linear_acceleration = Vector3(x=0.0, y=0.0, z=0.0)
        imu_msg.linear_acceleration_covariance = [0.0] * 9  # 默认协方差为 0

        self.imu_pub.publish(imu_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        bridge = AracatiBridge()
        bridge.run()
    except rospy.ROSInterruptException:
        pass