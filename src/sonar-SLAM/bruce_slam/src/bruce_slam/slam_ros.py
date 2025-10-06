#!/usr/bin/env python

# Python 导入
import threading
import tf
import rospy
import cv_bridge
from nav_msgs.msg import Odometry
from message_filters import Subscriber
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseWithCovarianceStamped
from message_filters import ApproximateTimeSynchronizer

# Bruce 模块导入
from bruce_slam.utils.io import *
from bruce_slam.utils.conversions import *
from bruce_slam.utils.visualization import *
from bruce_slam.slam import SLAM, Keyframe
from bruce_slam import pcl

# Argonaut 模块导入
from sonar_oculus.msg import OculusPing


class SLAMNode(SLAM):
    """SLAM 节点类，将 slam.py 的功能集成到 ROS 环境中
    """
    
    def __init__(self):
        super(SLAMNode, self).__init__()  # 调用父类 SLAM 的构造函数

        # 初始化线程锁
        self.lock = threading.RLock()
        self.pub_num,self.slam_callback_num = 0,0

        # 初始化 pose 文件句柄，用于存储 evo_traj 格式的轨迹数据
        self.pose_file_path = "/home/hzr/catkin_ws/src/sonar-SLAM/output/aracati_sonargan_fake_pose.txt"
        self.pose_file = open(self.pose_file_path, "w")  # 以写模式打开文件（覆盖现有内容）

    def init_node(self, ns="~")->None:
        """配置 SLAM 节点，加载参数并初始化订阅者和发布者

        Args:
            ns (str, optional): 节点的命名空间，默认为 "~"
        """
        # print("SLAMNode init_node")

        # 关键帧参数，控制添加关键帧的频率
        self.keyframe_duration = rospy.get_param(ns + "keyframe_duration")  # 关键帧时间间隔
        self.keyframe_duration = rospy.Duration(self.keyframe_duration)  # 转换为 ROS Duration
        self.keyframe_translation = rospy.get_param(ns + "keyframe_translation")  # 关键帧平移阈值
        self.keyframe_rotation = rospy.get_param(ns + "keyframe_rotation")  # 关键帧旋转阈值

        # SLAM 参数，是否启用 SLAM（否则仅使用死 reckoning）
        self.enable_slam = rospy.get_param(ns + "enable_slam")
        print("SLAM 状态: ", self.enable_slam)

        # 噪声模型参数
        self.prior_sigmas = rospy.get_param(ns + "prior_sigmas")  # 先验噪声
        self.odom_sigmas = rospy.get_param(ns + "odom_sigmas")  # 里程计噪声
        self.icp_odom_sigmas = rospy.get_param(ns + "icp_odom_sigmas")  # ICP 里程计噪声

        # 地图降采样分辨率
        self.point_resolution = rospy.get_param(ns + "point_resolution")

        # 序列扫描匹配（SSM）参数
        self.ssm_params.enable = rospy.get_param(ns + "ssm/enable")  # 是否启用 SSM
        self.ssm_params.min_points = rospy.get_param(ns + "ssm/min_points")  # 最小点数
        self.ssm_params.max_translation = rospy.get_param(ns + "ssm/max_translation")  # 最大平移
        self.ssm_params.max_rotation = rospy.get_param(ns + "ssm/max_rotation")  # 最大旋转
        self.ssm_params.target_frames = rospy.get_param(ns + "ssm/target_frames")  # 目标帧数
        print("序列扫描匹配: ", self.ssm_params.enable)

        # 非序列扫描匹配（NSSM，闭环检测）参数
        self.nssm_params.enable = rospy.get_param(ns + "nssm/enable")  # 是否启用 NSSM
        self.nssm_params.min_st_sep = rospy.get_param(ns + "nssm/min_st_sep")  # 最小时间间隔
        self.nssm_params.min_points = rospy.get_param(ns + "nssm/min_points")  # 最小点数
        self.nssm_params.max_translation = rospy.get_param(ns + "nssm/max_translation")  # 最大平移
        self.nssm_params.max_rotation = rospy.get_param(ns + "nssm/max_rotation")  # 最大旋转
        self.nssm_params.source_frames = rospy.get_param(ns + "nssm/source_frames")  # 源帧数
        self.nssm_params.cov_samples = rospy.get_param(ns + "nssm/cov_samples")  # 协方差样本数
        print("非序列扫描匹配: ", self.nssm_params.enable)

        # 闭环检测的成对一致性最大化（PCM）参数，用于剔除异常值
        self.pcm_queue_size = rospy.get_param(ns + "pcm_queue_size")  # PCM 队列大小
        self.min_pcm = rospy.get_param(ns + "min_pcm")  # 最小 PCM 值

        # 点云与里程计的最大时间同步延迟
        self.feature_odom_sync_max_delay = 0.5
        # self.feature_odom_sync_max_delay = 1

        # 定义订阅话题
        self.feature_sub = Subscriber(SONAR_FEATURE_TOPIC, PointCloud2)  # 声呐点云话题
        self.odom_sub = Subscriber(LOCALIZATION_ODOM_TOPIC, Odometry)  # 里程计话题

        # 定义时间同步策略
        self.time_sync = ApproximateTimeSynchronizer(
            [self.feature_sub, self.odom_sub], 200, 
            self.feature_odom_sync_max_delay, allow_headerless=False)  # 同步点云和里程计

        # 注册同步回调函数
        self.time_sync.registerCallback(self.SLAM_callback)

        # 定义位姿发布者
        self.pose_pub = rospy.Publisher(
            SLAM_POSE_TOPIC, PoseWithCovarianceStamped, queue_size=10)  # 发布优化后的位姿

        # 定义死 reckoning 话题发布者
        self.odom_pub = rospy.Publisher(SLAM_ODOM_TOPIC, Odometry, queue_size=10)  # 发布里程计数据

        # 定义轨迹发布者
        self.traj_pub = rospy.Publisher(
            SLAM_TRAJ_TOPIC, PointCloud2, queue_size=1, latch=True)  # 发布轨迹点云

        # 定义约束发布者
        self.constraint_pub = rospy.Publisher(
            SLAM_CONSTRAINT_TOPIC, Marker, queue_size=1, latch=True)  # 发布因子图约束

        # 定义点云发布者
        self.cloud_pub = rospy.Publisher(
            SLAM_CLOUD_TOPIC, PointCloud2, queue_size=1, latch=True)  # 发布降采样点云地图

        # 定义 TF 广播器，用于发布坐标变换
        self.tf = tf.TransformBroadcaster()

        # 初始化 CV 桥接对象
        self.CVbridge = cv_bridge.CvBridge()

        # 从 YAML 文件加载 ICP 配置
        icp_config = rospy.get_param(ns + "icp_config")
        self.icp.loadFromYaml(icp_config)
        
        # 定义机器人 ID（用于多机器人 SLAM，此处未使用）
        self.rov_id = ""

        # 调用配置函数
        self.configure()
        loginfo("SLAM 节点初始化完成")

    @add_lock
    def sonar_callback(self, ping:OculusPing)->None:
        """订阅声呐消息以配置 Oculus 属性，仅订阅一次。
        假设声呐配置变化不大。

        Args:
            ping (OculusPing): 声呐消息
        """
        
        self.oculus.configure(ping)
        self.sonar_sub.unregister()  # 配置后取消订阅

    @add_lock
    def SLAM_callback(self, feature_msg:PointCloud2, odom_msg:Odometry)->None:
        """SLAM 回调函数，订阅声呐点云和里程计消息，
        处理整个 SLAM 系统并发布地图、位姿和约束。

        Args:
            feature_msg (PointCloud2): 输入的声呐点云
            odom_msg (Odometry): 输入的 DVL/IMU 状态估计
        """

        self.slam_callback_num += 1
        # print(f"SLAM_callback {self.slam_callback_num}")
        # 获取线程锁
        self.lock.acquire()

        # 从点云获取 ROS 时间戳
        time = feature_msg.header.stamp

        # 从里程计消息获取死 reckoning 位姿，转换为 GTSAM 格式
        dr_pose3 = r2g(odom_msg.pose.pose)

        # 初始化新的关键帧
        frame = Keyframe(False, time, dr_pose3)

        # 将点云消息转换为 2D numpy 数组
        points = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(feature_msg)
        points = np.c_[points[:,0], -1 * points[:,2]]

        # 如果点云为空或包含无效值，则标记关键帧无效
        if len(points) and np.isnan(points[0, 0]):
            frame.status = False
        else:
            frame.status = self.is_keyframe(frame)  # 判断是否为关键帧

        # 设置关键帧的速度信息
        frame.twist = odom_msg.twist.twist

        # 使用死 reckoning 更新关键帧位姿
        if self.keyframes:
            dr_odom = self.current_keyframe.dr_pose.between(frame.dr_pose)  # 计算帧间里程计变化
            pose = self.current_keyframe.pose.compose(dr_odom)  # 融合位姿
            frame.update(pose)

        # 检查关键帧状态，是否需要添加关键帧（基于死 reckoning 的移动距离）
        if frame.status:
            # 添加点云到关键帧
            frame.points = points

            # 执行序列扫描匹配
            if not self.keyframes:
                self.add_prior(frame)  # 第一个关键帧添加先验
            else:
                self.add_sequential_scan_matching(frame)  # 添加序列扫描匹配

            # 更新因子图
            self.update_factor_graph(frame)

            # 如果启用闭环检测且检测到闭环，重新更新因子图
            if self.nssm_params.enable and self.add_nonsequential_scan_matching():
                self.update_factor_graph()
            
        # 更新当前帧并发布所有话题
        self.current_frame = frame
        self.publish_all()
        self.lock.release()  # 释放线程锁

    def publish_all(self)->None:
        """发布所有输出话题，包括轨迹、约束、点云和完整的 GTSAM 实例
        """
        if not self.keyframes:  # 如果没有关键帧，直接返回
            print("no keyframe")
            return

        self.publish_pose()  # 发布位姿
        if self.current_frame.status:  # 仅在关键帧更新时发布其他内容
            self.publish_trajectory()  # 发布轨迹
            self.publish_constraint()  # 发布约束
            self.publish_point_cloud()  # 发布点云

    def publish_pose(self)->None:
        """将死 reckoning 位姿与 SLAM 估计结合，发布实时 TF 变换
        """

        self.pub_num += 1
        # print(f"publish pose {self.pub_num}")
        # 定义带协方差的位姿消息
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.current_frame.time
        if self.rov_id == "":
            pose_msg.header.frame_id = "map"
        else:
            pose_msg.header.frame_id = self.rov_id + "_map"
        pose_msg.pose.pose = g2r(self.current_frame.pose3)  # 转换为 ROS 格式

        # 设置协方差（FIXME: 应使用 current_frame 的协方差）
        cov = 1e-4 * np.identity(6, np.float32)
        cov[np.ix_((0, 1, 5), (0, 1, 5))] = self.current_keyframe.transf_cov
        pose_msg.pose.covariance = cov.ravel().tolist()
        self.pose_pub.publish(pose_msg)  # 发布到 SLAM_POSE_TOPIC

        # 存储 pose 数据到文件（evo_traj 格式: timestamp tx ty tz qx qy qz qw）
        timestamp = pose_msg.header.stamp.to_sec()
        tx = pose_msg.pose.pose.position.x
        ty = pose_msg.pose.pose.position.y
        tz = pose_msg.pose.pose.position.z
        qx = pose_msg.pose.pose.orientation.x
        qy = pose_msg.pose.pose.orientation.y
        qz = pose_msg.pose.pose.orientation.z
        qw = pose_msg.pose.pose.orientation.w
        self.pose_file.write(f"{timestamp:.9f} {tx} {ty} {tz} {qx} {qy} {qz} {qw}\n")
        self.pose_file.flush()  # 确保数据写入磁盘

        # 计算 odom 到 map 的变换
        o2m = self.current_frame.pose3.compose(self.current_frame.dr_pose3.inverse())
        o2m = g2r(o2m)
        p = o2m.position
        q = o2m.orientation
        self.tf.sendTransform(
            (p.x, p.y, p.z),
            [q.x, q.y, q.z, q.w],
            self.current_frame.time,
            "odom",
            "map",
        )  # 发布 odom 到 map 的 TF 变换

        # 发布里程计消息
        odom_msg = Odometry()
        odom_msg.header = pose_msg.header
        odom_msg.pose.pose = pose_msg.pose.pose
        if self.rov_id == "":
            odom_msg.child_frame_id = "base_link"
        else:
            odom_msg.child_frame_id = self.rov_id + "_base_link"
        odom_msg.twist.twist = self.current_frame.twist
        self.odom_pub.publish(odom_msg)  # 发布到 SLAM_ODOM_TOPIC
        # print(f"pose_msg: {pose_msg}")
        # print(f"odom_msg: {odom_msg}")

    def publish_constraint(self)->None:
        """发布因子图中的位姿约束（序列和非序列约束）
        """

        # 定义约束列表
        links = []

        # 遍历所有关键帧
        for x, kf in enumerate(self.keyframes[1:], 1):
            # 添加序列约束（绿色）
            p1 = self.keyframes[x - 1].pose3.x(), self.keyframes[x - 1].pose3.y(), self.keyframes[x - 1].dr_pose3.z()
            p2 = self.keyframes[x].pose3.x(), self.keyframes[x].pose3.y(), self.keyframes[x].dr_pose3.z()
            links.append((p1, p2, "green"))

            # 添加闭环约束（红色）
            for k, _ in self.keyframes[x].constraints:
                p0 = self.keyframes[k].pose3.x(), self.keyframes[k].pose3.y(), self.keyframes[k].dr_pose3.z()
                links.append((p0, p2, "red"))

        # 如果存在约束，发布为 Marker 消息
        if links:
            link_msg = ros_constraints(links)
            link_msg.header.stamp = self.current_keyframe.time
            if self.rov_id != "":
                link_msg.header.frame_id = self.rov_id + "_map"
            self.constraint_pub.publish(link_msg)  # 发布到 SLAM_CONSTRAINT_TOPIC

    def publish_trajectory(self)->None:
        """发布 3D 轨迹，以点云形式表示 [x, y, z, roll, pitch, yaw, index]
        """

        # 获取所有关键帧的位姿
        poses = np.array([g2n(kf.pose3) for kf in self.keyframes])

        # 转换为 ROS 彩色线消息
        traj_msg = ros_colorline_trajectory(poses)
        traj_msg.header.stamp = self.current_keyframe.time
        if self.rov_id == "":
            traj_msg.header.frame_id = "map"
        else:
            traj_msg.header.frame_id = self.rov_id + "_map"
        self.traj_pub.publish(traj_msg)  # 发布到 SLAM_TRAJ_TOPIC

    def publish_point_cloud(self)->None:
        """发布降采样的 3D 点云（z=0），最后一列表示观测到的关键帧索引
        """

        # 初始化空点云数组
        all_points = [np.zeros((0, 2), np.float32)]

        # 关键帧索引列表
        all_keys = []

        # 遍历所有关键帧，基于 SLAM 估计注册点云
        for key in range(len(self.keyframes)):
            # 解析位姿
            pose = self.keyframes[key].pose

            # 获取注册后的点云
            transf_points = self.keyframes[key].transf_points

            # 添加到列表
            all_points.append(transf_points)
            all_keys.append(key * np.ones((len(transf_points), 1)))

        # 合并点云和索引
        all_points = np.concatenate(all_points)
        all_keys = np.concatenate(all_keys)

        # 使用 PCL 降采样点云
        sampled_points, sampled_keys = pcl.downsample(
            all_points, all_keys, self.point_resolution
        )

        # 转换为 ROS xyzi 格式
        sampled_xyzi = np.c_[sampled_points, np.zeros_like(sampled_keys), sampled_keys]
        
        # 如果点云为空，直接返回
        if len(sampled_xyzi) == 0:
            return

        # 转换为 ROS 点云消息并发布
        cloud_msg = n2r(sampled_xyzi, "PointCloudXYZI")
        cloud_msg.header.stamp = self.current_keyframe.time
        if self.rov_id == "":
            cloud_msg.header.frame_id = "map"
        else:
            cloud_msg.header.frame_id = self.rov_id + "_map"
        self.cloud_pub.publish(cloud_msg)  # 发布到 SLAM_CLOUD_TOPIC

    def __del__(self):
        """析构函数，关闭文件句柄"""
        if hasattr(self, 'pose_file') and self.pose_file:
            self.pose_file.close()
