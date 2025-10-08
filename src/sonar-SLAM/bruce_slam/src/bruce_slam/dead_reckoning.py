# python imports
import tf
import rospy
import gtsam
import numpy as np

# ros-python imports
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, Imu
from message_filters import ApproximateTimeSynchronizer, Cache, Subscriber

# import custom messages
from kvh_gyro.msg import gyro as GyroMsg
from rti_dvl.msg import DVL
from bar30_depth.msg import Depth

# bruce imports
from bruce_slam.utils.topics import *
from bruce_slam.utils.conversions import *
from bruce_slam.utils.io import *
from bruce_slam.utils.visualization import ros_colorline_trajectory

import math
from std_msgs.msg import String, Float32
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class DeadReckoningNode(object):
    '''A class to support dead reckoning using DVL and IMU readings
    支持使用DVL和IMU数据进行死 reckoning（死推定位）的类，用于水下机器人定位
    '''
    def __init__(self):
        self.pose = None #vehicle pose
        # 机器人当前位姿（位置和姿态），使用GTSAM的Pose3表示
        self.prev_time = None #previous reading time
        # 上一次传感器数据的时间戳
        self.prev_vel = None #previous reading velocity
        # 上一次DVL测量的速度
        self.keyframes = [] #keyframe list
        # 关键帧列表，存储时间戳和对应的位姿

        # Force yaw at origin to be aligned with x axis
        # 强制初始偏航角与x轴对齐
        self.imu_yaw0 = None
        # 初始偏航角，用于校准
        self.imu_pose = [0, 0, 0, -np.pi / 2, 0, 0]
        # IMU的安装姿态（默认值），表示相对于机器人坐标系的偏移
        self.imu_rot = None
        # IMU的旋转矩阵，从imu_pose中提取
        self.dvl_max_velocity = 0.3
        # DVL速度的最大阈值（m/s），用于检测异常速度

        # Create a new key pose when
        # - |ti - tj| > min_duration and
        # - |xi - xj| > max_translation or
        # - |ri - rj| > max_rotation
        # 生成新关键帧的条件：
        # - 时间间隔 |ti - tj| 超过最小持续时间
        # - 平移距离 |xi - xj| 超过最大平移阈值 或
        # - 旋转角度 |ri - rj| 超过最大旋转阈值
        self.keyframe_duration = None
        # 关键帧时间间隔阈值
        self.keyframe_translation = None
        # 关键帧平移距离阈值
        self.keyframe_rotation = None
        # 关键帧旋转角度阈值
        self.dvl_error_timer = 0.0
        # DVL速度异常的持续时间计数器

        # place holder for multi-robot SLAM
        # 多机器人SLAM的占位符
        self.rov_id = ""
        # 机器人ID，用于多机器人SLAM场景（当前未使用）

        # 初始化 odom 文件句柄，用于存储 evo_traj 格式的轨迹数据
        self.odom_file_path = "/home/hzr/catkin_ws/src/sonar-SLAM/output/aracati_sonargan_odom.txt"
        self.odom_file = open(self.odom_file_path, "w")  # 以写模式打开文件（覆盖现有内容）



    def init_node(self, ns="~")->None:
        """Init the node, fetch all paramaters from ROS
        初始化ROS节点，从ROS参数服务器获取所有参数

        Args:
            ns (str, optional): The namespace of the node. Defaults to "~".
            ns (str, 可选): 节点的命名空间，默认为"~"。
        """
        # Parameters for Node
        # 节点参数配置
        self.imu_pose = rospy.get_param(ns + "imu_pose")
        # 从ROS参数获取IMU安装姿态
        self.imu_pose = n2g(self.imu_pose, "Pose3")
        # 将IMU姿态转换为GTSAM的Pose3对象
        self.imu_rot = self.imu_pose.rotation()
        # 提取IMU姿态的旋转矩阵
        self.dvl_max_velocity = rospy.get_param(ns + "dvl_max_velocity")
        # 获取DVL最大速度阈值
        self.keyframe_duration = rospy.get_param(ns + "keyframe_duration")
        # 获取关键帧时间间隔阈值
        self.keyframe_translation = rospy.get_param(ns + "keyframe_translation")
        # 获取关键帧平移距离阈值
        self.keyframe_rotation = rospy.get_param(ns + "keyframe_rotation")
        # 获取关键帧旋转角度阈值

        # Subscribers and caches
        # 初始化订阅者和缓存
        self.dvl_sub = Subscriber(DVL_TOPIC, DVL)
        # 订阅DVL话题，获取速度数据
        self.gyro_sub = Subscriber(GYRO_INTEGRATION_TOPIC, Odometry)
        # 订阅光纤陀螺仪话题，获取姿态数据
        self.depth_sub = Subscriber(DEPTH_TOPIC, Depth)
        # 订阅深度传感器话题，获取深度数据
        self.depth_cache = Cache(self.depth_sub, 1)
        # 缓存最近的深度消息

        if rospy.get_param(ns + "imu_version") == 1:
            self.imu_sub = Subscriber(IMU_TOPIC, Imu)
            # 订阅IMU话题（版本1）
        elif rospy.get_param(ns + "imu_version") == 2:
            self.imu_sub = Subscriber(IMU_TOPIC_MK_II, Imu)
            # 订阅IMU话题（版本2）

        # Use point cloud for visualization
        # 使用点云消息进行轨迹可视化
        self.traj_pub = rospy.Publisher(
            "traj_dead_reck", PointCloud2, queue_size=10)
        # 发布轨迹点云消息

        self.odom_pub = rospy.Publisher(
            LOCALIZATION_ODOM_TOPIC, Odometry, queue_size=10)
        # 发布里程计消息

        # are we using the FOG gyroscope?
        # 是否使用光纤陀螺仪（FOG）？
        self.use_gyro = rospy.get_param(ns + "use_gyro")
        # 获取是否使用光纤陀螺仪的参数

        # define the callback, are we using the gyro or the VN100?
        # 定义回调函数，根据是否使用光纤陀螺仪选择不同的回调
        if self.use_gyro:
            self.ts = ApproximateTimeSynchronizer([self.imu_sub, self.dvl_sub, self.gyro_sub], 300, .1)
            # 使用光纤陀螺仪，同步IMU、DVL和光纤陀螺仪数据，允许0.1秒时间差
            self.ts.registerCallback(self.callback_with_gyro)
        else:
            self.ts = ApproximateTimeSynchronizer([self.imu_sub, self.dvl_sub], 200, .1)
            # 不使用光纤陀螺仪，同步IMU和DVL数据
            self.ts.registerCallback(self.callback)

        self.tf = tf.TransformBroadcaster()
        # 初始化TF变换广播器

        loginfo("Localization node is initialized")
        # 记录日志：定位节点已初始化


    def callback(self, imu_msg:Imu, dvl_msg:DVL)->None:
        """Handle the dead reckoning using the VN100 and DVL only. Fuse and publish an odometry message.
        使用VN100（IMU）和DVL进行死 reckoning，融合数据并发布里程计消息

        Args:
            imu_msg (Imu): the message from VN100
            imu_msg (Imu): VN100的IMU消息
            dvl_msg (DVL): the message from the DVL
            dvl_msg (DVL): DVL消息
        """
        #get the previous depth message
        # 获取最近的深度消息
        depth_msg = self.depth_cache.getLast()
        #if there is no depth message, then skip this time step
        # 如果没有深度消息，跳过当前时间步
        if depth_msg is None:
            return

        #check the delay between the depth message and the DVL
        # 检查深度消息和DVL消息的时间差
        dd_delay = (depth_msg.header.stamp - dvl_msg.header.stamp).to_sec()
        #print(dd_delay)
        if abs(dd_delay) > 1.0:
            logdebug("Missing depth message for {}".format(dd_delay))
            # 如果时间差超过1秒，记录调试日志

        #convert the imu message from msg to gtsam rotation object
        # 将IMU消息的四元数转换为GTSAM旋转对象
        rot = r2g(imu_msg.orientation)
        rot = rot.compose(self.imu_rot.inverse())
        # 校准IMU姿态，去除安装偏移

        #if we have no yaw yet, set this one as zero
        # 如果没有初始偏航角，将当前偏航角设为零点
        if self.imu_yaw0 is None:
            self.imu_yaw0 = rot.yaw()

        # Get a rotation matrix
        # if use_gyro has the same value in Kalman and DeadReck, use this line
        # 获取旋转矩阵
        # 如果卡尔曼滤波和死 reckoning 的 use_gyro 参数相同，使用此行
        rot = gtsam.Rot3.Ypr(rot.yaw()-self.imu_yaw0, rot.pitch(), np.radians(90)+rot.roll())
        # 构造旋转矩阵，校准偏航角，调整横滚角（加90度）
        # if use_gyro = True in Kalman and use_gyro = False in DeadReck, use this line:
        # 如果卡尔曼滤波中 use_gyro=True 而死 reckoning 中 use_gyro=False，使用此行：
        # rot = gtsam.Rot3.Ypr(rot.yaw()-self.imu_yaw0, rot.pitch(), np.radians(90)+rot.roll())

        # parse the DVL message into an array of velocites
        # 将DVL消息解析为速度数组（x, y, z）
        vel = np.array([dvl_msg.velocity.x, dvl_msg.velocity.y, dvl_msg.velocity.z])

        # package the odom message and publish it
        # 打包里程计消息并发布
        self.send_odometry(vel,rot,dvl_msg.header.stamp,depth_msg.depth)


    def callback_with_gyro(self, imu_msg:Imu, dvl_msg:DVL, gyro_msg:GyroMsg)->None:
        """Handle the dead reckoning state estimate using the fiber optic gyro. Here we use the
        Gyro as a means of getting the yaw estimate, roll and pitch are still VN100.
        使用光纤陀螺仪进行死 reckoning 状态估计，使用光纤陀螺仪提供偏航角，俯仰和横滚仍使用VN100

        Args:
            imu_msg (Imu): the vn100 imu message
            imu_msg (Imu): VN100的IMU消息
            dvl_msg (DVL): the DVL message
            dvl_msg (DVL): DVL消息
            gyro_msg (GyroMsg): the euler angles from the gyro
            gyro_msg (GyroMsg): 光纤陀螺仪的欧拉角消息
        """
        # decode the gyro message
        # 解析光纤陀螺仪消息，提取偏航角
        gyro_yaw = r2g(gyro_msg.pose.pose).rotation().yaw()

        #get the previous depth message
        # 获取最近的深度消息
        depth_msg = self.depth_cache.getLast()

        #if there is no depth message, then skip this time step
        # 如果没有深度消息，跳过当前时间步
        if depth_msg is None:
            return

        #check the delay between the depth message and the DVL
        # 检查深度消息和DVL消息的时间差
        dd_delay = (depth_msg.header.stamp - dvl_msg.header.stamp).to_sec()
        #print(dd_delay)
        if abs(dd_delay) > 1.0:
            logdebug("Missing depth message for {}".format(dd_delay))
            # 如果时间差超过1秒，记录调试日志

        #convert the imu message from msg to gtsam rotation object
        # 将IMU消息的四元数转换为GTSAM旋转对象
        rot = r2g(imu_msg.orientation)
        rot = rot.compose(self.imu_rot.inverse())
        # 校准IMU姿态，去除安装偏移

        # Get a rotation matrix
        # 构造旋转矩阵，使用光纤陀螺仪的偏航角，IMU的俯仰和横滚
        rot = gtsam.Rot3.Ypr(gyro_yaw, rot.pitch(), rot.roll())

        #parse the DVL message into an array of velocites
        # 将DVL消息解析为速度数组（x, y, z）
        vel = np.array([dvl_msg.velocity.x, dvl_msg.velocity.y, dvl_msg.velocity.z])

        # package the odom message and publish it
        # 打包里程计消息并发布
        self.send_odometry(vel,rot,dvl_msg.header.stamp,depth_msg.depth)


    def send_odometry(self,vel:np.array,rot:gtsam.Rot3,dvl_time:rospy.Time,depth:float)->None:
        """Package the odometry given all the DVL, rotation matrix, and depth
        根据DVL速度、旋转矩阵和深度数据，计算并打包里程计消息

        Args:
            vel (np.array): a numpy array (1D) of the DVL velocities
            vel (np.array): DVL速度的1D数组（x, y, z）
            rot (gtsam.Rot3): the rotation matrix of the vehicle
            rot (gtsam.Rot3): 机器人的旋转矩阵
            dvl_time (rospy.Time): the time stamp for the DVL message
            dvl_time (rospy.Time): DVL消息的时间戳
            depth (float): vehicle depth
            depth (float): 机器人深度
        """

        #if the DVL message has any velocity above the max threhold do some error handling
        # 如果DVL速度超过最大阈值，进行异常处理
        if np.any(np.abs(vel) > self.dvl_max_velocity):
            if self.pose:
                # 累加异常持续时间
                self.dvl_error_timer += (dvl_time - self.prev_time).to_sec()
                if self.dvl_error_timer > 5.0:
                    logwarn(
                        "DVL velocity ({:.1f}, {:.1f}, {:.1f}) exceeds max velocity {:.1f} for {:.1f} secs.".format(
                            vel[0],
                            vel[1],
                            vel[2],
                            self.dvl_max_velocity,
                            self.dvl_error_timer,
                        )
                    )
                    # 如果异常持续超过5秒，记录警告日志
                vel = self.prev_vel
                # 使用上一次速度代替当前异常速度
            else:
                return
                # 如果没有初始位姿，跳过
        else:
            self.dvl_error_timer = 0.0
            # 重置异常计时器

        if self.pose:
            # figure out how far we moved in the body frame using the DVL message
            # 使用DVL消息计算在机器人坐标系中的位移
            dt = (dvl_time - self.prev_time).to_sec()
            # 计算时间差
            dv = (vel + self.prev_vel) * 0.5
            # 计算平均速度
            trans = dv * dt
            # 计算位移

            # get a rotation matrix with only roll and pitch
            # 获取仅包含俯仰和横滚的旋转矩阵（忽略偏航）
            rotation_flat = gtsam.Rot3.Ypr(0, rot.pitch(), rot.roll())

            # transform our movement to the global frame
            # 将位移转换到全局坐标系
            #trans[2] = -trans[2]
            #trans = trans.dot(rotation_flat.matrix())

            # propagate our movement forward using the GTSAM utilities
            # 使用GTSAM工具向前传播位姿
            local_point = gtsam.Point2(trans[0], trans[1])
            # 构造2D局部位移点

            pose2 = gtsam.Pose2(
                self.pose.x(), self.pose.y(), self.pose.rotation().yaw()
            )
            # 使用当前位姿的x、y和偏航角构造2D位姿
            point = pose2.transformFrom(local_point)
            # 将局部位移转换为全局坐标系

            self.pose = gtsam.Pose3(
                rot, gtsam.Point3(point[0], point[1], depth)
            )
            # 更新3D位姿，z轴使用深度值

        else:
            # init the pose
            # 初始化位姿
            self.pose = gtsam.Pose3(rot, gtsam.Point3(0, 0, depth))
            # 设置初始位置为(0, 0, depth)，姿态为输入的旋转矩阵

        # log the this timesteps messages for next time
        # 记录当前时间步的数据，供下一次使用
        self.prev_time = dvl_time
        self.prev_vel = vel

        new_keyframe = False
        # 判断是否生成新关键帧
        if not self.keyframes:
            new_keyframe = True
            # 如果没有关键帧，创建第一个关键帧
        else:
            duration = self.prev_time.to_sec() - self.keyframes[-1][0]
            # 计算与上一个关键帧的时间间隔
            if duration > self.keyframe_duration:
                odom = self.keyframes[-1][1].between(self.pose)
                # 计算当前位姿与上一个关键帧位姿的相对变换
                odom = g2n(odom)
                # 转换为numpy数组
                translation = np.linalg.norm(odom[:3])
                # 计算平移距离
                rotation = abs(odom[-1])
                # 计算旋转角度

                if (
                    translation > self.keyframe_translation
                    or rotation > self.keyframe_rotation
                ):
                    new_keyframe = True
                    # 如果平移或旋转超过阈值，创建新关键帧

        if new_keyframe:
            self.keyframes.append((self.prev_time.to_sec(), self.pose))
            # 添加新关键帧（时间戳，位姿）
        self.publish_pose(new_keyframe)
        # 发布位姿和轨迹


    def publish_pose(self, publish_traj:bool=False)->None:
        """Publish the pose
        发布位姿和轨迹

        Args:
            publish_traj (bool, optional): Are we publishing the whole set of keyframes?. Defaults to False.
            publish_traj (bool, 可选): 是否发布完整的关键帧轨迹？默认为False
        """
        if self.pose is None:
            return
            # 如果位姿为空，跳过

        header = rospy.Header()
        header.stamp = self.prev_time
        header.frame_id = "odom"
        # 设置消息头，时间戳和参考坐标系为"odom"

        odom_msg = Odometry()
        odom_msg.header = header
        # pose in odom frame
        # 位姿在odom坐标系中
        odom_msg.pose.pose = g2r(self.pose)
        # 将GTSAM位姿转换为ROS Pose消息
        # twist in local frame
        # 速度在机器人局部坐标系中
        odom_msg.child_frame_id = "base_link"
        # 设置子坐标系为"base_link"
        # Local planer behaves worse
        # 局部规划器表现较差，注释掉速度发布
        # odom_msg.twist.twist.linear.x = self.prev_vel[0]
        # odom_msg.twist.twist.linear.y = self.prev_vel[1]
        # odom_msg.twist.twist.linear.z = self.prev_vel[2]
        # odom_msg.twist.twist.angular.x = self.prev_omega[0]
        # odom_msg.twist.twist.angular.y = self.prev_omega[1]
        # odom_msg.twist.twist.angular.z = self.prev_omega[2]
        odom_msg.twist.twist.linear.x = 0
        odom_msg.twist.twist.linear.y = 0
        odom_msg.twist.twist.linear.z = 0
        odom_msg.twist.twist.angular.x = 0
        odom_msg.twist.twist.angular.y = 0
        odom_msg.twist.twist.angular.z = 0
        # 当前将速度设为零
        self.odom_pub.publish(odom_msg)
        # 发布里程计消息

        # 存储 odom 数据到文件（evo_traj 格式: timestamp tx ty tz qx qy qz qw）
        timestamp = odom_msg.header.stamp.to_sec()
        tx = odom_msg.pose.pose.position.x
        ty = odom_msg.pose.pose.position.y
        tz = odom_msg.pose.pose.position.z
        qx = odom_msg.pose.pose.orientation.x
        qy = odom_msg.pose.pose.orientation.y
        qz = odom_msg.pose.pose.orientation.z
        qw = odom_msg.pose.pose.orientation.w
        self.odom_file.write(f"{timestamp:.9f} {tx} {ty} {tz} {qx} {qy} {qz} {qw}\n")
        self.odom_file.flush()  # 确保数据写入磁盘

        p = odom_msg.pose.pose.position
        q = odom_msg.pose.pose.orientation
        self.tf.sendTransform(
            (p.x, p.y, p.z), (q.x, q.y, q.z, q.w), header.stamp, "base_link", "odom"
        )
        # 使用TF广播器发布odom到base_link的坐标变换

        if publish_traj:
            # print("**************")
            traj = np.array([g2n(pose) for _, pose in self.keyframes])
            # 将关键帧位姿转换为numpy数组
            traj_msg = ros_colorline_trajectory(traj)
            # 转换为点云消息用于轨迹可视化
            traj_msg.header = header
            self.traj_pub.publish(traj_msg)
            # 发布轨迹点云消息
            
    def __del__(self):
        """Destructor to close the odometry file handle"""
        if hasattr(self, 'odom_file') and self.odom_file:
            self.odom_file.close()
