#!/usr/bin/env python
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import PointCloud2, Image
import cv_bridge
import ros_numpy

from bruce_slam.utils.io import *
from bruce_slam.utils.topics import *
from bruce_slam.utils.conversions import *
from bruce_slam.utils.visualization import apply_custom_colormap
# from bruce_slam.feature import FeatureExtraction
from bruce_slam import pcl
import matplotlib.pyplot as plt
from sonar_oculus.msg import OculusPing, OculusPingUncompressed
from scipy.interpolate import interp1d

from .utils import *
from .sonar import *

from bruce_slam.CFAR import CFAR

# from bruce_slam.bruce_slam import sonar

class FeatureExtraction(object):
    '''类用于从声呐图像中提取特征，使用CFAR算法
    订阅声呐驱动话题，并发布点云数据
    '''

    def __init__(self):
        '''类构造函数，从yaml文件读取所有参数，无需传入参数
        '''

        # 声呐设备信息
        self.oculus = OculusProperty()

        # CFAR算法默认参数
        self.Ntc = 40  # 训练单元数量
        self.Ngc = 10  # 保护单元数量
        self.Pfa = 1e-2  # 虚警概率
        self.rank = None  # CFAR排序参数
        self.alg = "SOCA"  # CFAR算法类型，默认为SOCA
        self.detector = None  # CFAR检测器对象
        self.threshold = 0  # 特征检测阈值

        # 点云生成默认参数
        self.colormap = "RdBu_r"  # 颜色映射，用于可视化
        self.pub_rect = True  # 是否发布矩形点云
        self.resolution = 0.5  # 点云降采样分辨率
        self.outlier_filter_radius = 1.0  # 离群点滤波半径
        self.outlier_filter_min_points = 5  # 离群点滤波最小邻居点数
        self.skip = 5  # 跳帧参数，控制处理帧的频率

        # 用于离线可视化的特征图像
        self.feature_img = None

        # 用于从极坐标到笛卡尔坐标的映射
        self.res = None  # 分辨率
        self.height = None  # 图像高度
        self.rows = None  # 图像行数
        self.width = None  # 图像宽度
        self.cols = None  # 图像列数
        self.map_x = None  # x坐标映射
        self.map_y = None  # y坐标映射
        self.f_bearings = None  # 角度插值函数
        self.to_rad = lambda bearing: bearing * np.pi / 18000  # 角度转弧度
        self.REVERSE_Z = 1  # z轴反转标志
        self.maxRange = None  # 最大范围

        # 是否使用压缩图像
        self.compressed_images = True

        # 多机器人系统占位符
        self.rov_id = ""

    def configure(self):
        '''配置CFAR检测器，初始化CFAR类
        '''
        self.detector = CFAR(self.Ntc, self.Ngc, self.Pfa, self.rank)

    def init_node(self, ns="~"):
        '''初始化ROS节点，读取参数并设置订阅与发布话题
        参数:
            ns: 命名空间，默认为"~"
        '''

        # 读取CFAR参数
        self.Ntc = rospy.get_param(ns + "CFAR/Ntc")  # 训练单元数量
        self.Ngc = rospy.get_param(ns + "CFAR/Ngc")  # 保护单元数量
        self.Pfa = rospy.get_param(ns + "CFAR/Pfa")  # 虚警概率
        self.rank = rospy.get_param(ns + "CFAR/rank")  # 排序参数
        self.alg = rospy.get_param(ns + "CFAR/alg", "SOCA")  # CFAR算法类型
        self.threshold = rospy.get_param(ns + "filter/threshold")  # 特征检测阈值

        # 读取点云降采样参数
        self.resolution = rospy.get_param(ns + "filter/resolution")  # 降采样分辨率
        self.outlier_filter_radius = rospy.get_param(ns + "filter/radius")  # 离群点滤波半径
        self.outlier_filter_min_points = rospy.get_param(ns + "filter/min_points")  # 离群点滤波最小邻居点数

        # 读取跳帧参数
        self.skip = rospy.get_param(ns + "filter/skip")  # 跳帧频率

        # 是否使用压缩图像
        self.compressed_images = rospy.get_param(ns + "compressed_images")

        # 初始化CV桥，用于图像转换
        self.BridgeInstance = cv_bridge.CvBridge()
        
        # 读取可视化坐标格式
        self.coordinates = rospy.get_param(ns + "visualization/coordinates", "cartesian")  # 坐标系，默认笛卡尔

        # 读取可视化参数
        self.radius = rospy.get_param(ns + "visualization/radius")  # 可视化半径
        self.color = rospy.get_param(ns + "visualization/color")  # 可视化颜色

        # 初始化声呐订阅者
        if self.compressed_images:
            self.sonar_sub = rospy.Subscriber(
                SONAR_TOPIC, OculusPing, self.callback, queue_size=10)  # 订阅压缩声呐图像
        else:
            self.sonar_sub = rospy.Subscriber(
                SONAR_TOPIC_UNCOMPRESSED, OculusPingUncompressed, self.callback, queue_size=10)  # 订阅未压缩声呐图像

        # 初始化特征发布者
        self.feature_pub = rospy.Publisher(
            SONAR_FEATURE_TOPIC, PointCloud2, queue_size=10)  # 发布点云特征

        # 初始化可视化图像发布者
        self.feature_img_pub = rospy.Publisher(
            SONAR_FEATURE_IMG_TOPIC, Image, queue_size=10)  # 发布可视化特征图像

        self.configure()  # 配置CFAR检测器

    def generate_map_xy(self, ping):
        '''生成声呐图像的网格映射，用于从极坐标转换为笛卡尔坐标
        参数:
            ping: OculusPing消息，包含声呐数据
        '''

        # 从ping消息中获取参数
        _res = ping.range_resolution  # 范围分辨率
        _height = ping.num_ranges * _res  # 图像高度
        _rows = ping.num_ranges  # 图像行数
        _width = np.sin(
            self.to_rad(ping.bearings[-1] - ping.bearings[0]) / 2) * _height * 2  # 图像宽度
        _cols = int(np.ceil(_width / _res))  # 图像列数

        # 检查参数是否发生变化
        if self.res == _res and self.height == _height and self.rows == _rows and self.width == _width and self.cols == _cols:
            return  # 参数未变，直接返回

        # 更新参数
        self.res, self.height, self.rows, self.width, self.cols = _res, _height, _rows, _width, _cols

        # 生成角度插值
        bearings = self.to_rad(np.asarray(ping.bearings, dtype=np.float32))  # 转换为弧度
        f_bearings = interp1d(
            bearings,
            range(len(bearings)),
            kind='linear',
            bounds_error=False,
            fill_value=-1,
            assume_sorted=True)  # 角度插值函数

        # 构建网格
        XX, YY = np.meshgrid(range(self.cols), range(self.rows))  # 生成网格坐标
        x = self.res * (self.rows - YY)  # x坐标（笛卡尔）
        y = self.res * (-self.cols / 2.0 + XX + 0.5)  # y坐标（笛卡尔）
        b = np.arctan2(y, x) * self.REVERSE_Z  # 计算角度
        r = np.sqrt(np.square(x) + np.square(y))  # 计算距离
        self.map_y = np.asarray(r / self.res, dtype=np.float32)  # 距离映射
        self.map_x = np.asarray(f_bearings(b), dtype=np.float32)  # 角度映射

    def publish_features(self, ping, points):
        '''发布特征点云消息
        参数:
            ping: OculusPing消息，包含时间戳等信息
            points: 笛卡尔坐标系中的点云数据（单位：米）
        '''

        # 添加z轴（设为0）
        points = np.c_[points[:,0], np.zeros(len(points)), points[:,1]]

        # 转换为ROS点云格式
        feature_msg = n2r(points, "PointCloudXYZ")

        # 使用声呐图像的时间戳，确保时间同步
        feature_msg.header.stamp = ping.header.stamp
        feature_msg.header.frame_id = "base_link"  # 参考坐标系

        # 发布点云，供SLAM使用
        self.feature_pub.publish(feature_msg)

    # @add_lock
    def callback(self, sonar_msg):
        '''特征提取回调函数，处理声呐消息
        参数:
            sonar_msg: OculusPing消息，极坐标下的声呐数据
        '''

        # 跳帧处理，减少计算量
        if sonar_msg.ping_id % self.skip != 0:
            self.feature_img = None
            # 发布空点云以保持SLAM同步
            nan = np.array([[np.nan, np.nan]])
            self.publish_features(sonar_msg, nan)
            return

        # 解码压缩图像
        if self.compressed_images:
            img = np.frombuffer(sonar_msg.ping.data, np.uint8)  # 从缓冲区读取
            img = np.array(cv2.imdecode(img, cv2.IMREAD_COLOR)).astype(np.uint8)  # 解码为彩色图像
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)  # 转换为灰度图
        else:
            # 未压缩图像直接使用ros_numpy转换
            img = ros_numpy.image.image_to_numpy(sonar_msg.ping)

        # 生成极坐标到笛卡尔坐标的映射
        self.generate_map_xy(sonar_msg)

        # 使用CFAR检测目标并应用阈值（在极坐标下）
        peaks = self.detector.detect(img, self.alg)
        peaks &= img > self.threshold  # 应用强度阈值

        # 将图像从极坐标转换为笛卡尔坐标并可视化
        vis_img = cv2.remap(img, self.map_x, self.map_y, cv2.INTER_LINEAR)  # 坐标转换
        vis_img = cv2.applyColorMap(vis_img, 2)  # 应用颜色映射
        self.feature_img_pub.publish(ros_numpy.image.numpy_to_image(vis_img, "bgr8"))  # 发布可视化图像

        # 将特征点转换为笛卡尔坐标
        peaks = cv2.remap(peaks, self.map_x, self.map_y, cv2.INTER_LINEAR)        
        locs = np.c_[np.nonzero(peaks)]  # 获取特征点位置

        # 从图像坐标转换为米
        x = locs[:,1] - self.cols / 2.  # x坐标偏移
        x = (-1 * ((x / float(self.cols / 2.)) * (self.width / 2.)))  # 转换为米
        y = (-1*(locs[:,0] / float(self.rows)) * self.height) + self.height  # 转换为米
        points = np.column_stack((y, x))  # 组成点云

        # 使用PCL进行点云降采样
        if len(points) and self.resolution > 0:
            points = pcl.downsample(points, self.resolution)

        # 移除离群点
        if self.outlier_filter_min_points > 1 and len(points) > 0:
            points = pcl.remove_outlier(
                points, self.outlier_filter_radius, self.outlier_filter_min_points
            )

        # 发布特征点云
        self.publish_features(sonar_msg, points)