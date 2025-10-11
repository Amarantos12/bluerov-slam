import os
import cv2  # 导入 OpenCV 库，用于图像处理
import pickle 
import json 
import math
from scipy.io import savemat  # 导入 scipy.io 用于保存 MATLAB 文件

def load_data(target):
    """
    加载数据集，从指定目录读取 .pkl 文件和配置文件，提取声呐图像和传感器位姿等信息。

    参数:
        target (str): 数据集名称或路径，用于定位数据目录

    返回:
        dict: 包含声呐图像、传感器位姿和声呐参数的字典
    """
    # 构建数据目录路径
    dirpath = "./data/{}".format(target)  # 数据集根目录
    pickle_loc = "{}/Data".format(dirpath)  # .pkl 文件存放目录
    output_loc = "{}/UnzipData".format(dirpath)  # 解压数据输出目录
    cfg_path = "{}/Config.json".format(dirpath)  # 配置文件路径

    # 读取配置文件 Config.json
    with open(cfg_path, 'r') as f:
        cfg = json.load(f)  # 解析 JSON 格式的配置文件

    # 遍历配置文件中的传感器，提取声呐传感器的参数
    for agents in cfg["agents"][0]["sensors"]:
        if agents["sensor_type"] != "ImagingSonar": 
            continue  # 跳过非声呐传感器
        # 提取声呐参数
        hfov = agents["configuration"]["Azimuth"]  # 水平视场（度）
        vfov = agents["configuration"]["Elevation"]  # 垂直视场（度）
        min_range = agents["configuration"]["RangeMin"]  # 最小检测范围（米）
        max_range = agents["configuration"]["RangeMax"]  # 最大检测范围（米）
        hfov = math.radians(hfov)  # 将水平视场转换为弧度
        vfov = math.radians(vfov)  # 将垂直视场转换为弧度

    # 如果输出目录不存在，则创建
    if not os.path.exists(output_loc):
        os.makedirs(output_loc)

    # 初始化存储声呐图像和传感器位姿的列表
    images = []  # 存储声呐图像
    sensor_poses = []  # 存储传感器位姿

    # 遍历 .pkl 文件目录，加载数据
    for pkls in os.listdir(pickle_loc):
        filename = "{}/{}".format(pickle_loc, pkls)  # 构造 .pkl 文件完整路径
        with open(filename, 'rb') as f:
            state = pickle.load(f)  # 加载 .pkl 文件内容
            image = state["ImagingSonar"]  # 提取声呐图像数据
            s = image.shape  # 获取图像形状 (H, W)
            # image[image < 0.2] = 0  # 将强度小于 0.2 的像素置零（去噪）
            # image[s[0] - 200:, :] = 0  # 将图像底部 200 行置零（可能去除无效区域）
            pose = state["PoseSensor"]  # 提取传感器位姿
            images.append(image)  # 添加处理后的图像到列表
            sensor_poses.append(pose)  # 添加位姿到列表

    # 构建返回的字典，包含所有数据
    data = {
        "images": images,  # 声呐图像列表
        "images_no_noise": [],  # 空列表，可能用于存储无噪图像（未使用）
        "sensor_poses": sensor_poses,  # 传感器位姿列表
        "min_range": min_range,  # 声呐最小范围
        "max_range": max_range,  # 声呐最大范围
        "hfov": hfov,  # 水平视场（弧度）
        "vfov": vfov  # 垂直视场（弧度）
    }
    
    # 将数据保存为 MATLAB 文件
    savemat('{}/{}.mat'.format(dirpath, target), data, oned_as='row')
    
    # 返回数据字典
    return data