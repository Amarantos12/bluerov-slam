# 把文件夹中的笛卡尔坐标系图像转换为极坐标系图像


import numpy as np
import cv2
import os

def cartesian_to_polar(image, max_range, bearing_range, num_beams, bearing_resolution, origin, reverse_z=1):
    """
    将笛卡尔坐标系图像转换为极坐标系图像，适配 BlueView P900-130 声纳。
    
    参数：
    - image: 输入的笛卡尔图像（NumPy 数组，灰度或彩色）
    - max_range: 最大探测距离（米）
    - bearing_range: 角度范围（弧度，例如 [-1.1345, 1.1345]）
    - num_beams: 总波束数（对应极坐标图像的宽度）
    - bearing_resolution: 角度分辨率（弧度/像素）
    - origin: 声纳原点在笛卡尔图像中的位置 (x, y)，例如 (776.5, 848)
    - reverse_z: 角度方向（1 或 -1，-1 表示翻转）
    
    返回：
    - polar_image: 极坐标系图像（num_beams×height，宽度×高度）
    """
    # 获取图像尺寸
    height, width = image.shape[:2]
    print(f"笛卡尔图像尺寸：{height}x{width}, 原点位置：{origin}")
    
    # 检查图像内容
    if np.max(image) == 0:
        print("警告：输入笛卡尔图像全为零，可能为空或内容错误")
    
    # 极坐标图像尺寸
    polar_width = num_beams  # 角度维度
    polar_height = height  # 距离维度，基于笛卡尔图像高度
    print(f"极坐标图像目标尺寸：{polar_width}x{polar_height}")
    
    # 计算距离分辨率
    range_resolution = max_range / polar_height  # 米/像素
    
    # 生成极坐标网格（x 对应角度，y 对应距离）
    x = np.arange(polar_width)  # 0 到 num_beams-1
    y = np.arange(polar_height)  # 0 到 height-1
    X, Y = np.meshgrid(x, y)
    
    # 计算极坐标 (r, θ)
    r = Y * range_resolution  # 距离：y * (max_range/height)
    theta = bearing_range[0] + X * bearing_resolution * reverse_z  # 角度：-65° + x * (130/num_beams)°
    
    # 映射到笛卡尔坐标（声纳原点，单位为米）
    x1 = r * np.sin(theta)  # x1 = r * sin(θ)
    y1 = r * np.cos(theta)  # y1 = r * cos(θ)
    
    # 转换为 OpenCV 图像坐标（原点在左上角，单位为像素）
    map_x = x1 / range_resolution + origin[0]  # x_image = x1/range_resolution + origin_x
    map_y = origin[1] - y1 / range_resolution  # y_image = origin_y - y1/range_resolution
    
    # 确保 map_x, map_y 在图像范围内
    map_x = np.clip(map_x, 0, width - 1).astype(np.float32)
    map_y = np.clip(map_y, 0, height - 1).astype(np.float32)
    
    # 使用 cv2.remap 进行重映射
    polar_image = cv2.remap(image, map_x, map_y, cv2.INTER_CUBIC, borderValue=0)
    
    # 验证输出尺寸
    print(f"实际输出极坐标图像尺寸：{polar_image.shape[1]}x{polar_image.shape[0]}")
    
    return polar_image

if __name__ == "__main__":
    # 声纳参数（基于 BlueView P900-130）
    # max_range = 50.0  # 最大探测距离（米）
    # bearing_range = [-65 * np.pi / 180, 65 * np.pi / 180]  # 角度范围（弧度）
    # num_beams = 768  # 总波束数
    # bearing_resolution = (130 / 768) * np.pi / 180  # 角度分辨率（130/768°）
    # cartesian_shape = (848, 1553)  # 笛卡尔图像尺寸（高度×宽度）
    # origin = (1553 / 2, 848)  # 声纳原点（776.5, 848）

    # 基于 BlueView m450d
    max_range = 10.0  # 最大探测距离（米）
    bearing_range = [-0.773598775, 0.773598775]  # 角度范围（弧度）
    num_beams = 768  # 总波束数
    bearing_resolution = 1.54719755 / num_beams  # 角度分辨率
    cartesian_shape = (399, 512)  # 笛卡尔图像尺寸（高度×宽度）
    origin = (256, 399)  # 声纳原点（776.5, 848）

    # 输入和输出目录
    input_dir = "/home/hzr/rosbag/aurora_dikaer"
    output_dir = "/home/hzr/rosbag/aurora_polar"

    # 确保输出目录存在
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        print(f"创建输出目录：{output_dir}")

    # 支持的图像扩展名
    valid_extensions = ('.png', '.jpg', '.jpeg', '.bmp', '.tiff')

    # 遍历输入目录中的所有图像
    for filename in os.listdir(input_dir):
        if filename.lower().endswith(valid_extensions):
            input_path = os.path.join(input_dir, filename)
            output_path = os.path.join(output_dir, filename)

            # 读取笛卡尔图像
            cartesian_image = cv2.imread(input_path, cv2.IMREAD_GRAYSCALE)  # 假设为灰度图像
            if cartesian_image is None:
                print(f"无法读取图像：{input_path}")
                continue

            # 获取图像尺寸
            height, width = cartesian_image.shape[:2]
            print(f"\n处理图像：{filename}")
            print(f"图像尺寸：{height}x{width}")

            # 验证图像尺寸是否符合预期
            if (height, width) != cartesian_shape:
                print(f"警告：图像 {filename} 的尺寸 {height}x{width} 与预期尺寸 {cartesian_shape} 不符，跳过")
                continue

            # 转换为极坐标图像
            polar_image = cartesian_to_polar(
                cartesian_image, max_range, bearing_range, num_beams, bearing_resolution, origin, reverse_z=1
            )

            # 保存极坐标图像
            cv2.imwrite(output_path, polar_image)
            print(f"极坐标图像已保存到：{output_path}")