# 生成rosbag
# 包含话题 /sonar_oculus_node/ping，/son/compressed，/rti/body_velocity/raw，/bar30/depth/raw，/vn100/imu/raw

import rosbag
import cv_bridge
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage, Image, Imu
from sonar_oculus.msg import OculusPingUncompressed, OculusFire
from rti_dvl.msg import DVL
from bar30_depth.msg import Depth
from geometry_msgs.msg import TwistStamped, PoseStamped, Vector3, Quaternion
from std_msgs.msg import Header
import tf.transformations as tf_trans
import math
from tqdm import tqdm  # For progress bar
from collections import defaultdict

# Main processing
input_bag_path = '/home/hzr/rosbag/ARACATI_2017_8bits_full.bag'  # Input bag file
output_bag_path = '/home/hzr/rosbag/ARACATI_2017_8bits_full_slam.bag'  # Output bag file


def cartesian_to_polar(image, max_range, bearing_range, num_beams, bearing_resolution, origin, reverse_z=1):
    """
    Convert Cartesian sonar image to polar coordinates for BlueView P900-130 sonar.
    """
    # Get image dimensions
    height, width = image.shape[:2]
    
    # Polar image dimensions
    polar_width = num_beams
    polar_height = height
    
    # Calculate range resolution
    range_resolution = max_range / polar_height
    
    # Generate polar coordinate grid
    x = np.arange(polar_width)
    y = np.arange(polar_height)
    X, Y = np.meshgrid(x, y)
    
    # Compute polar coordinates (r, θ)
    r = Y * range_resolution
    theta = bearing_range[0] + X * bearing_resolution * reverse_z
    
    # Map to Cartesian coordinates (sonar origin, in meters)
    x1 = r * np.sin(theta)
    y1 = r * np.cos(theta)
    
    # Convert to OpenCV image coordinates
    map_x = x1 / range_resolution + origin[0]
    map_y = origin[1] - y1 / range_resolution
    
    # Clip coordinates to image bounds
    map_x = np.clip(map_x, 0, width - 1).astype(np.float32)
    map_y = np.clip(map_y, 0, height - 1).astype(np.float32)
    
    # Remap to create polar image
    polar_image = cv2.remap(image, map_x, map_y, cv2.INTER_CUBIC, borderValue=0)
    
    # Ensure uint8
    if polar_image.dtype != np.uint8:
        polar_image = np.clip(polar_image, 0, 255).astype(np.uint8)
    
    return polar_image

def process_son_compressed(msg, ping_id_counter):
    try:
        # Initialize CvBridge
        bridge = cv_bridge.CvBridge()
        
        # Convert ROS compressed image message to OpenCV image (default encoding)
        image = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")  # Assume RGB/BGR input
        
        # Convert to grayscale if necessary
        if len(image.shape) == 3:  # Check if image is multi-channel (e.g., RGB)
            image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)  # Convert to grayscale
        
        # BlueView P900-130 sonar parameters
        max_range = 50.0
        bearing_range_deg = [-65.0, 65.0]
        bearing_range = [deg * np.pi / 180 for deg in bearing_range_deg]
        num_beams = 768
        bearing_resolution_deg = 130.0 / num_beams
        bearing_resolution = bearing_resolution_deg * np.pi / 180
        origin = (776.5, 848)
        
        # Convert to polar coordinates
        polar_image = cartesian_to_polar(
            image, max_range, bearing_range, num_beams, bearing_resolution, origin, reverse_z=1
        )
        
        # Create OculusPingUncompressed message
        oculus_msg = OculusPingUncompressed()
        
        # Header
        oculus_msg.header = msg.header
        
        # Fire msg
        oculus_msg.fire_msg = OculusFire()
        oculus_msg.fire_msg.header = msg.header
        oculus_msg.fire_msg.mode = 1  # Low frequency mode
        oculus_msg.fire_msg.gamma = 128
        oculus_msg.fire_msg.flags = 0
        oculus_msg.fire_msg.range = max_range
        oculus_msg.fire_msg.gain = 0.0
        oculus_msg.fire_msg.speed_of_sound = 1500.0
        oculus_msg.fire_msg.salinity = 35.0
        
        # Ping id
        oculus_msg.ping_id = ping_id_counter
        
        # Part number (using Oculus M750d as proxy)
        oculus_msg.part_number = 1032
        
        # Start time
        oculus_msg.start_time = msg.header.stamp.secs
        
        # Bearings (int16, centi-degrees)
        bearings_deg = np.linspace(bearing_range_deg[0], bearing_range_deg[1], num_beams)
        oculus_msg.bearings = (bearings_deg * 100).astype(np.int16).tolist()
        
        # Range resolution
        oculus_msg.range_resolution = max_range / polar_image.shape[0]
        
        # Num ranges and beams
        oculus_msg.num_ranges = polar_image.shape[0]
        oculus_msg.num_beams = polar_image.shape[1]
        
        # Ping data (sensor_msgs/Image)
        oculus_msg.ping = Image()
        oculus_msg.ping.header = msg.header
        oculus_msg.ping.height = polar_image.shape[0]
        oculus_msg.ping.width = polar_image.shape[1]
        oculus_msg.ping.encoding = "mono8"
        oculus_msg.ping.is_bigendian = 0
        oculus_msg.ping.step = polar_image.shape[1]  # bytes per row (mono8: 1 byte per pixel)
        oculus_msg.ping.data = polar_image.tobytes()
        
        return oculus_msg, ping_id_counter + 1
    
    except cv_bridge.CvBridgeError as e:
        print("CvBridge Error: %s" % e)
        return None, ping_id_counter
    except Exception as e:
        print("Error in processing: %s" % str(e))
        return None, ping_id_counter

# Initialize variables
ping_id_counter = 0
pose_gt_dict = {}  # Dictionary to store /pose_gt messages by timestamp (sec)

# First pass: Collect all /pose_gt messages
with rosbag.Bag(input_bag_path) as input_bag:
    for topic, msg, t in input_bag.read_messages(topics=['/pose_gt']):
        time_sec = msg.header.stamp.to_sec()
        pose_gt_dict[time_sec] = msg

# Count total messages for progress bar
total_messages = rosbag.Bag(input_bag_path).get_message_count()

# Process bag with progress bar
with rosbag.Bag(output_bag_path, 'w') as outbag:
    with tqdm(total=total_messages, desc="Processing ROS bag", unit="msg") as pbar:
        for topic, msg, t in rosbag.Bag(input_bag_path).read_messages():
            
            # Process /son/compressed and write original + new /sonar_oculus_node/ping
            if topic == '/son/compressed':
                # Write original message
                outbag.write(topic, msg, t)
                
                # Process to generate OculusPingUncompressed
                oculus_msg, ping_id_counter = process_son_compressed(msg, ping_id_counter)
                if oculus_msg is not None:
                    outbag.write('/sonar_oculus_node/ping', oculus_msg, t)
            
            # Process /cmd_vel to generate /rti/body_velocity/raw, /bar30/depth/raw, /vn100/imu/raw
            elif topic == '/cmd_vel':
                cmd_vel_time_sec = msg.header.stamp.to_sec()
                
                # Find the closest /pose_gt by time
                closest_pose_gt = None
                min_time_diff = float('inf')
                for pose_time_sec in pose_gt_dict:
                    time_diff = abs(cmd_vel_time_sec - pose_time_sec)
                    if time_diff < min_time_diff and time_diff < 0.1:  # Threshold 0.1 sec
                        min_time_diff = time_diff
                        closest_pose_gt = pose_gt_dict[pose_time_sec]
                
                if closest_pose_gt is not None:
                    # Unified header using /cmd_vel stamp
                    header = Header()
                    header.stamp = msg.header.stamp
                    header.frame_id = 'base_link'  # Assumed frame_id
                    
                    # Generate and write DVL (/rti/body_velocity/raw)
                    dvl_msg = DVL()
                    dvl_msg.header = header
                    dvl_msg.velocity = Vector3(
                        x=msg.twist.linear.x,
                        y=msg.twist.linear.y,
                        z=0.0  # 2D motion, z velocity = 0
                    )
                    dvl_msg.temperature = 0.0  # Default
                    dvl_msg.altitude = 0.0  # Default
                    dvl_msg.time = header.stamp.to_sec()
                    outbag.write('/rti/body_velocity/raw', dvl_msg, t)
                    
                    # Generate and write Depth (/bar30/depth/raw)
                    depth_msg = Depth()
                    depth_msg.header = header
                    depth_msg.time = header.stamp.to_sec()
                    depth_msg.pressure_abs = 0.0  # Default
                    depth_msg.pressure_diff = 0.0  # Default
                    depth_msg.temperature = 0.0  # Default
                    depth_msg.depth = 0.0  # 2D motion, depth = 0
                    outbag.write('/bar30/depth/raw', depth_msg, t)
                    
                    # Generate and write Imu (/vn100/imu/raw)
                    imu_msg = Imu()
                    imu_msg.header = header
                    
                    # Orientation: From closest /pose_gt, force roll=0, pitch=0, keep yaw
                    euler = tf_trans.euler_from_quaternion([
                        closest_pose_gt.pose.orientation.x,
                        closest_pose_gt.pose.orientation.y,
                        closest_pose_gt.pose.orientation.z,
                        closest_pose_gt.pose.orientation.w
                    ])
                    quat = tf_trans.quaternion_from_euler(0.0, 0.0, euler[2])
                    imu_msg.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
                    imu_msg.orientation_covariance = [0.0] * 9  # Default
                    
                    # Angular velocity: From /cmd_vel, only z (yaw rate), x/y=0
                    imu_msg.angular_velocity = Vector3(x=0.0, y=0.0, z=msg.twist.angular.z)
                    imu_msg.angular_velocity_covariance = [0.0] * 9  # Default
                    
                    # Linear acceleration: All 0 (no data)
                    imu_msg.linear_acceleration = Vector3(x=0.0, y=0.0, z=0.0)
                    imu_msg.linear_acceleration_covariance = [0.0] * 9  # Default
                    
                    outbag.write('/vn100/imu/raw', imu_msg, t)
            
            # Update progress bar
            pbar.update(1)

print("Processing complete. Output bag saved to: %s" % output_bag_path)


