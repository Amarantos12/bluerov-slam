# 接收并发布话题，odom与sonar_oculus/ping话题，并rosbag record生成新的rosbag

import rosbag
import cv_bridge
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage, Image
from geometry_msgs.msg import TwistStamped, PoseStamped, PoseWithCovarianceStamped
from sonar_oculus.msg import OculusPingUncompressed, OculusFire
from std_msgs.msg import Header
import tf.transformations
import math
import rospy  # For rospy.Time, but not initializing node
from tqdm import tqdm  # For progress bar

class Odometry:
    """Python implementation of the Odometry class from odometry.h/cpp."""
    def __init__(self):
        self.p_x = 0.0  # Point2d.x (current position x)
        self.p_y = 0.0  # Point2d.y (current position y)
        self.theta = 0.0  # Current yaw
        self.v_x = 0.0  # Point2d.x (current velocity x)
        self.v_y = 0.0  # Point2d.y (current velocity y)
        self.w_theta = 0.0  # Current angular velocity
        self.time = 0.0  # Last update time
        self.is_initialized = False

    def set_initial_pose(self, time, px, py, theta):
        """Set initial pose (px, py, theta)."""
        self.p_x = px
        self.p_y = py
        self.theta = theta
        self.time = time
        self.is_initialized = True

    def update_velocity(self, time, vx, vy, w_theta):
        """Update pose based on velocity and time difference."""
        dt = time - self.time

        if dt < 0.0:
            print("Invalid time: dt = %f" % dt)
            return

        dx = self.v_x * dt
        dy = self.v_y * dt
        d_yaw = w_theta * dt

        self.theta -= d_yaw  # Match C++ behavior (m_theta -= dYaw)

        # Convert body velocity to global velocity
        co = math.cos(self.theta)
        so = math.sin(self.theta)
        self.p_x += dx * so + dy * co
        self.p_y += dx * co - dy * so

        # Save current velocity and time
        self.v_x = vx
        self.v_y = vy
        self.w_theta = w_theta
        self.time = time
        self.is_initialized = True

    def predict(self, time):
        """Predict pose at given time, return (x, y, theta) or None if invalid."""
        dt = time - self.time

        if dt < 0.0:
            print("Invalid time: dt = %f" % dt)
            return None

        if dt == 0.0:
            return self.p_x, self.p_y, self.theta

        dx = self.v_x * dt
        dy = self.v_y * dt
        d_yaw = self.w_theta * dt

        # Convert body velocity to global velocity
        co = math.cos(self.theta)
        so = math.sin(self.theta)
        x = self.p_x + dx * co - dy * so
        y = self.p_y + dx * so + dy * co
        theta = self.theta + d_yaw

        # Normalize theta to [0, 2pi]
        while theta > 2 * math.pi:
            theta -= 2 * math.pi
        while theta < 0.0:
            theta += 2 * math.pi

        return x, y, theta

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

# Main processing
input_bag_path = '/home/hzr/rosbag/ARACATI_2017_8bits_full.bag'  # Input bag file
output_bag_path = '/home/hzr/rosbag/processed_aracati_full.bag'  # Output bag file

# Initialize variables
odom = Odometry()
ping_id_counter = 0
odom_frame_id = 'odom'  # From your code
world_frame_id = 'map'  # From your code
base_frame_id = 'son'   # From your code
initial_pose_set = False

# Count total messages for progress bar
input_bag = rosbag.Bag(input_bag_path)
total_messages = input_bag.get_message_count()
input_bag.close()

# Process bag with progress bar
with rosbag.Bag(output_bag_path, 'w') as outbag:
    with tqdm(total=total_messages, desc="Processing ROS bag", unit="msg") as pbar:
        for topic, msg, t in rosbag.Bag(input_bag_path).read_messages():
            
            # Write /pose_gt to output if it's the topic
            if topic == '/pose_gt':
                outbag.write(topic, msg, t)
                
                # Set initial pose if not set
                if not initial_pose_set:
                    quat = (msg.pose.orientation.x, msg.pose.orientation.y,
                            msg.pose.orientation.z, msg.pose.orientation.w)
                    yaw = tf.transformations.euler_from_quaternion(quat)[2]
                    time_sec = msg.header.stamp.to_sec()
                    odom.set_initial_pose(time_sec, msg.pose.position.x, msg.pose.position.y, yaw)
                    
                    # Publish initial pose (but write to bag instead)
                    ini_pose_msg = PoseWithCovarianceStamped()
                    ini_pose_msg.header.stamp = msg.header.stamp
                    ini_pose_msg.header.frame_id = world_frame_id
                    ini_pose_msg.pose.pose.position.x = msg.pose.position.x
                    ini_pose_msg.pose.pose.position.y = msg.pose.position.y
                    ini_pose_msg.pose.pose.position.z = 0.0
                    ini_pose_msg.pose.pose.orientation = msg.pose.orientation
                    ini_pose_msg.pose.covariance = [0.0] * 36
                    ini_pose_msg.pose.covariance[0] = 0.5 * 0.5  # CovX
                    ini_pose_msg.pose.covariance[7] = 0.5 * 0.5  # CovY
                    ini_pose_msg.pose.covariance[21] = 0.4 * 0.4  # CovYaw
                    outbag.write('/initialpose', ini_pose_msg, t)  # If you want to include it, but not required
                    initial_pose_set = True
            
            # Process /cmd_vel to generate /odom_pose
            elif topic == '/cmd_vel':
                if odom.is_initialized:
                    time_sec = msg.header.stamp.to_sec()
                    odom.update_velocity(time_sec, msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z)
                    
                    result = odom.predict(time_sec)
                    if result is not None:
                        odom_x, odom_y, odom_yaw = result
                        
                        # Create PoseStamped for /odom_pose
                        pose_msg = PoseStamped()
                        pose_msg.header.stamp = msg.header.stamp
                        pose_msg.header.frame_id = odom_frame_id
                        pose_msg.pose.position.x = odom_x
                        pose_msg.pose.position.y = odom_y
                        pose_msg.pose.position.z = 0.0
                        quat = tf.transformations.quaternion_from_euler(0.0, 0.0, odom_yaw)
                        pose_msg.pose.orientation.x = quat[0]
                        pose_msg.pose.orientation.y = quat[1]
                        pose_msg.pose.orientation.z = quat[2]
                        pose_msg.pose.orientation.w = quat[3]
                        
                        # Write to output bag
                        outbag.write('/odom_pose', pose_msg, t)
            
            # Process /son/compressed and write original + new /sonar_oculus_node/ping
            elif topic == '/son/compressed':
                # Write original message
                outbag.write(topic, msg, t)
                
                # Process to generate OculusPingUncompressed
                oculus_msg, ping_id_counter = process_son_compressed(msg, ping_id_counter)
                if oculus_msg is not None:
                    outbag.write('/sonar_oculus_node/ping', oculus_msg, t)
            
            # Update progress bar
            pbar.update(1)

print("Processing complete. Output bag saved to: %s" % output_bag_path)