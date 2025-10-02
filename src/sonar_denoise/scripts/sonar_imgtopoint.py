#!/usr/bin/env python

import rospy
import cv2
import cv_bridge
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from sonar_oculus.msg import OculusPingUncompressed, OculusFire

# Global counter for ping_id
ping_id_counter = 0

def cartesian_to_polar(image, max_range, bearing_range, num_beams, bearing_resolution, origin, reverse_z=1):
    """
    Convert Cartesian sonar image to polar coordinates for BlueView P900-130 sonar.
    
    Parameters:
    - image: Input Cartesian image (NumPy array, grayscale)
    - max_range: Maximum detection range (meters)
    - bearing_range: Angle range (radians, e.g., [-1.1345, 1.1345])
    - num_beams: Total number of beams (width of polar image)
    - bearing_resolution: Angular resolution (radians/pixel)
    - origin: Sonar origin in Cartesian image (x, y), e.g., (776.5, 848)
    - reverse_z: Angle direction (1 or -1, -1 to flip)
    
    Returns:
    - polar_image: Polar coordinate image (num_beams × height, uint8)
    """
    # Get image dimensions
    height, width = image.shape[:2]
    rospy.loginfo(f"Cartesian image size: {height}x{width}, origin: {origin}")
    
    if np.max(image) == 0:
        rospy.logwarn("Input Cartesian image is all zeros, may be empty or invalid")
    
    # Polar image dimensions
    polar_width = num_beams
    polar_height = height
    rospy.loginfo(f"Target polar image size: {polar_width}x{polar_height}")
    
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
    
    rospy.loginfo(f"Actual polar image size: {polar_image.shape[1]}x{polar_image.shape[0]}")
    
    return polar_image

def image_callback(msg):
    global ping_id_counter
    try:
        # Initialize CvBridge
        bridge = cv_bridge.CvBridge()
        
        # Convert ROS image message to OpenCV image (assume grayscale)
        image = bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
        
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
        
        # Display the polar image
        cv2.imshow("Cartesian view", image)
        cv2.imshow("Polar View", polar_image)
        cv2.waitKey(1)
        rospy.loginfo("Displaying polar image, size: %dx%d",
                      polar_image.shape[1], polar_image.shape[0])
        
        # Now, publish as OculusPingUncompressed
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
        ping_id_counter += 1
        
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
        
        # Publish
        pub.publish(oculus_msg)
        rospy.loginfo("Published OculusPingUncompressed message with ping_id: %d", oculus_msg.ping_id)
        
    except cv_bridge.CvBridgeError as e:
        rospy.logerr("CvBridge Error: %s", e)
    except Exception as e:
        rospy.logerr("Error in callback: %s", str(e))

def main():
    global pub
    rospy.init_node('oculus_publisher_node', anonymous=True)
    
    # Publisher for OculusPingUncompressed
    pub = rospy.Publisher('/sonar_oculus_node/ping', OculusPingUncompressed, queue_size=10)
    
    # Subscriber to /son
    rospy.Subscriber('/son', Image, image_callback, queue_size=15)
    rospy.loginfo("Subscribed to /son topic and publishing to /sonar_oculus_node/ping")
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
    finally:
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()