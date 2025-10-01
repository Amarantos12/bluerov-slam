#!/usr/bin/env python

import rospy
import cv2
import cv_bridge
import numpy as np
from sensor_msgs.msg import Image

def cartesian_to_polar(image, max_range, bearing_range, num_beams, bearing_resolution, origin, reverse_z=1):
    """
    Convert Cartesian sonar image to polar coordinates for BlueView P900-130 sonar.
    
    Parameters:
    - image: Input Cartesian image (NumPy array, grayscale)
    - max_range: Maximum detection range (meters)
    - bearing_range: Angle range (r tells, e.g., [-1.1345, 1.1345])
    - num_beams: Total number of beams (width of polar image)
    - bearing_resolution: Angular resolution (radians/pixel)
    - origin: Sonar origin in Cartesian image (x, y), e.g., (776.5, 848)
    - reverse_z: Angle direction (1 or -1, -1 to flip)
    
    Returns:
    - polar_image: Polar coordinate image (num_beams × height)
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
    
    rospy.loginfo(f"Actual polar image size: {polar_image.shape[1]}x{polar_image.shape[0]}")
    
    return polar_image

def image_callback(msg):
    try:
        # Initialize CvBridge
        bridge = cv_bridge.CvBridge()
        
        # Convert ROS image message to OpenCV image
        image = bridge.imgmsg_to_cv2(msg, desired_encoding=msg.encoding)
        
        # BlueView P900-130 sonar parameters
        max_range = 50.0  # Maximum range in meters
        bearing_range = [-65 * np.pi / 180, 65 * np.pi / 180]  # Angle range in radians
        num_beams = 768  # Number of beams
        bearing_resolution = (130 / num_beams) * np.pi / 180  # Angular resolution
        origin = (776.5, 848)  # Sonar origin (x, y)
        
        # Convert to polar coordinates
        polar_image = cartesian_to_polar(
            image, max_range, bearing_range, num_beams, bearing_resolution, origin, reverse_z=1
        )
        
        # Display the polar image
        cv2.imshow("Cartesian view", image)
        cv2.imshow("Polar View", polar_image)
        cv2.waitKey(1)
        rospy.loginfo("Displaying polar image, size: %dx%d, channels: %d",
                      polar_image.shape[1], polar_image.shape[0],
                      polar_image.shape[2] if len(polar_image.shape) > 2 else 1)
        
    except cv_bridge.CvBridgeError as e:
        rospy.logerr("CvBridge Error: %s", e)

def main():
    rospy.init_node('image_sub_node', anonymous=True)
    rospy.Subscriber('/son', Image, image_callback, queue_size=15)
    rospy.loginfo("Subscribed to /son topic")
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
    finally:
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
    