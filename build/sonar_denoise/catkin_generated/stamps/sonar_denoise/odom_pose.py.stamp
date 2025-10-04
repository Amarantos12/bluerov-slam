#!/usr/bin/env python

import rospy
import tf2_ros
import tf.transformations  # Use tf.transformations instead of tf2_ros.transformations
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, TwistStamped, TransformStamped
import math

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
            rospy.logwarn("Invalid time: dt = %f", dt)
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
            rospy.logwarn("Invalid time: dt = %f", dt)
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

    def is_initialized(self):
        """Check if odometry is initialized."""
        return self.is_initialized

class Odom:
    """Python implementation of the Odom class from odom.h/cpp."""
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('odom', anonymous=True)

        # Initialize parameters
        self.world_frame_id = rospy.get_param('~world_frame_id', 'map')
        self.odom_frame_id = rospy.get_param('~odom_frame_id', 'odom')
        self.base_frame_id = rospy.get_param('~base_frame_id', 'son')

        # Initialize odometry
        self.odom = Odometry()

        # Publishers
        self.pub_odom = rospy.Publisher('/odom_pose', PoseStamped, queue_size=1)
        self.pub_ini_pose = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # Subscribers
        self.sub_cmd_vel = rospy.Subscriber('/cmd_vel', TwistStamped, self.cmd_vel_callback)

        # Initialize with /pose_gt
        self.init_ros()

    def publish_initial_pose(self, time, x, y, yaw):
        """Publish initial pose to /initialpose topic."""
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = time
        msg.header.frame_id = self.world_frame_id

        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0

        # Convert yaw to quaternion
        quat = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)
        msg.pose.pose.orientation.x = quat[0]
        msg.pose.pose.orientation.y = quat[1]
        msg.pose.pose.orientation.z = quat[2]
        msg.pose.pose.orientation.w = quat[3]

        # Set covariance (same as C++ code)
        msg.pose.covariance = [0.0] * 36
        msg.pose.covariance[0] = 0.5 * 0.5  # CovX
        msg.pose.covariance[7] = 0.5 * 0.5  # CovY
        msg.pose.covariance[21] = 0.4 * 0.4  # CovYaw

        self.pub_ini_pose.publish(msg)
        rospy.loginfo("Published initial pose: x=%f, y=%f, yaw=%f", x, y, yaw)

    def cmd_vel_callback(self, msg):
        """Callback for /cmd_vel topic to update and publish odometry."""
        time = msg.header.stamp.to_sec()

        # Update odometry with velocity
        self.odom.update_velocity(time, msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z)

        # Predict odometry
        result = self.odom.predict(time)
        if result is not None:
            odom_x, odom_y, odom_yaw = result

            # Publish odom_pose
            pose_msg = PoseStamped()
            pose_msg.header.stamp = msg.header.stamp
            pose_msg.header.frame_id = self.odom_frame_id
            pose_msg.pose.position.x = odom_x
            pose_msg.pose.position.y = odom_y
            pose_msg.pose.position.z = 0.0

            quat = tf.transformations.quaternion_from_euler(0.0, 0.0, odom_yaw)
            pose_msg.pose.orientation.x = quat[0]
            pose_msg.pose.orientation.y = quat[1]
            pose_msg.pose.orientation.z = quat[2]
            pose_msg.pose.orientation.w = quat[3]

            self.pub_odom.publish(pose_msg)

            # Publish TF
            tf_odom = TransformStamped()
            tf_odom.header.stamp = msg.header.stamp
            tf_odom.header.frame_id = self.odom_frame_id
            tf_odom.child_frame_id = self.base_frame_id
            tf_odom.transform.translation.x = odom_x
            tf_odom.transform.translation.y = odom_y
            tf_odom.transform.translation.z = 0.0
            tf_odom.transform.rotation.x = quat[0]
            tf_odom.transform.rotation.y = quat[1]
            tf_odom.transform.rotation.z = quat[2]
            tf_odom.transform.rotation.w = quat[3]

            self.tf_broadcaster.sendTransform(tf_odom)

    def init_ros(self):
        """Initialize ROS and wait for initial pose from /pose_gt."""
        rospy.loginfo("Waiting for first vehicle pose from /pose_gt")
        try:
            msg = rospy.wait_for_message('/pose_gt', PoseStamped, timeout=30.0)
            # Extract yaw from quaternion
            quat = (msg.pose.orientation.x, msg.pose.orientation.y,
                    msg.pose.orientation.z, msg.pose.orientation.w)
            yaw = tf.transformations.euler_from_quaternion(quat)[2]
            time = msg.header.stamp.to_sec()
            x = msg.pose.position.x
            y = msg.pose.position.y

            # Set initial pose
            self.odom.set_initial_pose(time, x, y, yaw)
            self.publish_initial_pose(msg.header.stamp, x, y, yaw)
        except rospy.ROSException:
            rospy.logerr("No initial position received from /pose_gt!")
            time = rospy.get_time()
            self.odom.set_initial_pose(time, 0.0, 0.0, 0.0)
            self.publish_initial_pose(rospy.Time.now(), 0.0, 0.0, 0.0)

    def start(self):
        """Start the ROS node."""
        rospy.spin()

if __name__ == '__main__':
    try:
        odom = Odom()
        odom.start()
    except rospy.ROSInterruptException:
        pass