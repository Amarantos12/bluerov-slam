# 记录输出的话题/bruce/slam/localization/odom，/bruce/slam/slam/pose

#!/usr/bin/env python
import rospy
import rosbag
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import os
from datetime import datetime

class BagRecorder:
    def __init__(self):
        # 初始化 ROS 节点
        rospy.init_node('slam_pose_recorder', anonymous=True)
        
        # 输出目录和动态文件名
        output_dir = '/home/hzr/catkin_ws/src/sonar-SLAM/output'
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)
        self.bag_filename = os.path.join(output_dir, f"aracati_part1_pose.bag")
        
        # 打开输出 bag 文件
        self.bag = rosbag.Bag(self.bag_filename, 'w')
        
        # 订阅话题
        rospy.Subscriber('/bruce/slam/localization/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/bruce/slam/slam/pose', PoseWithCovarianceStamped, self.pose_callback)
        
        rospy.loginfo(f"Recording started. Saving to: {self.bag_filename}")
    
    def odom_callback(self, msg):
        self.bag.write('/bruce/slam/localization/odom', msg, t=msg.header.stamp)
        # rospy.loginfo(f"Wrote Odometry message with stamp: {msg.header.stamp}")
    
    def pose_callback(self, msg):
        self.bag.write('/bruce/slam/slam/pose', msg, t=msg.header.stamp)
        # rospy.loginfo(f"Wrote PoseWithCovarianceStamped message with stamp: {msg.header.stamp}")
    
    def shutdown(self):
        self.bag.close()
        rospy.loginfo(f"Recording stopped. Bag saved to: {self.bag_filename}")

if __name__ == '__main__':
    recorder = BagRecorder()
    rospy.on_shutdown(recorder.shutdown)
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        recorder.shutdown()