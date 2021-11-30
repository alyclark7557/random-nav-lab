#!/usr/bin/env python
"""
This node addresses some rough-edges when working with TB3 under dashing:

 - Scan messages don't show up in rviz because they run ahead of tf.
     - This node delays and republishes /scan messages on /scan_viz.

 - Rviz expects camera_info messages on a different topic (from Gazebo).
   and the frame_id is incorrect .
     - Republish camara/image_raw to camera/image_raw_viz
     - Republish camera/camera_info to camera/image_raw_viz/camera_info

 - Turtlebot continues to obey last cmd_vel message forever, which can
   be akward if a control node crashes or exits while the robot is moving.
     - Periodically check to see if no other nodes are publishing to /cmd_vel
       if not, publish a stop command.

Author: Nathan Sprague
Version: 10/18/2020

"""
import time
import rclpy
import rclpy.node
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class TBFixNode(rclpy.node.Node):

    def __init__(self):
        super().__init__('tb_fixer')

        group = rclpy.callback_groups.ReentrantCallbackGroup()

        self.thrust_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.info_pub = self.create_publisher(CameraInfo,
                                              'camera/image_raw_viz/camera_info',
                                              10)
        self.img_pub = self.create_publisher(Image,
                                             'camera/image_raw_viz',
                                             10)

        self.scan_pub = self.create_publisher(LaserScan, 'scan_viz', 10)

        self.create_timer(.1, self.timer_callback, callback_group=group)

        self.create_subscription(CameraInfo,
                                 'camera/camera_info',
                                 self.info_callback,
                                 qos_profile_sensor_data,
                                 callback_group=group)

        self.create_subscription(Image,
                                 'camera/image_raw',
                                 self.img_callback,
                                 qos_profile_sensor_data,
                                 callback_group=group)

        self.create_subscription(LaserScan, 'scan',
                                 self.scan_callback,
                                 qos_profile_sensor_data,
                                 callback_group=group)

    def scan_callback(self, scan_msg):
        time.sleep(.1)
        self.scan_pub.publish(scan_msg)

    def timer_callback(self):
        if self.count_publishers('cmd_vel') == 1:  # Just this node.
            twist = Twist()
            self.thrust_pub.publish(twist)

    def info_callback(self, info_msg):
        info_msg.header.frame_id = 'camera_rgb_optical_frame'
        self.info_pub.publish(info_msg)
        
    def img_callback(self, img_msg):
        img_msg.header.frame_id = 'camera_rgb_optical_frame'
        self.img_pub.publish(img_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TBFixNode()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor.spin()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
