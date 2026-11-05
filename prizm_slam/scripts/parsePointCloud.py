#!/usr/bin/env python3
import math
import sys
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

class ParsePointCloud(Node):

    def __init__(self):
        super().__init__("ParsePointCloud")
        self.pc_sub = self.create_subscription(
            PointCloud2, "point_cloud", self.pc_cb, 10)
        self.max_stamps = []
        self.min_stamps = []

    def pc_cb(self, msg: PointCloud2):
        # self.get_logger().info("Logging point cloud")
        # idx map for fields
        # 0: x,         offset 0 
        # 1: y,         offset 4
        # 2: z,         offset 8
        # 3: intensity, offset 12
        # 4: index,     offset 16
        # 5: stamps,    offset 18
        
        # self.get_logger().info(f"data len: {len(msg.data) % 18}")
        pc2_stamp = point_cloud2.read_points_numpy(msg, field_names=["stamps"])
        self.get_logger().info(f"min: {np.min(pc2_stamp)}, max: {np.max(pc2_stamp)}")
        self.max_stamps.append(np.max(pc2_stamp))
        self.min_stamps.append(np.min(pc2_stamp))

    def print_results(self):
        self.min_stamps = np.array(self.min_stamps)
        self.max_stamps = np.array(self.max_stamps)
        self.get_logger().info(f"Avg min: {self.min_stamps.mean()}, pm {self.min_stamps.std()}")
        self.min_stamps_trim = np.trim_zeros(self.min_stamps)
        self.get_logger().info(f"Avg min trim: {self.min_stamps_trim.mean()}, pm {self.min_stamps_trim.std()}")
        self.get_logger().info(f"Avg max: {self.max_stamps.mean()}, pm {self.max_stamps.std()}")

def main(args=None):

    rclpy.init(args=args)
    
    point_cloud_parser = ParsePointCloud()

    try:
        rclpy.spin(point_cloud_parser)
    except KeyboardInterrupt:
        pass
    finally:
        point_cloud_parser.print_results()
        # point_cloud_parser.destroy_node()
        # rclpy.shutdown()


if __name__ == '__main__':
    main()