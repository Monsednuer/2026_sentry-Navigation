#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import os

import rclpy
from rclpy.node import Node
import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs
from sensor_msgs_py import point_cloud2

import numpy as np
import open3d as o3d


class PCDPublisher(Node):
    def __init__(self, pcd_path = "/home/hustlyrm/slam_source/odom_ws/pcd/map.pcd", topic_name="pointcloud", frame_id="map", publish_rate=10.0):
        super().__init__('pcd_publisher_node')
        
        # 检查文件是否存在
        if not os.path.exists(pcd_path):
            self.get_logger().error(f"PCD file not found: {pcd_path}")
            sys.exit(1)
            
        self.pcd_path = pcd_path
        self.frame_id = frame_id
        
        # 使用 Open3D 读取 PCD 文件
        self.get_logger().info(f"Loading PCD file: {pcd_path}")cd
        pcd = o3d.io.read_point_cloud(pcd_path)
        
        if len(pcd.points) == 0:
            self.get_logger().error("No points found in PCD file!")
            sys.exit(1)
            
        # 转换为 numpy 数组
        points = np.asarray(pcd.points, dtype=np.float32)
        
        # 如果有颜色信息也一并读取
        if pcd.has_colors():
            colors = np.asarray(pcd.colors, dtype=np.float32)  # 0-1 范围
            self.get_logger().info(f"Loaded {len(points)} points with color")
            self.points_with_color = np.hstack([points, colors])
            self.has_color = True
        else:
            self.get_logger().info(f"Loaded {len(points)} points (no color)")
            self.points_with_color = points
            self.has_color = False
            
        self.num_points = len(points)
        
        # 创建发布者
        self.pcd_publisher = self.create_publisher(
            sensor_msgs.PointCloud2, 
            topic_name, 
            10
        )
        
        # 创建定时器发布点云
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info(
            f"Publishing to topic '{topic_name}' at {publish_rate} Hz, "
            f"frame_id='{frame_id}'"
        )

    def timer_callback(self):
        """构造并发布 PointCloud2 消息"""
        if self.has_color:
            # 发布带颜色的点云 (x, y, z, r, g, b)
            msg = point_cloud2.create_cloud(
                header=std_msgs.Header(frame_id=self.frame_id, stamp=self.get_clock().now().to_msg()),
                fields=[
                    sensor_msgs.PointField(name='x', offset=0, datatype=sensor_msgs.PointField.FLOAT32, count=1),
                    sensor_msgs.PointField(name='y', offset=4, datatype=sensor_msgs.PointField.FLOAT32, count=1),
                    sensor_msgs.PointField(name='z', offset=8, datatype=sensor_msgs.PointField.FLOAT32, count=1),
                    sensor_msgs.PointField(name='r', offset=12, datatype=sensor_msgs.PointField.FLOAT32, count=1),
                    sensor_msgs.PointField(name='g', offset=16, datatype=sensor_msgs.PointField.FLOAT32, count=1),
                    sensor_msgs.PointField(name='b', offset=20, datatype=sensor_msgs.PointField.FLOAT32, count=1),
                ],
                points=self.points_with_color
            )
        else:
            # 发布纯 XYZ 点云
            msg = point_cloud2.create_cloud_xyz32(
                header=std_msgs.Header(frame_id=self.frame_id, stamp=self.get_clock().now().to_msg()),
                points=self.points_with_color
            )
            
        self.pcd_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    
    # # 从命令行参数获取 PCD 路径，默认使用第一个参数
    # if len(sys.argv) < 2:
    #     print("Usage: ros2 run <package> pcd_publisher <path_to_pcd> [topic_name] [frame_id]")
    #     print("Example: python3 pcd_publisher.py /home/user/map.pcd")
    #     sys.exit(1)
        
    # pcd_path = sys.argv[1]
    # topic_name = sys.argv[2] if len(sys.argv) > 2 else "pointcloud"
    # frame_id = sys.argv[3] if len(sys.argv) > 3 else "map"
    pcd_path = "/home/hustlyrm/slam_source/odom_ws/pcd/map.pcd"
    topic_name = "pointcloud"
    frame_id = "map"
    
    
    node = PCDPublisher(pcd_path, topic_name, frame_id)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()