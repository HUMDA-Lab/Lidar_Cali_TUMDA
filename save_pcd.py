import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import open3d as o3d
import os
from rclpy.qos import QoSProfile, ReliabilityPolicy

class PointCloudSaver(Node):
    def __init__(self):
        super().__init__('pcd_saver')

        ouster_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        
        self.front_sub = self.create_subscription(
            PointCloud2,
            '/sensor/lidar_front/points',
            self.front_callback,
            10)
        self.right_sub = self.create_subscription(
            PointCloud2,
            '/sensor/lidar_right/points',
            self.right_callback,
            10)
        self.left_sub = self.create_subscription(
            PointCloud2,
            '/sensor/lidar_left/points',
            self.left_callback,
            10)
        self.ouster_sub = self.create_subscription(
            PointCloud2,
            '/ouster/points',
            self.ouster_callback,
            ouster_qos_profile)
        
        os.makedirs("saved_pcd", exist_ok=True)
        os.makedirs("saved_pcd/front", exist_ok=True)
        os.makedirs("saved_pcd/right", exist_ok=True)
        os.makedirs("saved_pcd/left", exist_ok=True)
        os.makedirs("saved_pcd/ouster", exist_ok=True)
        self.front_counter = 0
        self.right_counter = 0
        self.left_counter = 0
        self.ouster_counter = 0
        self.get_logger().info("PointCloudSaver node has been started.")

    def front_callback(self, msg):
        points = list(pc2.read_points(msg, field_names=["x", "y", "z"], skip_nans=True))
        if not points:
            self.get_logger().info("No points to save.")
            return

        np_points = np.array([[p[0], p[1], p[2]] for p in points], dtype=np.float32)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np_points)

        filename = f"saved_pcd/front/lidar_front_{self.front_counter:04d}.pcd"
        o3d.io.write_point_cloud(filename, pcd)
        self.get_logger().info(f"Saved {filename}")
        self.front_counter += 1

    def right_callback(self, msg):
        points = list(pc2.read_points(msg, field_names=["x", "y", "z"], skip_nans=True))
        if not points:
            self.get_logger().info("No points to save.")
            return

        np_points = np.array([[p[0], p[1], p[2]] for p in points], dtype=np.float32)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np_points)

        filename = f"saved_pcd/right/lidar_right_{self.right_counter:04d}.pcd"
        o3d.io.write_point_cloud(filename, pcd)
        self.get_logger().info(f"Saved {filename}")
        self.right_counter += 1

    def left_callback(self, msg):
        points = list(pc2.read_points(msg, field_names=["x", "y", "z"], skip_nans=True))
        if not points:
            self.get_logger().info("No points to save.")
            return

        np_points = np.array([[p[0], p[1], p[2]] for p in points], dtype=np.float32)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np_points)

        filename = f"saved_pcd/left/lidar_left_{self.left_counter:04d}.pcd"
        o3d.io.write_point_cloud(filename, pcd)
        self.get_logger().info(f"Saved {filename}")
        self.left_counter += 1

    def ouster_callback(self, msg):
        points = list(pc2.read_points(msg, field_names=["x", "y", "z"], skip_nans=True))
        if not points:
            self.get_logger().info("No points to save.")
            return

        np_points = np.array([[p[0], p[1], p[2]] for p in points], dtype=np.float32)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np_points)

        filename = f"saved_pcd/ouster/os_lidar_{self.ouster_counter:04d}.pcd"
        o3d.io.write_point_cloud(filename, pcd)
        self.get_logger().info(f"Saved {filename}")
        self.ouster_counter += 1

def main():
    rclpy.init()
    node = PointCloudSaver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
