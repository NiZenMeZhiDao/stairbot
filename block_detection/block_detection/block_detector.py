#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32, String
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import open3d as o3d
import cv2
import json

class BlockDetectorNode(Node):
    def __init__(self):
        super().__init__('block_detector')
        
        self.lidar_sub = self.create_subscription(
            PointCloud2, '/livox/lidar', self.lidar_callback, 10)
            
        self.height_sub = self.create_subscription(
            Float32, '/robot/lift_height', self.height_callback, 10)
            
        self.result_pub = self.create_publisher(
            String, '/robot/block_status', 10)

        self.base_lidar_height = 0.54 
        self.current_lift_height = 0.0 

        self.get_logger().info("方块检测节点已启动 (已加入 1.2m 尺寸校验与鲁棒高度检测).")

    def height_callback(self, msg):
        self.current_lift_height = msg.data

    def lidar_callback(self, msg):
        pc_data = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        points_lidar = np.array(list(pc_data))
        if len(points_lidar) < 100:
            return

        # 1. 坐标系换算：雷达坐标 -> 机器人底盘投影坐标
        H_total = self.base_lidar_height + self.current_lift_height
        points_base = np.zeros_like(points_lidar)
        points_base[:, 0] = points_lidar[:, 0]
        points_base[:, 1] = -points_lidar[:, 1]
        points_base[:, 2] = -points_lidar[:, 2] + H_total
        
        # 2. 滤除机器人自身遮挡
        body_mask = (np.abs(points_base[:, 0]) < 0.35) & (np.abs(points_base[:, 1]) < 0.35)
        points_env = points_base[~body_mask]

        # 3. 提取地面点并 RANSAC 拟合当前方块
        ground_mask = (np.abs(points_env[:, 2]) < 0.1)
        near_ground_points = points_env[ground_mask]
        
        if len(near_ground_points) < 50:
            return

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(near_ground_points)
        _, inliers = pcd.segment_plane(distance_threshold=0.03, ransac_n=3, num_iterations=100)
        current_block_points = near_ground_points[inliers]

        if len(current_block_points) < 20:
            return

        # 4. 计算偏航角与【尺寸校验】
        points_2d = np.float32(current_block_points[:, :2])
        rect = cv2.minAreaRect(points_2d)
        
        # 拆解出矩形的中心点、宽高、角度
        (center_x, center_y), (width, height), angle = rect
        
        # ==================== 改进 1：检查长宽是否在 1.2m 左右 ====================
        # 考虑到点云稀疏、遮挡和提取误差，容差设为 1.05m 到 1.35m 之间
        if not (1.05 < width < 1.35 and 1.05 < height < 1.35):
            # 如果不符合，说明这可能不是一个完整的方块，拒绝输出，防止带偏机器人
            # 此处可取消注释用于调试：
            # self.get_logger().warn(f"当前平面尺寸不符 W:{width:.2f}, H:{height:.2f}，跳过该帧")
            return
        # ========================================================================

        box = cv2.boxPoints(rect)
        edge_vector = box[1] - box[0]
        yaw_rad = np.arctan2(edge_vector[1], edge_vector[0])
        yaw_rad = (yaw_rad + np.pi/4) % (np.pi/2) - np.pi/4
        yaw_deg = float(np.degrees(yaw_rad))
        
        # 5. 生成反向旋转矩阵，将点云 "摆正"
        cos_val = np.cos(-yaw_rad)
        sin_val = np.sin(-yaw_rad)
        rot_matrix = np.array([
            [cos_val, -sin_val, 0],
            [sin_val,  cos_val, 0],
            [0,        0,       1]
        ])
        
        points_aligned = np.dot(points_env, rot_matrix.T)
        block_aligned = np.dot(current_block_points, rot_matrix.T)
        
        # 6. 求出绝对垂直的前、左边界距离
        min_x = np.min(block_aligned[:, 0])
        max_x = np.max(block_aligned[:, 0])
        min_y = np.min(block_aligned[:, 1])
        max_y = np.max(block_aligned[:, 1])
        
        dist_front = max_x  
        dist_left = max_y   

        # 7. 在“摆正”后的坐标系中，精准探查四面方块高度
        h_front = self.analyze_region_height(points_aligned, max_x + 0.05, max_x + 0.525, -0.4, 0.4)
        h_back  = self.analyze_region_height(points_aligned, min_x - 0.525, min_x - 0.05, -0.4, 0.4)
        h_left  = self.analyze_region_height(points_aligned, -0.4, 0.4, max_y + 0.05, max_y + 0.525)
        h_right = self.analyze_region_height(points_aligned, -0.4, 0.4, min_y - 0.525, min_y - 0.05)

        # 8. 发布增强版 JSON 数据
        result_dict = {
            "yaw_offset_deg": round(yaw_deg, 2), 
            "dist_front": round(float(dist_front), 3),
            "dist_left": round(float(dist_left), 3),
            "heights": {
                "front": h_front,
                "back": h_back,
                "left": h_left,
                "right": h_right
            }
        }
        msg_out = String()
        msg_out.data = json.dumps(result_dict)
        self.result_pub.publish(msg_out)

    def analyze_region_height(self, points, x_min, x_max, y_min, y_max):
        mask = (points[:, 0] > x_min) & (points[:, 0] < x_max) & \
               (points[:, 1] > y_min) & (points[:, 1] < y_max)
        region_points = points[mask]

        # 提高点数阈值，点太少不足以进行统计学分析
        if len(region_points) < 30: 
            return -999 

        z_values = region_points[:, 2]

        # ==================== 改进 2：多点采样替代极值 ====================
        # 对 Z 坐标进行排序
        z_sorted = np.sort(z_values)

        # 动态决定采样数量 (取总点数的 10%，但最少取 5 个点，最多取 50 个点)
        sample_n = max(5, min(50, int(len(z_sorted) * 0.1)))

        # 鲁棒最高点：取最高的 N 个点求平均
        z_robust_max = np.mean(z_sorted[-sample_n:])
        
        # 鲁棒最低点：取最低的 N 个点求平均 (可用于辅助判断悬崖)
        z_robust_min = np.mean(z_sorted[:sample_n])
        
        # 主体平面中位数
        z_median = np.median(z_values)
        # ================================================================

        # 【墙壁/凸起检测】使用 z_robust_max 替代原先的 z_max
        if z_robust_max > 0.12:  
            if 0.12 <= z_robust_max < 0.30: return 200
            elif 0.30 <= z_robust_max < 0.55: return 400
            else: return 999 

        # 【落差/下降检测】依然使用 z_median 较为稳定，因为坑底通常是一个面
        if z_median < -0.12:
            if -0.30 < z_median <= -0.12: return -200
            elif -0.55 < z_median <= -0.30: return -400
                
        # 【平地检测】
        if -0.10 <= z_median <= 0.10:
            return 0

        return 999 

def main(args=None):
    rclpy.init(args=args)
    node = BlockDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()