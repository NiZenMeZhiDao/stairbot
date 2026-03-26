#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class SuspensionController(Node):
    def __init__(self):
        super().__init__('suspension_controller')

        # --- 底盘四轮高度状态 ---
        # [左前, 右前, 左后, 右后]
        self.wheel_heights = [0.0, 0.0, 0.0, 0.0] 
        self.high = 0
        # --- 传感器状态变量 ---
        # 仅保留正前方的 2 个测距数据 (初始值设为很大，代表安全)
        self.front_distances = [999.0, 999.0] 
        # 仅保留正前方的 2 个下视光电状态 (0.0为悬空，1.0为接触)
        self.front_downward = [0.0, 0.0]       

        # --- 发布者 ---
        # 100Hz 发布四轮高度 (Float32MultiArray, 4个数据)
        self.height_pub = self.create_publisher(
            Float32MultiArray, 
            'wheel_heights', 
            10)

        # --- 订阅者 ---
        # 订阅前向测距数据 (Float32MultiArray, 8个数据)
        self.distance_sub = self.create_subscription(
            Float32MultiArray, 
            'sensor_distances', 
            self.distance_callback, 
            10)
        
        # 订阅下视光电开关数据 (Float32MultiArray, 8个数据)
        self.downward_sub = self.create_subscription(
            Float32MultiArray, 
            'r0x0201', 
            self.downward_callback, 
            10)

        # --- 定时器 (100Hz) ---
        timer_period = 0.01  
        self.timer = self.create_timer(timer_period, self.control_loop)
        
        self.get_logger().info("主动悬挂控制节点已启动 (前向测试模式)")

    def distance_callback(self, msg):
        """处理包含8个数据的测距数组，提取前2个"""
        if len(msg.data) >= 8:
            # 假设 index 0 和 1 是正前方的两个传感器
            self.front_distances[0] = msg.data[0]
            self.front_distances[1] = msg.data[1]
        else:
            self.get_logger().warn("测距数据异常: 数组长度不足8个！")

    def downward_callback(self, msg):
        """处理包含8个数据的下视光电数组，提取前2个"""
        if len(msg.data) >= 8:
            # 假设 index 0 和 1 是正前方下视的两个传感器
            self.front_downward[0] = msg.data[0]
            self.front_downward[1] = msg.data[1]
        else:
            self.get_logger().warn("下视光电数据异常: 数组长度不足8个！")

    def update_suspension_logic(self):
        """
        前向测试：上楼梯控制逻辑
        """
    
        # 示例：动作 1 - 接近楼梯，抬起前轮
        # 当距离小于 0.5 米时触发
        if (self.front_distances[0] < 200.0) & (self.front_distances[1] < 200.0):
            self.wheel_heights[0] = 230.0  # 左前抬升
            self.wheel_heights[1] = 230.0  # 右前抬升
            self.wheel_heights[2] = 230.0  # 左前抬升
            self.wheel_heights[3] = 230.0  # 右前抬升
            self.high = 1
            # self.get_logger().debug("检测到楼梯，前轮已抬升")
        if (self.front_distances[0] < 200.0) & (self.front_distances[1] < 400.0):
            self.wheel_heights[0] = 430.0  # 左前抬升
            self.wheel_heights[1] = 430.0  # 右前抬升
            self.wheel_heights[2] = 430.0  # 左前抬升
            self.wheel_heights[3] = 430.0  # 右前抬升
            self.high = 2
            # self.get_logger().debug("检测到楼梯，前轮已抬升")
        # 示例：动作 2 - 前轮已经搭上台阶
        # 两个前向下视光电开关都检测到接触面 (值为 1.0)
        if (self.front_downward[0] == 1.0) & (self.high >= 1):
            self.wheel_heights[0] = 0.0  # 左前放下
            self.wheel_heights[1] = 0.0  # 右前放下
        if (self.front_downward[1] == 1.0) & (self.high >= 1):
            self.wheel_heights[2] = 0.0  # 左前放下
            self.wheel_heights[3] = 0.0  # 右前放下 

    def control_loop(self):
        """100Hz 定时器控制循环"""
        # 更新逻辑
        self.update_suspension_logic()

        # 发布数据
        msg = Float32MultiArray()
        msg.data = self.wheel_heights
        self.height_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SuspensionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('节点已被手动停止')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()