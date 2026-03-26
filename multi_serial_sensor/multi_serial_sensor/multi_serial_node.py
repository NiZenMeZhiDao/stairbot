#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
import math

class MultiSerialPublisher(Node):
    def __init__(self):
        super().__init__('multi_serial_publisher')
        
        # 创建发布者，发布类型为 Float32MultiArray，话题名为 /sensor_distances
        self.publisher_ = self.create_publisher(Float32MultiArray, 'sensor_distances', 10)
        
        # 创建定时器，100Hz = 每 0.01 秒触发一次
        self.timer = self.create_timer(0.01, self.timer_callback)
        
        # 串口配置
        self.port_names = [f'/dev/ttyCH9344USB{i}' for i in range(8)]
        self.baudrate = 230400
        
        self.serials = []
        self.buffers = [bytearray() for _ in range(8)]
        self.last_distances = [math.nan] * 8  # 缓存上一次的有效数据，默认全为 NaN
        self.miss_counts = [0] * 8  # 记录每个串口连续未读取到有效数据的次数
        
        # 初始化 8 个串口
        for port in self.port_names:
            try:
                # 【关键】timeout=0 设置为非阻塞模式，保证 100Hz 定时器不被卡死
                ser = serial.Serial(port, self.baudrate, timeout=0)
                self.serials.append(ser)
                self.get_logger().info(f'成功打开串口: {port}')
            except serial.SerialException:
                self.serials.append(None)
                self.get_logger().warn(f'未连接设备或无法打开: {port}')

    def parse_packet(self, data):
        """解析单帧 195 字节的数据包，寻找自信度为 100 的测量点"""
        # 遍历这 1 个数据包中的所有测距点 (步长15字节)
        for i in range(10, 195, 15):
            if i + 15 <= len(data):
                confidence = data[i + 8]
                # 【关键】条件过滤：只有自信度等于 100 才提取距离
                if confidence == 100:
                    distance = data[i] | (data[i + 1] << 8)
                    return float(distance)
        
        # 如果整包数据都没有自信度达到 100 的点，返回 NaN
        return math.nan

    def timer_callback(self):
        msg = Float32MultiArray()
        
        # 遍历 8 个串口读取并解析数据
        for i in range(8):
            ser = self.serials[i]
            
            if ser is not None and ser.is_open:
                try:
                    # 1. 读取当前缓冲区所有可用数据
                    if ser.in_waiting > 0:
                        self.buffers[i].extend(ser.read(ser.in_waiting))
                    
                    # 2. 处理完整的数据包
                    while len(self.buffers[i]) >= 195:
                        if self.buffers[i][0] == 0xAA:
                            # 取出一帧完整数据
                            packet = self.buffers[i][:195]
                            # 解析并更新距离（可能是有效数值，也可能是 NaN）
                            distance = self.parse_packet(packet)
                            if math.isnan(distance):
                                # 如果解析结果是 NaN，增加未读取到有效数据的计数
                                self.miss_counts[i] += 1
                            else:
                                # 如果解析成功，重置未读取计数并更新距离
                                self.miss_counts[i] = 0
                                self.last_distances[i] = distance
                            # 从缓冲区移除已处理的数据
                            self.buffers[i] = self.buffers[i][195:]
                        else:
                            # 发生错位，丢弃第一个字节，重新寻找包头 0xAA
                            self.buffers[i].pop(0)

                        # 如果连续 5 次未读取到有效数据，将距离设置为 NaN
                        if self.miss_counts[i] >= 5:
                            if not math.isnan(self.last_distances[i]):
                                # 仅在状态变化时输出日志，避免重复打印
                                self.get_logger().warn(f'串口 {self.port_names[i]} 掉线，连续 5 次未读取到有效数据。')
                            self.last_distances[i] = math.nan

                    # 确保设备拔掉时清理残留数据
                    if ser is None or not ser.is_open:
                        self.last_distances[i] = math.nan
                except Exception as e:
                    self.get_logger().error(f'读取串口 {self.port_names[i]} 失败，设备可能已拔出: {e}')
                    self.serials[i].close()
                    self.serials[i] = None
                    self.last_distances[i] = math.nan  # 断开连接时赋值 NaN
            else:
                # 串口未连接，直接赋值 NaN
                self.last_distances[i] = math.nan
        
        # 将包含 8 个数据的数组装填入 msg 并发布
        msg.data = self.last_distances
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MultiSerialPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('节点被用户手动终止。')
    finally:
        # 安全退出：清理并关闭所有打开的串口
        for ser in node.serials:
            if ser is not None and ser.is_open:
                ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
