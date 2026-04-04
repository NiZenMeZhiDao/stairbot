#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, Int32
import math
from enum import Enum
import collections
import time

# 定义系统状态机
class State(Enum):
    IDLE = 0
    UP_1_PREPARE = 10
    UP_2_LIFT = 11
    UP_3_FRONT_DOCK = 12
    UP_4_RETRACT_FRONT = 13
    UP_5_FRONT_LAND = 14
    UP_6_SIDE_DOCK_RETRACT_REAR = 15
    UP_7_REAR_LAND = 16
    UP_8_RECOVER = 17
    
    DOWN_1_PREPARE = 20
    DOWN_2_FRONT_HOVER_LAND = 21
    DOWN_3_REAR_HOVER_LAND = 22
    DOWN_4_LAND = 23
    DOWN_5_RECOVER = 24

class Direction(Enum):
    FORWARD = 0
    LEFT = 1
    RIGHT = 2
    BACKWARD = 3

class SuspensionController(Node):
    def __init__(self):
        super().__init__('suspension_controller')
        
        # --- 参数配置 ---
        self.H_LIFT_LOW = 200.0   # 台阶高度1
        self.H_LIFT_HIGH = 400.0  # 台阶高度2
        self.H_INIT = 30.0        # 初始/常规运动姿态高度
        self.CREEP_SPEED = 0.2   # 安全蠕行速度 (m/s)
        self.HEIGHT_TOLERANCE = 0.05 # 5% 误差范围允许转跳
        
        # --- 状态变量 ---
        self.current_state = State.IDLE
        self.target_height = 0.0  # 当前识别到的台阶目标高度
        self.current_direction = Direction.FORWARD
        self.is_reversing = False
        
        # 物理层状态
        self.raw_cmd_vel = Twist()
        self.distances_raw = [0.0] * 8
        self.pe_switches_raw = [0] * 4
        self.wheel_heights_current = [0.0] * 4  # [FL, FR, RL, RR]
        
        # 滤波后状态
        self.distance_filtered = [0.0] * 8
        self.pe_switches_filtered = [0] * 4
        
        # 输出控制状态
        self.wheel_heights_target = [self.H_INIT] * 4
        self.chassis_cmd_vel = Twist()
        self.v_distances_idx = [0.0] * 6

        # 虚拟层状态 (根据方向映射后的 前左, 前右, 后左, 后右)
        self.v_wheels_idx = [0, 1, 2, 3] 
        self.v_pe_idx = [0, 1, 2, 3]

        # --- 滤波器初始化 ---
        self.distance_buffers = [collections.deque(maxlen=5) for _ in range(6)]
        self.pe_debounce_counters = [0] * 4  # 用于 10ms 防抖 (主频 100Hz 下 1 帧 = 10ms)
        self.pe_last_states = [0] * 4
        
        # --- ROS 2 接口 ---
        self.sub_cmd_vel = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_cb, 10)
        self.sub_sensor_dist = self.create_subscription(Float32MultiArray, 'sensor_distances', self.dist_cb, 10)
        self.sub_r0x0201 = self.create_subscription(Float32MultiArray, 'r0x0201', self.hw_status_cb, 10)
        
        self.pub_wheel_heights = self.create_publisher(Float32MultiArray, 't0x0101_wheel_heights', 10)
        self.pub_chassis_vel = self.create_publisher(Twist, 'cmd_vel_chassis', 10) # 实际下发到底盘的速度
        self.pub_state = self.create_publisher(Int32, 'current_state', 10)
        # 控制主循环 (100Hz)
        self.timer = self.create_timer(0.01, self.control_loop)
        self.get_logger().info("Step Climber Node Initialized.")

    # ================= 数据接收与滤波 =================
    def cmd_vel_cb(self, msg):
        self.raw_cmd_vel = msg
        # 仲裁方向逻辑
        if msg.linear.x > 0:
            self.current_direction = Direction.FORWARD
            self.is_reversing = False
        # elif msg.linear.x < 0:
            # self.is_reversing = True # 触发倒退逻辑
        elif msg.linear.y > 0:
            self.current_direction = Direction.LEFT
            self.is_reversing = False
        elif msg.linear.y < 0:
            self.current_direction = Direction.RIGHT
            self.is_reversing = False

    def dist_cb(self, msg):
        if len(msg.data) >= 8:
            for i in range(8):
                self.distance_buffers[i].append(msg.data[i])
                self.distance_filtered[i] = sum(self.distance_buffers[i]) / len(self.distance_buffers[i])

    def hw_status_cb(self, msg):
        # r0x0201: [PE_0, PE_1, PE_2, PE_3, H_FL, H_FR, H_RL, H_RR]
        if len(msg.data) >= 8:
            for i in range(4):
                current_pe = int(msg.data[i])
                # 10ms (1 frame) 防抖逻辑
                if current_pe != self.pe_last_states[i]:
                    self.pe_debounce_counters[i] += 1
                    if self.pe_debounce_counters[i] >= 1: # 连续1次跳变(10ms)确认
                        self.pe_switches_filtered[i] = current_pe
                        self.pe_last_states[i] = current_pe
                        self.pe_debounce_counters[i] = 0
                else:
                    self.pe_debounce_counters[i] = 0
                    
            for i in range(4):
                self.wheel_heights_current[i] = msg.data[4 + i]
            self.raw_cmd_vel.linear.x = msg.data[8]  
            self.raw_cmd_vel.linear.y = msg.data[9]
            self.raw_cmd_vel.angular.z = msg.data[10]
            if self.current_state == State.IDLE:
                if(msg.data[11] > 0):
                    self.current_direction = Direction.RIGHT
                elif(msg.data[11] < 0):
                    self.current_direction = Direction.LEFT
                else:
                    self.current_direction = Direction.FORWARD        
    # ================= 核心映射与控制 =================
    def update_virtual_mapping(self):
        """根据行驶方向，将物理轮/传感器映射为虚拟的 前左(VFL), 前右(VFR), 后左(VRL), 后右(VRR)"""
        if self.current_direction == Direction.FORWARD:
            self.v_wheels_idx = [2, 1, 0, 3] # [FL, FR, RL, RR]
            self.v_pe_idx = [0, 1, 3, 2]     # 假设 0,1为前，2,3为后
            self.v_distances_idx = [0, 1, 5, 4] # 前后距离传感器不变
        elif self.current_direction == Direction.LEFT:
            self.v_wheels_idx = [0, 2, 3, 1] 
            self.v_pe_idx = [1, 2, 0, 3]     
            self.v_distances_idx = [2, 3, 7, 6]
        elif self.current_direction == Direction.RIGHT:
            self.v_wheels_idx = [1, 3, 2, 0]
            self.v_pe_idx = [3, 0, 2, 1]
            self.v_distances_idx = [6, 7, 3, 2]

    def check_height_reached(self, virtual_indices, target_h):
        """高度闭环验证"""
        for v_idx in virtual_indices:
            phys_idx = self.v_wheels_idx[v_idx]
            curr_h = self.wheel_heights_current[phys_idx]
            # 允许 5% 误差，若目标为 0，则绝对误差 < 5mm
            if target_h == 0:
                if abs(curr_h) > 5.0: return False
            else:
                if abs(curr_h - target_h) > (target_h * self.HEIGHT_TOLERANCE): return False
        return True

    def control_loop(self):
        self.update_virtual_mapping()
        
        v_fl, v_fr, v_rl, v_rr = 0, 1, 2, 3 

        self.execute_state_machine(v_fl, v_fr, v_rl, v_rr)

        h_msg = Float32MultiArray(data=self.wheel_heights_target)
        self.pub_wheel_heights.publish(h_msg)
        self.pub_chassis_vel.publish(self.chassis_cmd_vel)
        
        state_msg = Int32(data=[self.current_state.value])
        self.pub_state.publish(state_msg)

    def execute_state_machine(self, v_fl, v_fr, v_rl, v_rr):
        """核心状态机"""
        state = self.current_state
     
        self.chassis_cmd_vel.linear.x = self.raw_cmd_vel.linear.x
        self.chassis_cmd_vel.linear.y = self.raw_cmd_vel.linear.y
        self.chassis_cmd_vel.angular.z = self.raw_cmd_vel.angular.z

        if state == State.IDLE:
            front_dist = self._get_v_distance(1)
            if front_dist < 200:  # 前方有台阶，准备上台阶
                self.current_state = State.UP_1_PREPARE
            if self._get_v_distance(0) > 200:
                self.current_state = State.DOWN_1_PREPARE

        
        elif state == State.UP_1_PREPARE:
            self.target_height = self.H_LIFT_LOW
            self.wheel_heights_target = [self.target_height] * 4
            if self._get_v_distance(0) > 200: #已经到200
                if self._get_v_distance(1) < 200: #还有遮挡，是400
                    self.target_height = self.H_LIFT_HIGH
                    self.wheel_heights_target = [self.target_height] * 4
                    
                    self.current_state = State.UP_2_LIFT
                elif self._get_v_distance(1) > 250: #无遮挡，是200
                    self.target_height = self.H_LIFT_LOW
                    self.wheel_heights_target = [self.target_height] * 4
                    
                    self.current_state = State.UP_2_LIFT

        elif state == State.UP_2_LIFT:
            # 整体升起，速度强制为 0
            self._stop_chassis()
            self.wheel_heights_target = [self.target_height] * 4
            if self.check_height_reached([v_fl, v_fr, v_rl, v_rr], self.target_height):
                self.current_state = State.UP_3_FRONT_DOCK

        elif state == State.UP_3_FRONT_DOCK:
            # 前轮搭接：钳位速度为安全蠕行速度
            self._creep_forward()
            # 检查前置向下测距 (假设索引1代表前从动轮测距)
            if self._get_v_distance(0) < 80.0:  # 已经稳稳搭在平台上
                self.current_state = State.UP_4_RETRACT_FRONT

        elif state == State.UP_4_RETRACT_FRONT:
            # 收起前主动轮，等待电机到位
            self._stop_chassis()
            self._set_v_wheel_height([v_fl, v_fr], 0.0) # 0为与从动轮齐平
            if self.check_height_reached([v_fl, v_fr], 0.0):
                self.current_state = State.UP_5_FRONT_LAND

        elif state == State.UP_5_FRONT_LAND:
            self._creep_forward()
            # 检查虚拟前轮的下视光电开关 (有遮挡表示处于平台上空)
            pe_front_clear = (self._get_v_pe(v_fl) == 1) #1有遮挡
            if pe_front_clear:
                self._set_v_wheel_height([v_fl, v_fr], 10.0) # 降下前主动轮
                self._set_v_wheel_height([v_rl, v_rr], self.target_height + 10.0) 
                self.current_state = State.UP_6_SIDE_DOCK_RETRACT_REAR

        elif state == State.UP_6_SIDE_DOCK_RETRACT_REAR:
            self._creep_forward()
            # 假设后轮前方光电开关被遮挡，表明车体中部搭上
            # 此时立即收起后轮
            if self._get_v_pe(v_rl) == 1: 
                self._set_v_wheel_height([v_rl, v_rr], 0.0)
                if self.check_height_reached([v_rl, v_rr], 0.0):
                    self.current_state = State.UP_7_REAR_LAND

        elif state == State.UP_7_REAR_LAND:
            self._creep_forward()
            if self._get_v_pe(v_rr) == 1: # 后轮光电被遮挡
                self._set_v_wheel_height([v_rl, v_rr], 10.0)
                if self.check_height_reached([v_rl, v_rr], 10.0):
                    self.current_state = State.UP_8_RECOVER

        elif state == State.UP_8_RECOVER:
            self._stop_chassis()
            self.wheel_heights_target = [self.H_INIT] * 4 # 恢复常规行驶姿态
            if self.check_height_reached([v_fl, v_fr, v_rl, v_rr], self.H_INIT):
                self.get_logger().info("Up step sequence complete.")
                self.current_state = State.IDLE

        # ================= 下台阶逻辑 =================
        elif state == State.DOWN_1_PREPARE:
            self._stop_chassis()
            if self._get_v_distance(0) > 380: #检测200还是400
                self.target_height = self.H_LIFT_HIGH # 反向高度
            elif self._get_v_distance(0) > 180:
                self.target_height = self.H_LIFT_LOW
            self.current_state = State.DOWN_2_FRONT_HOVER_LAND
            
        elif state == State.DOWN_2_FRONT_HOVER_LAND:
            self._creep_forward()
            pe_front_hover = (self._get_v_pe(v_fl) == 0) # 前轮光电开关无遮挡，表示前轮悬空放下前轮
            if pe_front_hover:
                self._set_v_wheel_height([v_fl, v_fr], self.target_height + 30.0)
                if self.check_height_reached([v_fl, v_fr], self.target_height +30.0):
                    self.current_state = State.DOWN_3_REAR_HOVER_LAND
                    
        elif state == State.DOWN_3_REAR_HOVER_LAND:
            self._creep_forward()
            pe_rear_hover = (self._get_v_pe(v_rr) == 0) #后轮光电开关无遮挡，表示后轮悬空放下后轮
            if pe_rear_hover:
                self.current_state = State.DOWN_4_LAND
                    
        elif state == State.DOWN_4_LAND:
            self._stop_chassis()
            self._set_v_wheel_height([v_rl, v_rr], self.target_height +30.0) #放下后轮
            if self.check_height_reached([v_rl, v_rr], self.target_height + 30.0):
                self._creep_forward()
                if self._get_v_distance(3) > 200.0:
                    self.current_state = State.DOWN_5_RECOVER

        elif state == State.DOWN_5_RECOVER:
            self._stop_chassis()
            self.wheel_heights_target = [self.H_INIT] * 4
            if self.check_height_reached([v_fl, v_fr, v_rl, v_rr], self.H_INIT):
                self.get_logger().info("Down step sequence complete.")
                self.current_state = State.IDLE

    def _stop_chassis(self):
        """强制速度为0"""
        self.chassis_cmd_vel.linear.x = 0.0
        self.chassis_cmd_vel.linear.y = 0.0
        self.chassis_cmd_vel.angular.z = 0.0

    def _creep_forward(self):
        """钳位安全速度"""
        if self.current_direction == Direction.FORWARD:
            self.chassis_cmd_vel.linear.x = self.CREEP_SPEED
        elif self.current_direction == Direction.LEFT:
            self.chassis_cmd_vel.linear.y = self.CREEP_SPEED
        elif self.current_direction == Direction.RIGHT:
            self.chassis_cmd_vel.linear.y = -self.CREEP_SPEED

    def _set_v_wheel_height(self, v_indices, height):
        """设置虚拟轮的物理高度"""
        for v_idx in v_indices:
            phys_idx = self.v_wheels_idx[v_idx]
            self.wheel_heights_target[phys_idx] = float(height)

    def _get_v_pe(self, v_idx):
        """获取虚拟轮对应的光电开关状态"""
        phys_pe_idx = self.v_pe_idx[v_idx]
        return self.pe_switches_filtered[phys_pe_idx]
    
    def _get_v_distance(self, v_idx):
        """获取虚拟轮对应的距离传感器值"""
        phys_dist_idx = self.v_distances_idx[v_idx]
        return self.distance_filtered[phys_dist_idx]

def main(args=None):
    rclpy.init(args=args)
    node = SuspensionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()