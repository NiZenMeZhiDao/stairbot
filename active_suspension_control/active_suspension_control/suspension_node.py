#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
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
        self.distances_raw = [0.0] * 6
        self.pe_switches_raw = [0] * 4
        self.wheel_heights_current = [0.0] * 4  # [FL, FR, RL, RR]
        
        # 滤波后状态
        self.distance_filtered = [0.0] * 6
        self.pe_switches_filtered = [0] * 4
        
        # 输出控制状态
        self.wheel_heights_target = [self.H_INIT] * 4
        self.chassis_cmd_vel = Twist()
        self.v_distances = [0.0] * 6

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
        elif msg.linear.x < 0:
            self.is_reversing = True # 触发倒退逻辑
        elif msg.linear.y > 0:
            self.current_direction = Direction.LEFT
            self.is_reversing = False
        elif msg.linear.y < 0:
            self.current_direction = Direction.RIGHT
            self.is_reversing = False

    def dist_cb(self, msg):
        if len(msg.data) >= 6:
            for i in range(6):
                # 滑动平均滤波剔除噪点
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
            if self.raw_cmd_vel.linear.x > 0:
                self.current_direction = Direction.FORWARD
                self.is_reversing = False
            elif self.raw_cmd_vel.linear.x < 0:
                self.is_reversing = True # 触发倒退逻辑
            elif self.raw_cmd_vel.linear.y > 0:
                self.current_direction = Direction.LEFT
                self.is_reversing = False
            elif self.raw_cmd_vel.linear.y < 0:
                self.current_direction = Direction.RIGHT
                self.is_reversing = False
        
    # ================= 核心映射与控制 =================
    def update_virtual_mapping(self):
        """根据行驶方向，将物理轮/传感器映射为虚拟的 前左(VFL), 前右(VFR), 后左(VRL), 后右(VRR)"""
        if self.current_direction == Direction.FORWARD:
            self.v_wheels_idx = [2, 1, 0, 3] # [FL, FR, RL, RR]
            self.v_pe_idx = [0, 1, 3, 2]     # 假设 0,1为前，2,3为后
            self.v_distances = self.distance_filtered # 前后距离传感器不变
        elif self.current_direction == Direction.LEFT:
            # 向左移动时，物理的RL变成虚拟的FL，物理的FL变成虚拟的FR...以此类推
            self.v_wheels_idx = [0, 2, 3, 1] 
            # 复用风车阵列光电开关 (具体索引视硬件连线为准，这里做概念演示)
            self.v_pe_idx = [1, 2, 0, 3]     
            self.v_distances = [self.distance_filtered[2], self.distance_filtered[3], self.distance_filtered[0], self.distance_filtered[1], self.distance_filtered[4], self.distance_filtered[5]] # 左右交换
        elif self.current_direction == Direction.RIGHT:
            self.v_wheels_idx = [1, 3, 2, 0]
            self.v_pe_idx = [3, 0, 2, 1]
            self.v_distances = [self.distance_filtered[4], self.distance_filtered[5], self.distance_filtered[0], self.distance_filtered[1], self.distance_filtered[2], self.distance_filtered[3]] # 左右交换

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
        
        v_fl, v_fr, v_rl, v_rr = 0, 1, 2, 3 # 虚拟索引常量
        
        # 状态机逻辑执行
        self.execute_state_machine(v_fl, v_fr, v_rl, v_rr)
        
        # 发布高度与速度
        h_msg = Float32MultiArray(data=self.wheel_heights_target)
        self.pub_wheel_heights.publish(h_msg)
        self.pub_chassis_vel.publish(self.chassis_cmd_vel)

    def execute_state_machine(self, v_fl, v_fr, v_rl, v_rr):
        """核心状态机"""
        state = self.current_state
        
        # 速度仲裁：默认透传，如果进入特殊状态会被覆盖
        self.chassis_cmd_vel.linear.x = self.raw_cmd_vel.linear.x
        self.chassis_cmd_vel.linear.y = self.raw_cmd_vel.linear.y
        self.chassis_cmd_vel.angular.z = self.raw_cmd_vel.angular.z

        # 倒退逻辑处理 (简易实现：当探测到外部下发反向速度时，强制回退上一状态)
        if self.is_reversing and state != State.IDLE:
            # 真实场景中需要建立反向状态机或回退映射，此处简单演示：速度变反，限制速度并尝试收起/放下对应的轮子。
            self.chassis_cmd_vel.linear.x = math.copysign(self.CREEP_SPEED, self.raw_cmd_vel.linear.x)
            self.get_logger().warn("Reversing during step sequence! Proceeding with caution.")

        if state == State.IDLE:
            # 测距传感器前下（假设索引0）检测到台阶
            front_dist = self.v_distances[1]
            if front_dist < 200:  
                self.current_state = State.UP_1_PREPARE
            if self.v_distances[0] > 200:
                self.current_state = State.DOWN_1_PREPARE

        # ================= 上台阶逻辑 =================
        elif state == State.UP_1_PREPARE:
            
            if self.v_distances[0] > 200:
                if self.v_distances[1] < 250:
                    self.target_height = self.H_LIFT_HIGH
                    self.wheel_heights_target = [self.target_height] * 4
                    if self.check_height_reached([v_fl, v_fr, v_rl, v_rr], self.target_height):
                        self.current_state = State.UP_2_LIFT
                elif self.v_distances[1] > 200:
                    self.target_height = self.H_LIFT_LOW
                    self.wheel_heights_target = [self.target_height] * 4
                    if self.check_height_reached([v_fl, v_fr, v_rl, v_rr], self.target_height):
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
            if self.v_distances[0] < 80.0:  # 已经稳稳搭在平台上
                self.current_state = State.UP_4_RETRACT_FRONT

        elif state == State.UP_4_RETRACT_FRONT:
            # 收起前主动轮，等待电机到位
            self._stop_chassis()
            self._set_v_wheel_height([v_fl, v_fr], 0.0) # 0为与从动轮齐平
            if self.check_height_reached([v_fl, v_fr], 0.0):
                self.current_state = State.UP_5_FRONT_LAND

        elif state == State.UP_5_FRONT_LAND:
            self._creep_forward()
            # 检查虚拟前轮的下视光电开关 (无遮挡表示处于平台上空)
            pe_front_clear = (self._get_v_pe(v_fl) == 1) #1有遮挡
            if pe_front_clear:
                self._set_v_wheel_height([v_fl, v_fr], 30.0) # 降下前主动轮
                self._set_v_wheel_height([v_rl, v_rr], self.target_height + 30.0) 
                self.current_state = State.UP_6_SIDE_DOCK_RETRACT_REAR

        elif state == State.UP_6_SIDE_DOCK_RETRACT_REAR:
            self._creep_forward()
            # 假设侧方光电开关(或距离传感器)被遮挡，表明车体中部搭上
            # 此时立即收起后轮
            if self._get_v_pe(v_rl) == 1: # 后轮光电被遮挡
                self._set_v_wheel_height([v_rl, v_rr], 0.0)
                if self.check_height_reached([v_rl, v_rr], 0.0):
                    self.current_state = State.UP_7_REAR_LAND

        elif state == State.UP_7_REAR_LAND:
            self._creep_forward()
            if self._get_v_pe(v_rr) == 1: # 后轮光电被遮挡
                self._set_v_wheel_height([v_rl, v_rr], 30.0)
                if self.check_height_reached([v_rl, v_rr], 30.0):
                    self.current_state = State.UP_8_RECOVER

        elif state == State.UP_8_RECOVER:
            self._stop_chassis()
            self.wheel_heights_target = [self.H_INIT] * 4 # 恢复常规行驶姿态
            if self.check_height_reached([v_fl, v_fr, v_rl, v_rr], self.H_INIT):
                self.get_logger().info("Up step sequence complete.")
                self.current_state = State.IDLE

        # ================= 下台阶逻辑 =================
        # 下台阶逻辑按描述简化实现
        elif state == State.DOWN_1_PREPARE:
            self._stop_chassis()
            if self.v_distances[0] > 380:
                self.target_height = self.H_LIFT_HIGH # 反向高度
            elif self.v_distances[0] > 180:
                self.target_height = self.H_LIFT_LOW
            self.current_state = State.DOWN_2_FRONT_HOVER_LAND
            
        elif state == State.DOWN_2_FRONT_HOVER_LAND:
            self._creep_forward()
            pe_front_hover = (self._get_v_pe(v_fl) == 0)
            if pe_front_hover:
                self._set_v_wheel_height([v_fl, v_fr], self.target_height + 30.0)
                if self.check_height_reached([v_fl, v_fr], self.target_height +30.0):
                    self.current_state = State.DOWN_3_REAR_HOVER_LAND
                    
        elif state == State.DOWN_3_REAR_HOVER_LAND:
            self._creep_forward()
            pe_rear_hover = (self._get_v_pe(v_rr) == 0)
            if pe_rear_hover:
                self.current_state = State.DOWN_4_LAND
                    
        elif state == State.DOWN_4_LAND:
            self._stop_chassis()
            self._set_v_wheel_height([v_rl, v_rr], self.target_height +30.0)
            if self.check_height_reached([v_rl, v_rr], self.target_height + 30.0):
                self._creep_forward()
                if self.v_distances[0] > 200.0:
                    self.current_state = State.DOWN_5_RECOVER

        elif state == State.DOWN_5_RECOVER:
            self._stop_chassis()
            self.wheel_heights_target = [self.H_INIT] * 4
            if self.check_height_reached([v_fl, v_fr, v_rl, v_rr], self.H_INIT):
                self.get_logger().info("Down step sequence complete.")
                self.current_state = State.IDLE


    # ================= 辅助函数 =================
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
        self.chassis_cmd_vel.angular.z = 0.0

    def _set_v_wheel_height(self, v_indices, height):
        """设置虚拟轮的物理高度"""
        for v_idx in v_indices:
            phys_idx = self.v_wheels_idx[v_idx]
            self.wheel_heights_target[phys_idx] = float(height)

    def _get_v_pe(self, v_idx):
        """获取虚拟轮对应的光电开关状态"""
        phys_pe_idx = self.v_pe_idx[v_idx]
        return self.pe_switches_filtered[phys_pe_idx]

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