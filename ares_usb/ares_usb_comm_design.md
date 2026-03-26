# ARES USB 通信包设计文档

*最后更新: 2025-07-10*

## 1. 设计目标
1. 复用 `ares_comm` 中成熟的 USB Bulk 协议实现思路，新增对 **9 个电机角度 / 速度** 以及 **IMU 六轴数据** 的传输支持。
2. 提供 **上位机 (ROS 2)** 与 **下位机 (STM32)** 间的双向可靠通信，满足 ≤ 3 ms 心跳、64 Byte 满速同步帧的实时性要求。
3. 遵循 ROS 2 C++/Python 规则，保持可维护、易移植、易扩展。

---

## 2. 协议层设计
### 2.1 帧类型继承
沿用 `ares_comm/ARES_bulk_library` 已定义的 4 类帧头：
| 帧头 | 说明                 | 向    | 典型用途             |
|------|----------------------|-------|----------------------|
| `0xCAFE` | ExecFrame          | Host → MCU | 单指令 / 参数设置       |
| `0xC0DE` | ExecReplyFrame     | MCU → Host | 指令返回值 / ACK       |
| `0x5A5A` | SyncFrame          | 双向      | **实时数据流**           |
| `0xCADE` | ErrorFrame         | 双向      | 错误 / 心跳           |

### 2.2 数据 ID 定义 (SyncFrame)
| DataID | 方向 | 数据含义 | Payload 字节布局 |
|--------|------|----------|-----------------|
| `0x0101` | Host → MCU | **MotorCmd** 电机角度 (9×float32) | `float32 angle[9]` = 36 B |
| `0x0102` | Host → MCU | **TorqueCmd** 前馈力矩 (9×float32) | `float32 tau_ff[9]` = 36 B |
| `0x0201` | MCU → Host | **MotorState-A** 反馈角度 (9×float32) | 同上 |
| `0x0202` | MCU → Host | **MotorState-S** 反馈速度 (9×float32) | 同上 |
| `0x0203` | MCU → Host | **MotorState-T** 反馈实际力矩 (9×float32) | `float32 tau[9]` = 36 B |
| `0x0301` | MCU → Host | **IMU** 4×Quat + 3×Gyro | `float quat[4]` + `float gyro[3]` = 28 B |

> 注：
> 1. **混合字节序**：`DataID` 本身采用 **大端序 (Big-Endian)**, 而 Payload 中的所有浮点数 (float32) 均采用 **小端序 (Little-Endian)**。
> 2. 单帧即可完整承载 9×float32 (36 B)。若未来电机数增多，可使用递增 DataID 扩展。但最大不能超过60B

### 2.2.1 电机索引映射
`MotorCmd` (action) 和 `MotorState` (joint_state) 消息中的 9-float 数组遵循以下索引到电机的映射：

| 索引 | 关节名称 |
|------|--------------------|
| 0 | `FL_thigh_joint_i` |
| 1 | `FL_thigh_joint_o` |
| 2 | `FR_thigh_joint_i` |
| 3 | `FR_thigh_joint_o` |
| 4 | `waist_joint`      |
| 5 | `RL_thigh_joint_i` |
| 6 | `RL_thigh_joint_o` |
| 7 | `RR_thigh_joint_i` |
| 8 | `RR_thigh_joint_o` |

> **坐标系约定**:
> * **髋关节 (hip)**: 正值表示腿向外侧摆开 (abduction)。
> * **膝关节 (knee)**: 正值表示腿向后弯曲 (flexion)。
> * **腰部关节 (waist)**: 从机器人后方俯视，正值表示顺时针旋转。


### 2.3 CRC 与心跳
* CRC-8 (多项式 0x07)，计算范围 **不含** Head 与 Tail。
* 每 `3 ms` MCU 发送 `ErrorFrame` 心跳 (`request_id=0xFF, error_code=0x0100`). Host 复用现有 `heartbeat_loop()` 定时发送同帧。

### 2.4 多频率帧调度策略
不同 `DataID` 对应的数据刷新率不同，下位机需按固定周期将帧写入 USB FIFO，上位机按到达顺序解析。

| DataID | 期望周期 | 典型频率 | 说明 |
|--------|----------|---------|------|
| `0x0101` (MotorCmd) | 按需 | ≤ 100 Hz | 由上位机主动发送，依据控制循环调度 |
| `0x0102` (TorqueCmd) | 按需 | ≤ 100 Hz | 与 `MotorCmd` 同周期发送，MIT 控制模式的前馈力矩 |
| `0x0201/0x0202/0x0203` (MotorState) | 5 ms | 200 Hz | STM32 实时回传，角度/速度/力矩，优先级 **高** |
| `0x0301` (IMU7) | 2 ms | 500 Hz | STM32 回传，优先级 **最高** |
| `ErrorFrame` (心跳) | 3 ms | ≈333 Hz | 双向，检测链路健康 |


<!-- #### 上位机接收侧
1. **时间戳队列**：`usb_read_loop()` 到帧即刻推送至无锁环形队列，随后由 `parser_thread` 按 DataID 发布 ROS 话题。
2. **丢包检测**：维护上一帧到达 `std::chrono::steady_clock::time_point`，若超过 2× 周期触发 `WARN` 日志。
3. **回压保护**：若解析线程滞后，读线程仍写入环形缓冲，当即用最新帧覆盖旧帧（采用 2^n 大小指数环）。 -->

---

## 3. ROS 2 接口层设计
### 3.1 消息
使用 ROS 2 标准消息类型：
- `sensor_msgs/msg/JointState`: 
  - **position** → 角度 (rad)
  - **velocity** → 速度 (rad/s)
  - **effort**   → 力矩 (N·m)
- `sensor_msgs/msg/JointState` (命名 `/torque_cmd`)：仅使用 **effort** 字段传输 MIT 控制模式的前馈力矩。
- `geometry_msgs/msg/Quaternion`: 用于发布 IMU 的姿态（方向）。
- `geometry_msgs/msg/Vector3`: 用于发布 IMU 的角速度。

### 3.2 话题 / 服务
| 名称 | 类型 | 方向 | 描述 |
|------|------|------|------|
| `/action` | sensor_msgs/JointState | App → usb_node | 发送目标角度 (position) & 前馈力矩 (effort，可选) |
| `/torque_cmd` | sensor_msgs/JointState | App → usb_node | **可选**：仅发送前馈力矩 (effort) |
| `/joint_state` | sensor_msgs/JointState | usb_node → App | 反馈电机状态 (position, velocity, effort) |
| `/orient` | geometry_msgs/Quaternion | usb_node → App | IMU 姿态（当前为默认值） |
| `/angvel` | geometry_msgs/Vector3 | usb_node → App | IMU 角速度 |

> 实现上，`usb_bridge_node` 在 MIT 模式下会同时打包角度 (`0x0101`) 与力矩 (`0x0102`) 两帧发送，或者在简化模式下仅发送角度帧。

### 3.3 节点
1. **usb_bridge_node (C++)**  
   * 封装 `Protocol` 类；订阅 `/action`，打包 `JointState.position` 为 `SyncFrame(0x0101)` 发送。
   * 在 `sync_cb_` 中根据 `DataID` 分发并发布 `/joint_state`, `/orient`, `/angvel` 消息。
   * 维护心跳与错误回调。

2. **diagnostic_node (Python)** *(可选)*  
   * 订阅 `/motor_state`、`/imu/data_raw`；发布 `/diagnostics`。

---

<!-- ## 4. 目录结构
```
ares_usb_comm/
├── ARES_bulk_library/         # 复用/重命名, 仅新增 DataID/FuncID
├── include/ares_usb_comm/
│   ├── protocol.hpp
│   └── motor_types.hpp
├── msg/
│   ├── MotorCmd.msg
│   └── MotorState.msg
├── src/
│   ├── usb_bridge_node.cpp
│   └── ...
├── launch/
│   └── bringup.launch.py
├── CMakeLists.txt
└── package.xml
``` -->

---

## 5. 上下位机数据流程
```mermaid
sequenceDiagram
    participant control
    participant usb_node
    participant STM32

    control->>usb_node: publish JointState (topic /action)
    usb_node->>STM32: SyncFrame 0x0101 (angles)
    STM32->>usb_node: SyncFrame 0x0201 (angle feedback)
    STM32-->>usb_node: SyncFrame 0x0202 (speed feedback)
    STM32-->>usb_node: SyncFrame 0x0301 (IMU Quat+Gyro)
    usb_node->>control: publish JointState (topic /joint_state)
    usb_node->>control: publish Quaternion (topic /orient)
    usb_node->>control: publish Vector3 (topic /angvel)
    Note over usb_node,STM32: ErrorFrame 心跳每 3 ms 双向发送
```

<!-- ---

## 6. 关键实现要点
1. **Payload 打包解包**：
   * 使用 `std::array<float, 9>` 直接 `memcpy()` 至 `SyncFrame::data` 发送角度。
2. **线程安全**：USB 读写使用互斥锁 `usb_mutex_` 与 `std::atomic<bool> running_`。**读写分别位于独立线程，实现“伪”全双工**。
3. **参数化**：`DataID`、缩放系数、心跳周期通过 ROS `params` 配置，便于实验调优。
4. **兼容旧协议**：保留原 `ExecFrame` 流程及心跳，实现平滑迁移。
5. **多频率调度**：实现环形缓冲 + 优先级仲裁，保证高频 IMU 帧不被低频内容饿死。

--- -->

<!-- ---

## 7. USB 全双工特性与注意事项
1. **Endpoint 独立**：Bulk `EP_OUT (0x01)` 与 `EP_IN (0x81)` 在硬件上分离，理论支持全双工。Host 每 125 µs 可调度多个事务，满足上表带宽（< 1 MBit/s）。
2. **Host 轮询模型**：USB 读依赖主机发起，故需持续调用 `libusb_bulk_transfer()` 或使用异步 `libusb_submit_transfer()` 保持 IN 端带宽。
3. **MTU 对齐**：保持每帧 ≤ 64 B，避免分段；若批量写需控制在 64×N 字节。
4. **突发写入**：上位机短时间内发送大量 MotorCmd 时，需睡眠 `0.1 ms` 间隔，避免填满 Device FIFO。
5. **延迟统计**：在心跳帧中复用 `error_code` 低字节上报 MCU->Host 单向延迟，以供调参。

---  -->