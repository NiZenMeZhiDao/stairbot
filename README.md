# ws_livox ROS 2 工作空间说明

这个仓库是一套面向主动悬挂底盘的 ROS 2 工作空间，包含 USB 下位机通信、主动悬挂状态机、IMU 和测距传感器等包。各包之间主要通过 ROS 2 topic 解耦，因此调试时可以只启动某几个包，用外部算法或命令行临时发布消息来替代未启动的模块。

## 快速构建

在工作空间根目录构建：

```bash
cd /home/xiexiang/ws_livox
colcon build --symlink-install
source install/setup.bash
```

如果只改了某一个包，可以单独构建：

```bash
colcon build --symlink-install --packages-select active_suspension_control
source install/setup.bash
```

## 包总览

| 包 | 作用 | 主要可执行/launch |
| --- | --- | --- |
| `active_suspension_control` | 主动悬挂和上下台阶状态机。接收外部速度、方向、测距、IMU、下位机状态，输出底盘/悬挂控制数组。 | `suspension_node`，`suspension.launch.py` |
| `ares_usb` | USB 透传桥。自动把 `t0x....` ROS 话题下发到下位机，把下位机 ID `0x....` 回传成 `r0x....` ROS 话题。 | `usb_bridge_node`，`comm_bringup.launch.py` |
| `multi_serial_sensor` | 8 路串口测距模块读取，发布距离数组。 | `multi_serial_node` |
| `wit_ros2_imu` | 维特 IMU 串口驱动，发布标准 IMU 和带 RPY 的自定义 IMU 消息。 | `wit_ros2_imu` |
| `imu_msg` | 自定义消息包，定义 `imu_msg/msg/ImuData`。 | 消息接口包 |

## 核心数据流

常规主动悬挂控制链路：

```text
外部导航/遥控
  -> /cmd_vel, /direction, /target_yaw_deg
active_suspension_control
  <- /sensor_distances        multi_serial_sensor
  <- /r0x0201                 ares_usb 从下位机回传
  <- /imu/ImuDataWithRPY      wit_ros2_imu
  -> /t0x0101_action          给 ares_usb 透传下发
  -> /cmd_vel_chassis         调试用：实际准备下发到底盘的速度
  -> /current_state           当前上下台阶状态机状态
ares_usb
  -> USB 下位机
```

## 主要话题约定

### `active_suspension_control`

订阅：

| Topic | 类型 | 来源/含义 |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 外部导航、遥控或测试节点给出的底盘速度指令。 |
| `/direction` | `std_msgs/msg/Int32` | 外部指定上台阶方向。当前代码中 `0` 前进，`-1` 左，`1` 右；只有 `control_by_sbus=False` 且状态机空闲时生效。 |
| `/sensor_distances` | `std_msgs/msg/Float32MultiArray` | 8 路测距数据，单位按测距模块输出使用，代码按 mm 阈值处理。 |
| `/r0x0201` | `std_msgs/msg/Float32MultiArray` | 下位机状态回传。当前解析至少 12 个 float。 |
| `/imu/ImuDataWithRPY` | `imu_msg/msg/ImuData` | IMU 姿态，`yaw` 单位为度。 |
| `/target_yaw_deg` | `std_msgs/msg/Float32` | 目标相对 yaw，单位为度。用于 yaw 闭环修正。 |

`/r0x0201` 数组格式：

```text
[0..3]   PE_0..PE_3 光电开关状态
[4..7]   H_0..H_3 四个主动轮当前高度
[8]      下位机/遥控回传的 raw_cmd_vel.linear.x
[9]      下位机/遥控回传的 raw_cmd_vel.linear.y
[10]     下位机/遥控回传的 raw_cmd_vel.angular.z
[11]     SBUS 方向量，>0 右，<0 左，=0 前
```

发布：

| Topic | 类型 | 含义 |
| --- | --- | --- |
| `/t0x0101_action` | `std_msgs/msg/Float32MultiArray` | 下发给下位机的动作数组，会被 `ares_usb` 透传为 USB ID `0x0101`。 |
| `/cmd_vel_chassis` | `geometry_msgs/msg/Twist` | 状态机接管、限速和 yaw 修正后的实际底盘速度，主要用于调试观察。 |
| `/current_state` | `std_msgs/msg/Int32` | 当前状态机状态值。`0` 为 `IDLE`，`10..17` 为上台阶过程，`20..23` 为下台阶过程。 |

`/t0x0101_action` 数组格式：

```text
[0..3]  四个主动轮目标高度
[4]     chassis linear.x
[5]     chassis linear.y
[6]     chassis angular.z
```

### `ares_usb`

`ares_usb` 是动态透传节点，不需要为每种命令手写 topic。规则如下：

| 方向 | Topic/ID 规则 | 类型 |
| --- | --- | --- |
| ROS -> 下位机 | 任意名为 `/t0xXXXX...` 的 topic 会被自动订阅，`XXXX` 解析为十六进制 ID。例：`/t0x0101_action` -> USB ID `0x0101`。 | `std_msgs/msg/Float32MultiArray` |
| 下位机 -> ROS | 下位机回传 ID `0xXXXX` 后，节点自动创建 `/r0xXXXX` topic。例：ID `0x0201` -> `/r0x0201`。 | `std_msgs/msg/Float32MultiArray` |

注意：透传 payload 按 float32 数组直接打包/解包，下位机和 ROS 端必须保持数组长度、顺序和单位一致。

### `multi_serial_sensor`

发布：

| Topic | 类型 | 含义 |
| --- | --- | --- |
| `/sensor_distances` | `std_msgs/msg/Float32MultiArray` | 8 路距离值。串口设备默认为 `/dev/ttyCH9344USB0` 到 `/dev/ttyCH9344USB7`，波特率 `230400`。 |

节点会以 100 Hz 发布。某一路无有效数据时，该路会发布 `NaN`。

### `wit_ros2_imu` 和 `imu_msg`

`imu_msg/msg/ImuData` 包含：

```text
std_msgs/Header header
sensor_msgs/Imu imu
float64 roll
float64 pitch
float64 yaw
```

发布：

| Topic | 类型 | 含义 |
| --- | --- | --- |
| `/imu/data` | `sensor_msgs/msg/Imu` | 标准 IMU 消息。 |
| `/imu/ImuDataWithRPY` | `imu_msg/msg/ImuData` | 标准 IMU 加 roll/pitch/yaw。当前驱动发布的 RPY 单位是度。 |

默认 launch 参数在 `active_suspension_control/launch/suspension.launch.py` 中：

```text
port=/dev/imu_usb
baudrate=115200
protocol=TTL_STD
modbusID=0x50
```

## 启动方式

### 一键启动主动悬挂相关节点

```bash
source /home/xiexiang/ws_livox/install/setup.bash
ros2 launch active_suspension_control suspension.launch.py
```

该 launch 会启动：

- `wit_ros2_imu`
- `active_suspension_control/suspension_node`
- `ares_usb/usb_bridge_node`
- `multi_serial_sensor/multi_serial_node`

适合实车主动悬挂联调。

### 只启动 USB 通信包

```bash
source /home/xiexiang/ws_livox/install/setup.bash
ros2 launch ares_usb comm_bringup.launch.py
```

适合验证 ROS 和下位机 USB 通信是否正常。启动后可以直接从命令行或外部算法包发布 `/t0x....` 指令。

例如只发底盘/悬挂动作，不启动 IMU、测距或悬挂状态机：

```bash
ros2 topic pub /t0x0101_action std_msgs/msg/Float32MultiArray \
"{data: [30.0, 30.0, 30.0, 30.0, 0.2, 0.0, 0.0]}" -r 20
```

这会被 `ares_usb` 转成 USB ID `0x0101` 下发。数组含义为四个轮目标高度加底盘速度。

查看下位机状态回传：

```bash
ros2 topic echo /r0x0201
```

### 只启动主动悬挂状态机，不启动真实传感器

```bash
ros2 run active_suspension_control suspension_node
```

这种方式适合用假数据测试状态机。需要额外发布它依赖的输入，例如：

```bash
ros2 topic pub /sensor_distances std_msgs/msg/Float32MultiArray \
"{data: [300.0, 300.0, 300.0, 300.0, 300.0, 300.0, 300.0, 300.0]}" -r 50

ros2 topic pub /r0x0201 std_msgs/msg/Float32MultiArray \
"{data: [0, 0, 0, 0, 30, 30, 30, 30, 0.0, 0.0, 0.0, 0.0]}" -r 50

ros2 topic pub /imu/ImuDataWithRPY imu_msg/msg/ImuData \
"{roll: 0.0, pitch: 0.0, yaw: 0.0}" -r 50
```

然后发布速度：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 20
```

观察状态机输出：

```bash
ros2 topic echo /t0x0101_action
ros2 topic echo /cmd_vel_chassis
ros2 topic echo /current_state
```

### 只启动 USB，让外部导航包直接跑底盘

导航算法成员如果只想验证底盘运动，不需要启动 IMU、测距和主动悬挂状态机，可以只启动 `ares_usb`，然后外部包发布 `/t0x0101_action`。

外部包需要发布：

```text
topic: /t0x0101_action
type:  std_msgs/msg/Float32MultiArray
data:  [h0, h1, h2, h3, vx, vy, wz]
```

建议调试时先把高度固定在常规运动高度，例如 `[30, 30, 30, 30]`，只改变 `vx/vy/wz`：

```text
[30.0, 30.0, 30.0, 30.0, vx, vy, wz]
```

这样可以绕开上/下台阶状态机和各传感器，只验证外部导航算法到下位机的底盘速度链路。

## 调试建议

查看当前 topic 和类型：

```bash
ros2 topic list -t
```

确认 USB 透传是否发现下发通道：

```bash
ros2 topic pub /t0x0101_action std_msgs/msg/Float32MultiArray \
"{data: [30, 30, 30, 30, 0, 0, 0]}" -1
```

`usb_bridge_node` 日志中应出现类似 `ROS Topic -> USB ID: 0x0101` 的信息。

确认主动悬挂有没有接管速度：

```bash
ros2 topic echo /cmd_vel_chassis
ros2 topic echo /current_state
```

当 `current_state=0` 时，通常会透传外部 `/cmd_vel`；进入 `10..17` 或 `20..23` 后，状态机会按上下台阶过程接管速度和轮高。

## 重要注意事项

- `ares_usb` 会订阅所有符合 `t0x` 命名规则且类型为 `Float32MultiArray` 的 topic；命名时避免误触发。
- `active_suspension_control` 当前默认 `control_by_sbus=True`，方向主要从 `/r0x0201` 的第 12 个元素解析；如果要完全用 `/direction` 控制，需要改代码中的 `control_by_sbus`。
- `/cmd_vel_chassis` 只是 ROS 侧调试输出，真正下发到底盘的是 `/t0x0101_action` 经 `ares_usb` 透传后的 USB 数据。
- 在只启动 USB 直接跑底盘时，不会有主动悬挂状态机的安全接管逻辑；请先低速、小范围测试。
