# 小米杯四足机器人“荒野寻宝”项目架构设计文档

## 1. 项目背景与核心挑战

本项目针对 2026 年全国大学生计算机系统能力大赛智能系统创新设计赛（小米杯）设计。比赛要求机器狗在规定时间（15 分钟）内，在复杂的“荒野寻宝”赛道中完成探路、寻物、交互等一系列动作。

全场分为 6 个赛段：石径探路、荒野寻珠、曲道冲锋、深隧寻珍、孤梁稳渡、撷金建功。设备必须采用全程自主的方式完成比赛，程序启动后交由现场工作人员保管，不再允许触碰电脑，不允许再次启动或进行远程人为操控。

**核心架构设计原则：**

- **纯 Python 极速迭代：** 抛弃繁琐的 ROS 2 编译流程，无需 `colcon build`，代码修改后即刻运行，提升赛场调参效率。
- **硬件抽象隔离（HAL Isolation）：** 将底层通信环境与上层算法彻底解耦，确保视觉和控制算法可以在无机器狗硬件的普通电脑上进行独立测试。
- **扁平化高内聚：** 核心逻辑全部收敛于 `core` 目录下，按职责划分为通信、视觉、运动和主控四大模块。

---

## 2. 当前项目目录结构

```text
Mvr/
├── AGENTS.md                       # 项目架构与协作约束说明
├── main.py                         # 当前测试入口（桥接层联调/烟雾测试）
├── config/
│   └── game_params.yaml            # 全局参数配置
├── core/
│   ├── __init__.py
│   ├── cyberdog_bridge.py          # 硬件通信枢纽
│   ├── vision_processor.py         # 视觉算法引擎（待实现）
│   ├── action_manager.py           # 运动控制核心（待实现）
│   └── game_controller.py          # 全局状态机（待实现）
├── scripts/
│   ├── start_game.sh               # 赛场启动脚本（待实现）
│   ├── test_vision.sh              # 视觉模块测试脚本（待实现）
│   └── test_motion.sh              # 运动模块测试脚本（待实现）
└── utils/                          # 现有底层消息定义与辅助文件，不要随意修改
    ├── file_recv_lcmt.py
    ├── file_send_lcmt.py
    ├── localization_lcmt.py
    ├── robot_control_cmd_lcmt.py
    ├── robot_control_response_lcmt.py
    └── simulator_lcmt.py
```

说明：
- 当前实际入口文件是 `main.py`，用于联调 `CyberdogBridge`。
- `main_node.py` 作为最终比赛入口可以后续再恢复或新增，但当前仓库中并不存在该文件。
- `utils/` 目录目前存放项目依赖的 LCM 消息定义，不作为本轮重构对象。

---

## 3. 核心模块详细设计

### 3.1 `core/cyberdog_bridge.py`（硬件通信枢纽）

**定位：**
系统唯一直接接触机器人底层通信环境的模块，统一桥接 `LCM`、`ROS 2`、相机数据、高层运控与底层电机控制。

**当前职责边界：**

- 接收机器狗本体相关状态：
  - 高层运动状态
  - 里程计状态
  - IMU 状态
  - 关节/电机状态
  - 摄像机图像帧
- 发送两类控制命令：
  - 高层运动控制命令
  - 底层电机控制命令
- 维护线程安全缓存和命令心跳。
- 负责控制模式互斥，避免高层控制与底层关节控制冲突。

**当前已实现内容：**

- 高层 `LCM` 通道：
  - `robot_control_cmd`
  - `robot_control_response`
  - `global_to_robot`
- ROS 2 图像订阅入口：默认 `/image`
- 缓存接口：
  - `get_motion_state()`
  - `get_odometry()`
  - `get_imu_data()`
  - `get_joint_state()`
  - `get_camera_frame()`
  - `get_current_pose()`
- 高层控制接口：
  - `send_velocity()`
  - `send_pose()`
  - `send_action()`
  - `recovery_stand()`
  - `pure_damper()`
- 底层电机控制接口骨架：
  - `enter_motor_control_mode()`
  - `exit_motor_control_mode()`
  - `send_motor_command()`
  - `send_zero_motor_command()`

**当前未完全落地的部分：**

- 底层电机真实消息发送器与状态接收器，当前通过可注入回调预留：
  - `motor_command_sender`
  - `motor_state_receiver`
- 语音、灯效、TOF、超声、电池等外围能力尚未接入。

**设计约束：**

- 上层模块不允许直接操作 `LCM` 或 `ROS 2`。
- 高层运动控制与底层电机控制必须互斥。
- 不要把视觉算法、状态机和比赛策略写进桥接层。

### 3.2 `core/vision_processor.py`（视觉算法引擎）

**定位：**
系统的“视觉皮层”，纯算法模块。

**职责：**

- 接收图像矩阵并进行处理。
- 负责目标检测、目标定位、赛道线提取等纯视觉逻辑。
- 不直接接触机器人底层话题或消息类型。

**预期接口：**

- `detect_targets(image)`
- `calculate_lane_offset(image)`

### 3.3 `core/action_manager.py`（复合动作管家）

**定位：**
系统的“小脑”。

**职责：**

- 维护运动控制相关逻辑，例如 PID、速度限幅、平滑控制。
- 将视觉或状态机输出的高维任务指令转换为对 `CyberdogBridge` 的调用。
- 封装过杆、撞球、稳定循迹等复合动作。

**预期接口：**

- `follow_lane_smoothly()`
- `execute_crash_target()`
- `pass_under_bar()`

### 3.4 `core/game_controller.py`（比赛主控大脑）

**定位：**
系统的“总指挥”。

**职责：**

- 管理当前赛段状态。
- 高频读取 `CyberdogBridge` 的传感器缓存。
- 按需调用 `VisionProcessor` 和 `ActionManager`。
- 完成赛段切换、行为决策和主循环调度。

**预期接口：**

- `run_main_loop()`
- `_handle_stage_X()`

---

## 4. 当前开发流转机制

1. `main.py` 作为当前调试入口，先对 `CyberdogBridge` 做联调和烟雾测试。
2. `CyberdogBridge` 持续接收图像、运动状态、里程计及底层状态缓存。
3. 后续 `GameController` 接入后，将统一从桥接层提取状态。
4. 后续 `VisionProcessor` 只处理图像，不感知底层通信。
5. 后续 `ActionManager` 只调用桥接层接口，不直接发底层消息。

---

## 5. 当前测试入口说明

当前 `main.py` 已用于桥接层基础测试。

可用测试方式：

```bash
python3 main.py
```

作用：
- 启动 `CyberdogBridge`
- 周期打印运动状态、里程计、相机状态、关节错误标志
- 不主动发动作

可选动作测试：

```bash
python3 main.py --action stand
python3 main.py --action damper
python3 main.py --action forward --vx 0.1 --duration 2
```

说明：
- `stand`：发送恢复站立测试指令
- `damper`：发送高阻尼测试指令
- `forward`：发送低速前进测试指令，并在持续时间后自动置零

当前建议联调顺序：

1. 先运行 `python3 main.py`
2. 确认能收到 `motion_state`
3. 确认能收到 `odometry`
4. 确认相机是否有帧
5. 最后再尝试最简单的高层动作测试

底层电机控制暂不建议直接实机测试，除非已经接好真实的底层 SDK 适配器。

---

## 6. 协作约束

- 不要随意修改 `utils/` 目录中的现有消息定义文件。
- 优先在 `core/` 内完成架构搭建与模块实现。
- 先打通桥接层，再接运动层，再接主控层，最后再补视觉与完整比赛流程。
- 任何新增的底层电机真实适配代码，都应通过 `CyberdogBridge` 注入，不应反向污染上层模块。
