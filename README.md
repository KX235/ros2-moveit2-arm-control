# ROS 2 + MoveIt 2 六自由度机械臂仿真控制

基于 ROS 2 Humble 与 MoveIt 2 的六自由度机械臂（含平行夹爪）仿真控制工程。从 URDF 建模、MoveIt 配置到 C++ 话题控制接口，全部从零搭建，在 WSL2 + WSLg 环境下完成验证。

> 本项目为学习复刻性质，参考了 [Rachel-eee/ros2-moveit2-arm-control-tutorial](https://github.com/Rachel-eee/ros2-moveit2-arm-control-tutorial) 的教程思路，模型、配置与控制代码均为独立实现，并在其基础上修正了若干配置问题。

## 功能

- 六自由度机械臂 + 两指平行夹爪的 URDF/Xacro 参数化建模
- MoveIt 2 运动规划（`arm` / `gripper` 两个规划组）
- `ros2_control` + `mock_components/GenericSystem` 硬件仿真
- C++ 控制节点，通过三个话题接收外部指令
- 支持关节空间、位姿空间（含笛卡尔直线）与夹爪三类控制

## 环境

| 项目 | 版本 |
|---|---|
| 系统 | Ubuntu 22.04（WSL2 或原生） |
| ROS | Humble |
| MoveIt | 2.5.x |
| 图形 | WSLg（rviz2 需软件渲染，见下） |

依赖安装：

```bash
sudo apt install -y \
  ros-humble-moveit ros-humble-moveit-setup-assistant \
  ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-joint-state-publisher ros-humble-xacro \
  ros-humble-example-interfaces ros-humble-rosidl-default-generators
```

## 目录结构

```
arm_ws/
└── src/
    ├── my_robot_description/       # 机器人描述
    │   └── urdf/
    │       ├── common_properties.xacro   # 尺寸与材质变量
    │       ├── arm.xacro                # 六轴本体
    │       ├── gripper.xacro            # 夹爪（右指 mimic）
    │       └── my_robot.urdf.xacro      # 入口
    ├── my_robot_moveit_config/     # MoveIt Setup Assistant 生成
    │   ├── config/                 # srdf / kinematics / controllers
    │   └── launch/demo.launch.py
    ├── my_robot_commander_cpp/     # C++ 控制节点
    │   └── src/
    │       ├── test_moveit.cpp          # 最小示例：回 home 位姿
    │       └── commander_template.cpp   # 话题控制主程序
    └── my_robot_interfaces/        # 自定义消息
        └── msg/PoseCommand.msg
```

## 快速开始

```bash
cd ~/arm_ws
colcon build --symlink-install
source install/setup.bash

# WSL2 下 rviz2 需要软件渲染，否则 Ogre 会段错误
export LIBGL_ALWAYS_SOFTWARE=1

ros2 launch my_robot_moveit_config demo.launch.py
```

另开终端启动控制节点：

```bash
cd ~/arm_ws && source install/setup.bash
ros2 run my_robot_commander_cpp commander_template
```

### 三种控制指令

```bash
# 1. 关节空间：六个关节角（弧度）
ros2 topic pub --once /joint_command example_interfaces/msg/Float64MultiArray \
  "{data: [0.0, 0.6, 0.9, 0.0, 0.6, 0.0]}"

# 2. 夹爪：true 张开 / false 闭合
ros2 topic pub --once /open_gripper example_interfaces/msg/Bool "{data: true}"

# 3. 位姿：末端位置 + RPY 欧拉角（弧度）+ 是否走笛卡尔直线
ros2 topic pub --once /pose_command my_robot_interfaces/msg/PoseCommand \
  "{x: 0.573, y: 0.0, z: 0.523, roll: 3.142, pitch: 1.041, yaw: -3.142, cartesian_path: false}"
```

## 设计说明

**Xacro 参数化**：连杆长度全部定义为变量（`l1`~`l6`），几何体的 `origin` 用 `${l1/2}`、关节位置用 `${l1}` 引用。改一处尺寸，整条运动链自动跟随，避免手改多个坐标导致连杆错位。

**夹爪 mimic**：右手指通过 `<mimic joint="gripper_left_finger_joint" multiplier="-1.0"/>` 联动，只控制左指，`robot_state_publisher` 自动推导右指位置。

**控制节点并发模型**：`MultiThreadedExecutor` + 原子标志位。回调只做派发，规划与执行放在工作线程；机械臂运动期间收到的新指令直接丢弃而非排队，避免过期目标延迟执行。

**工作空间边界**：模型最大臂展 1.06 m，实测臂展 85% 以内的位姿可稳定求解，接近 97% 时 IK 无解。位姿目标建议先移动到位后用 `tf2_echo` 读出真实位姿，再据此微调。

## 踩坑记录

| 现象 | 原因 | 解决 |
|---|---|---|
| `move_group` 启动即崩，退出码 -6 且无日志 | `joint_limits.yaml` 中 `max_velocity: 1` 被解析成整数 | 全部改写为 `1.0` / `0.0` |
| Plan 成功但 Execute 无反应 | `moveit_controllers.yaml` 缺 `default: true` | 补 `action_ns` 与 `default` 字段 |
| RViz 空白，`XML_ERROR_EMPTY_DOCUMENT` | Setup Assistant 找不到 description 包，或新增文件未重新编译 | `colcon build` 后重新 `source` |
| `Invalid goal state` | 目标位姿超出可达工作空间 | 换可达目标，或放宽关节限位 |
| Setup Assistant 报 `not a valid robot model` | 新增 `.xacro` 文件后未重新 build，install 目录无对应软链 | `--symlink-install` 只对已存在文件建链，新增文件需重编 |

## 已知限制

- 仅仿真，未接入真实机械臂硬件
- 腕部 `joint5` 限位为 ±90°，末端朝下的姿态受限
- 未实现运动取消机制，中断需终止进程

## License

[MIT](LICENSE)
