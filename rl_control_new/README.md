# 天工人形机器人强化学习控制库 (rl_control_new)

这是一个基于ROS2的人形机器人强化学习控制库，用于控制天工系列人形机器人。该库使用强化学习算法实现机器人运动控制，支持仿真和真实机器人环境。

## 目录结构

```
rl_control_new/
├── CMakeLists.txt              # CMake编译配置文件
├── package.xml                 # ROS2包描述文件
├── rlctrlnew_plugin.xml        # 插件描述文件
├── README.md                   # 本说明文件
├── config/                     # 配置文件目录
│   ├── policy/                 # 策略网络模型文件
│   │   └── policy.xml          # 策略网络模型
│   └── tg22_config.yaml        # 机器人配置文件
├── include/                    # 头文件目录
│   └── broccoli/               # Broccoli库头文件
├── launch/                     # ROS2启动文件
│   └── rl.launch.py            # 启动脚本
├── src/                        # 源代码目录
│   ├── common/                 # 通用工具代码
│   │   └── util/               # 工具类
│   └── plugins/                # 插件目录
│       ├── JoyStick/           # 手柄控制插件
│       ├── p2s/                # 串并联转换库
│       ├── rl_control_new/     # 主控制插件
│       │   ├── include/        # 头文件目录
│       │   │   ├── RLControlNewPlugin.h  # 主控制逻辑接口
│       │   │   └── bodyIdMap.h # 机器人关节ID映射
│       │   └── src/            # 源文件目录
│       │       └── RLControlNewPlugin.cpp  # 主控制逻辑实现
│       └── x_humanoid_rl_sdk/  # 机器人SDK库
└── README.md                   # 说明文档
```

## 依赖项

### 系统依赖
- Ubuntu 22.04 LTS
- ROS2 Humble
- C++17 编译器
- CMake 3.8 或更高版本

### ROS2包依赖
- rclcpp
- rclpy
- std_msgs
- sensor_msgs
- bodyctrl_msgs
- pluginlib
- eigen3_cmake_module
- Eigen3
- yaml-cpp
- ament_index_cpp
- joy (用于手柄控制)

### 第三方库依赖
- spdlog (日志库)
- fmt (格式化库)
- x_humanoid_rl_sdk (机器人强化学习SDK)
- OpenVINO (用于神经网络推理，默认)


## 编译说明

### 1. 环境准备

确保已安装ROS2和基本依赖：

```bash
# 更新系统
sudo apt update

# 安装基本依赖
sudo apt install -y cmake build-essential git

# 安装ROS2依赖
sudo apt install -y ros-${ROS_DISTRO}-rclcpp ros-${ROS_DISTRO}-rclpy
sudo apt install -y ros-${ROS_DISTRO}-std-msgs ros-${ROS_DISTRO}-sensor-msgs
sudo apt install -y ros-${ROS_DISTRO}-pluginlib ros-${ROS_DISTRO}-eigen3-cmake-module
sudo apt install -y ros-${ROS_DISTRO}-ament-cmake ros-${ROS_DISTRO}-ament-index-cpp

# 安装第三方库依赖
sudo apt install -y libyaml-cpp-dev libspdlog-dev libfmt-dev

# 安装手柄控制依赖
sudo apt install -y ros-${ROS_DISTRO}-joy
```

### 2. 编译项目

```bash
# 进入colcon工作空间
cd ~/tklab_ws

# 编译rl_control_new包及其依赖
colcon build --packages-select rl_control_new x_humanoid_rl_sdk

# 或者编译所有包
colcon build
```

### 3. 设置环境变量

```bash
# source工作空间
source install/setup.bash
```

## 使用说明

### 启动控制节点

使用launch文件启动强化学习控制节点：

```bash
# 基本启动命令
ros2 launch rl_control_new rl.launch.py
```
### 配置文件说明

配置文件 [tg22_config.yaml](./config/tg22_config.yaml) 包含以下主要参数：

- `motor_num`: 电机数量
- `actions_size`: 动作空间维度
- `dt`: 控制周期
- `ct_scale`: 控制参数缩放因子
- `joint_kp_p`: 关节位置控制P增益
- `joint_kd_p`: 关节位置控制D增益
- `simulation`: 是否为仿真环境
- `mlp.path`: 策略网络模型路径

### 节点信息

主控制节点 `rl_control_new::RLControlNewPlugin` 提供以下功能：

1. 读取机器人状态（关节位置、速度、IMU数据等）
2. 运行强化学习策略网络
3. 计算并发布关节控制指令
4. 支持状态机管理（STOP, ZERO, MLP等状态）
5. 支持手柄控制输入

### 主要组件

#### RLControlNewPlugin 类
这是主控制类，继承自 rclcpp::Node，负责：
- ROS2节点管理
- 机器人状态订阅和控制命令发布
- 强化学习策略执行
- 与机器人硬件接口通信

#### x_humanoid_rl_sdk 插件
提供机器人控制的核心功能：
- 有限状态机实现 (FSM)
- 机器人接口抽象
- openvino 模型推理支持

## 手柄控制逻辑

系统支持Xbox手柄有线模式和机器人自带手柄(云卓)：两个手柄不能同时使用！！！

### 模式一：xbox控制模式(有线连接)
启动方式：
```bash
ros2 run joy joy_node --ros-args --remap joy:=sbus_data #确保和机器人在同一个domain_id内
```
手柄按键功能分配如下：

| 按键 | 功能 |
|------|------|
| A键 | 切换到MLP（机器学习策略）控制模式 |
| X键 | 切换到ZERO（归零）控制模式 |
| Y键 | 切换到STOP（停止）控制模式 |
| 左摇杆 | 控制机器人前进/后退和左移/右移 |
| 右摇杆 | 控制机器人转向（左转/右转） |

控制逻辑说明：
1. 机器人初始状态为STOP模式，启动后按ZERO模式可确保机器人所有关节都回到设置的零位状态
2. 按A键切换到MLP模式，机器人开始行走
3. 按X键回到ZERO模式，机器人回到初始姿态
4. 按Y键进入STOP模式，保持当前姿态
5. 状态切换流程为：STOP -> ZERO -> MLP -> STOP 

### 模式二：云卓控制模式
手柄按键功能分配如下：

| 按键 | 功能 |
|------|------|
| A键+G键(拨至中间零位) | 切换到MLP（机器学习策略）控制模式 |
| D键 | 切换到ZERO（归零）控制模式 |
| C键 | 切换到STOP（停止）控制模式 |
| 左摇杆 | 控制机器人前进/后退和左移/右移 |
| 右摇杆 | 控制机器人转向（左转/右转） |

控制逻辑说明：
1. 机器人初始状态为STOP模式，启动后按ZERO模式可确保机器人所有关节都回到设置的零位状态
2. 按A键+G键(拨至中间零位) 切换到MLP模式，机器人开始行走
3. 按D键回到ZERO模式，机器人回到初始姿态
4. 按C键进入STOP模式，保持当前姿态
5. 状态切换流程为：STOP -> ZERO -> MLP -> STOP

## 故障排除

### 编译问题

1. 确保所有依赖包都已正确安装
2. 检查Eigen3是否正确安装：
   ```bash
   sudo apt install libeigen3-dev
   ```
3. 确保yaml-cpp库正确安装：
   ```bash
   sudo apt install libyaml-cpp-dev
   ```

### 运行时问题

1. 确保配置文件路径正确
2. 检查策略网络模型文件是否存在
3. 验证ROS2环境变量是否正确设置
4. 确认机器人硬件连接正常

### 常见错误

1. 如果遇到找不到 [bodyctrl_msgs]包的错误，请确保已安装该包：
   ```bash
   # 需要从相应仓库获取并编译bodyctrl_msgs包
   ```

## 联系方式

如有问题，请联系项目维护团队。