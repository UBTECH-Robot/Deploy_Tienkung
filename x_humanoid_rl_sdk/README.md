# 天工行者人形机器人强化学习控制SDK

这是一个用于人形机器人强化学习控制的C++ SDK，包含状态机实现、机器人接口和控制算法。

## 目录结构

```
x_humanoid_rl_sdk/
├── include/
│   ├── robot_FSM/          # 机器人有限状态机实现
│   └── robot_interface/    # 机器人接口定义
├── src/
│   ├── robot_FSM/          # 状态机实现源码
│   └── robot_interface/    # 机器人接口实现源码
├── lib/                    # 第三方库文件
├── CMakeLists.txt          # CMake编译配置
└── package.xml             # ROS2包描述文件
```

## 依赖项

### 系统依赖
- Ubuntu 22.04 LTS
- ROS2 Humble
- C++17 或更高版本
- CMake 3.0.2 或更高版本
- Eigen3
- yaml-cpp
- sensor_msgs
- bodyctrl_msgs
- OpenVINO (默认，用于神经网络推理加速)

### 第三方库
- OpenVINO (默认)

## 编译说明

### 1. 确保工作空间结构正确
```bash
# 假设您的工作空间结构如下：
# tklab_ws/src/
# ├── x_humanoid_rl_sdk/
# ├── rl_control_new/
# └── ... 其他包
```

### 2. 安装依赖
```bash
# Ubuntu/Debian系统
sudo apt update
sudo apt install -y cmake build-essential libeigen3-dev libyaml-cpp-dev

# 安装ROS 2依赖
sudo apt install -y ros-${ROS_DISTRO}-rclcpp ros-${ROS_DISTRO}-pluginlib
sudo apt install -y ros-${ROS_DISTRO}-sensor-msgs

# 安装bodyctrl_msgs（如果尚未安装）
# 需要从相应仓库获取

# 安装OpenVINO（可选，用于神经网络推理加速）
# 请参考Intel官方文档安装OpenVINO
wget -O- https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB | gpg --dearmor | sudo tee /usr/share/keyrings/intel-sw-products.gpg > /dev/null

echo "deb [signed-by=/usr/share/keyrings/intel-sw-products.gpg] https://apt.repos.intel.com/openvino/2023 ubuntu20 main" | sudo tee /etc/apt/sources.list.d/intel-openvino-2023.list

sudo apt update
sudo apt install openvino
```

### 3. 编译项目
```bash
cd ~/tklab_ws
colcon build --packages-select x_humanoid_rl_sdk
```

### 4. 设置环境变量
```bash
source install/setup.bash
```

## 使用说明

### 头文件包含
```cpp
#include "robot_FSM/RobotFSM.h"
#include "robot_interface/RobotInterface.h"
```

### 主要组件

#### 1. 机器人状态机 (FSM)
- `FSMState`: 基础状态类
- `RobotFSM`: 机器人状态机管理器
- 支持多种状态: STOP(停止), ZERO(归零), MLP(强化学习控制)

#### 2. 机器人接口
- `RobotInterface`: 机器人硬件接口抽象
- 提供关节控制、状态读取等功能
- `RobotData`: 机器人数据结构，包含关节位置、速度、力矩等信息

#### 3. 神经网络推理
- OpenVINO(默认)

### API参考

#### RobotFSM类
```cpp
// 获取机器人状态机实例
RobotFSM* robot_fsm = get_robot_FSM(robot_data);

// 运行状态机
robot_fsm->RunFSM(flag);

// 获取当前状态
FSMStateName current_state = robot_fsm->getCurrentState();
```

#### RobotInterface类
```cpp
// 获取机器人接口实例
RobotInterface* robot_interface = get_robot_interface();

// 初始化接口
robot_interface->Init();

// 获取状态
robot_interface->GetState(time, robot_data);

// 设置控制命令
robot_interface->SetCommand(robot_data);

// 禁用所有关节
robot_interface->DisableAllJoints();
```

### RobotData结构体
```cpp
struct RobotData {
  Eigen::VectorXd q_a_;       // 实际关节位置 (26维: 6自由度浮动基座 + 20关节)
  Eigen::VectorXd q_dot_a_;   // 实际关节速度 (26维)
  Eigen::VectorXd tau_a_;     // 实际关节力矩 (26维)
  
  Eigen::VectorXd q_d_;       // 期望关节位置 (26维)
  Eigen::VectorXd q_dot_d_;   // 期望关节速度 (26维)
  Eigen::VectorXd tau_d_;     // 期望关节力矩 (26维)
  
  Eigen::VectorXd joint_kp_p_; // 关节位置控制P增益 (20维)
  Eigen::VectorXd joint_kd_p_; // 关节位置控制D增益 (20维)
  
  Eigen::VectorXd imu_data_;  // IMU数据 (9维: 3欧拉角 + 3角速度 + 3加速度)
  double gait_a;              // 步态参数
  double time_now_;           // 当前时间
  std::string config_file_;   // 配置文件路径
  bool pos_mode_;             // 位置控制模式标志
};
```

## 注意事项

1. 确保在ROS 2环境下编译和运行
2. 需要正确配置YAML配置文件路径
3. 策略网络模型文件需要与配置文件中指定的路径一致
4. IMU和电机数据需要正确订阅和处理

## 故障排除

### 编译错误
- 检查Eigen3和YAML-CPP是否正确安装
- 确认CMake版本满足要求
- 验证ROS 2环境变量是否设置正确
- 确保bodyctrl_msgs包已正确安装

### 运行时错误
- 检查配置文件路径是否正确
- 确认模型文件是否存在且格式正确
- 验证硬件连接和数据订阅


## 联系方式

如有问题，请联系项目维护团队。
