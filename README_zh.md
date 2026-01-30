# 天工行者人形机器人强化学习控制项目集

这个仓库包含了天工行者系列人形机器人强化学习控制系统的主要组件。

**[English](./README.md)｜简体中文**

## 项目结构

```
Deploy_Tienkung/
├── rl_control_new/         # 强化学习控制主库
└── x_humanoid_rl_sdk/      # 机器人控制SDK
```

## 组件介绍

### [rl_control_new]

基于ROS2的人形机器人强化学习控制库，可以用于控制天工行者系列人形机器人。该库使用强化学习算法实现机器人运动控制，支持仿真和真实机器人环境。

主要特性：

- 基于强化学习的机器人控制策略
- 支持仿真和实机部署
- ROS2集成，便于系统扩展
- 插件化架构设计

### [x_humanoid_rl_sdk]

天工行者人形机器人强化学习控制SDK，包含状态机实现、机器人接口和控制算法。

主要特性：

- 有限状态机实现
- 标准化机器人接口定义
- 控制算法封装
- 易于集成的C++库

## 使用说明

请参考各子项目的README文档获取详细的安装和使用说明：

- [rl_control_new README](./rl_control_new/README.md)
- [x_humanoid_rl_sdk README](./x_humanoid_rl_sdk/README.md)

## 系统要求

- Ubuntu 22.04 LTS
- ROS2 Humble
- C++17 编译器
- CMake 3.8+
- Eigen3
- 其他依赖详见各子项目文档

## 致谢

**特别感谢北京人形机器人创新中心提供的宝贵支持与指导。**

**项目链接：** [Deploy_Tienkung](https://github.com/Open-X-Humanoid/Deploy_Tienkung)

## 联系方式

如有问题，请联系项目维护团队。
