#include "RobotInterface.h"

// 单例实现
class RobotInterfaceImpl : public RobotInterface {
public:
  void Init() override {
    // 你可以在这里建立设备连接、订阅 ROS 主题等
  }

  void GetState(double t, RobotData &robot_data) override {
    // 主循环里 robot_data 已更新，这里可以留空或添加日志
  }

  void SetCommand(RobotData &robot_data) override {
    // 主循环中你已发布命令，这里可留空
  }

  void DisableAllJoints() override {
    error_state_ = true;
    robot_data_.q_d_.setZero();
    robot_data_.q_dot_d_.setZero();
    robot_data_.tau_d_.setZero();
    robot_data_.joint_kp_p_.setZero();
    robot_data_.joint_kd_p_.setZero();
  }

private:
  RobotData robot_data_;
};

static RobotInterfaceImpl *instance = nullptr;

RobotInterface *get_robot_interface() {
  if (!instance) instance = new RobotInterfaceImpl();
  return instance;
}
