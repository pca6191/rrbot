/*
 * tricycle_driver.h
 *
 *  Created on: 2019年10月31日
 *      Author: kc.chang
 */
#ifndef TRICYCLE_DRIVER_H_
#define TRICYCLE_DRIVER_H_

#include <string>
#include <vector>
//#include <thread>

#include <ros/ros.h>
#include <ros/console.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>

// #include <agv_modbus.h>

namespace agv
{

/*
 * @file  繼承 RobotHW 介面，實做 velocity() / position()，對硬體溝通、控制。
 */
class TricycleDriver: public hardware_interface::RobotHW {
public:
  TricycleDriver(const std::string &port_1, const int baud,
      const int timeout, const int bytesize, const int parity);

  ~TricycleDriver();

  void read(const ros::Duration);
  void write();

  bool start();
  bool stop();

private:
  
  #if 0
  enum class MotorID {
    // 馬達編號
    LEFT_ID = 1,
    RIGHT_ID = 2,
  };
  #endif
  
  // canbud 最基本通訊函式
  bool send_command(const uint16_t id, const uint16_t cmd);

  // 本 driver 與 control manager 對接的出入口
  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::PositionJointInterface joint_pos_interface_;
  hardware_interface::VelocityJointInterface joint_vel_interface_;

  const std::string steer_joint_name_ = "front_steering_joint";
  const std::string wheel_joint_name_ = "front_wheel_joint";

  // 與 ROS control manager 對傳資料的變數
  constexpr static uint8_t num_of_joint_ = 2;
  constexpr static int steer_index_ = 0;  // 陣列索引位置
  constexpr static int wheel_index_ = 1;  // 陣列索引位置
  enum class JointIndex {
    // 關節編號
    STEER = 1,
    WHEEL = 2,
  };
  double cmd_[TricycleDriver::num_of_joint_];
  double pos_[TricycleDriver::num_of_joint_];
  double vel_[TricycleDriver::num_of_joint_];
  double eff_[TricycleDriver::num_of_joint_];
};

}  // namespace agv
#endif  // TRICYCLE_DRIVER_H_
