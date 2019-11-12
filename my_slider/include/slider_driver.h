/*
 * slider_driver.h
 *
 *  Created on: 2019年11月12日
 *      Author: KC
 */
#ifndef SLIDER_DRIVER_H_
#define SLIDER_DRIVER_H_

#include <string>
#include <vector>	
#include <thread>

#include <ros/ros.h>
#include <ros/console.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>

namespace rrbot
{

/*
 * @file  繼承 RobotHW 介面，實做 velocity() / position()，對硬體溝通、控制。
 */
class SliderDriver: public hardware_interface::RobotHW {
public:
  SliderDriver();

  ~SliderDriver();

  void read(const ros::Duration);
  void write();

private:
  hardware_interface::JointStateInterface jnt_state_interface_;
  hardware_interface::PositionJointInterface jnt_pos_interface_;

  const std::string slider_joint_name_ = "joint1";
  
  double cmd_[1];
  double pos_[1];
  double vel_[1];
  double eff_[1];
};


}  // namespace rrbot
#endif  // AGV_CONTROL_INCLUDE_AGV_MOTOR_DRIVER_H_
