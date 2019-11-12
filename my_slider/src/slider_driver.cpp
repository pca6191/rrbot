/*
 * slider_driver.cpp
 *
 *  Created on: 2019年11月12日
 *      Author: KC
 */
#include <slider_driver.h>
#include <controller_manager/controller_manager.h>

namespace rrbot
{
/*
 * @brief  建構函式，註冊自製的控制器到 ROS 框架中，以便 ROS 可以管理。
 */
SliderDriver::SliderDriver()
{
  pos_[0] = 0.0;
  vel_[0] = 0.0;
  eff_[0] = 0.0;
  cmd_[0] = 0.0;

  // connect and register the joint state interface
  hardware_interface::JointStateHandle state_handle_l(slider_joint_name_, &pos_[0], &vel_[0],
      &eff_[0]);
  jnt_state_interface_.registerHandle(state_handle_l);

  //註冊到 ROS 框架
  registerInterface(&jnt_state_interface_);

  // connect and register the joint velocity interface
  hardware_interface::JointHandle pos_handle_l(
      jnt_state_interface_.getHandle(slider_joint_name_), &cmd_[0]);
  jnt_pos_interface_.registerHandle(pos_handle_l);

  //註冊到 ROS 框架
  registerInterface(&jnt_pos_interface_);
}

SliderDriver::~SliderDriver()
{
  ROS_ERROR_STREAM_ONCE("~SliderDriver() can not stop.");
}

/*
 * @brief  讀取 ROS 上層算好的驅動指令 cmd_[]，對硬體下達驅動指令
 * Ref: https://github.com/TukamotoRyuzo/rostest/wiki/%E5%AE%9F%E6%A9%9F%E3%81%A7%E5%8B%95%E3%81%8F%E3%82%88%E3%81%86%E3%81%AB%E8%A8%AD%E5%AE%9A%E3%81%99%E3%82%8B
 */
void SliderDriver::write()
{
	ROS_INFO_STREAM("[write, cmd_[0]]" << cmd_[0]);
}

/*
 * @brief  計算 odometry ，並寫入到 ROS 上層
 * Ref: https://github.com/TukamotoRyuzo/rostest/wiki/%E5%AE%9F%E6%A9%9F%E3%81%A7%E5%8B%95%E3%81%8F%E3%82%88%E3%81%86%E3%81%AB%E8%A8%AD%E5%AE%9A%E3%81%99%E3%82%8B
 */
void SliderDriver::read(const ros::Duration)
{
    ROS_INFO_STREAM("[write, cmd_[0]]" << cmd_[0]);
}

}  // namespace rrbot
