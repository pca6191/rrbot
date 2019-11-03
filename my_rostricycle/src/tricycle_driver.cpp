/*
 * tricycle_driver.h
 *
 *  Created on: 2019年10月31日
 *      Author: kc.chang
 */

#include <controller_manager/controller_manager.h>

#include <tricycle_driver.h>

namespace agv
{
/*
 * @brief  建構函式，註冊自製的控制器到 ROS 框架中，以便 ROS 可以管理。
 */
TricycleDriver::TricycleDriver(const std::string &port_1, const int baud,
      const int timeout, const int bytesize, const int parity)
{
  (void)port_1;
  (void)baud;
  (void)timeout;
  (void)bytesize;
  (void)parity;

  // 初始化媒介變數
  pos_[steer_index_] = 0.0;
  pos_[wheel_index_] = 0.0;

  vel_[steer_index_] = 0.0;
  vel_[wheel_index_] = 0.0;

  eff_[steer_index_] = 0.0;
  eff_[wheel_index_] = 0.0;

  cmd_[steer_index_] = 0.0;
  cmd_[wheel_index_] = 0.0;

  // 連結舵關節 & 媒介變數：狀態發報用
  hardware_interface::JointStateHandle state_handle_steer(steer_joint_name_, &pos_[steer_index_], &vel_[steer_index_],
      &eff_[steer_index_]);
  joint_state_interface_.registerHandle(state_handle_steer);

  hardware_interface::JointStateHandle state_handle_wheel(wheel_joint_name_, &pos_[wheel_index_], &vel_[wheel_index_],
      &eff_[wheel_index_]);
  joint_state_interface_.registerHandle(state_handle_wheel);

  //註冊到 ROS 框架
  registerInterface(&joint_state_interface_);

  // 連結輪關節 & 媒介變數：控制器用
  hardware_interface::JointHandle pos_handle(
      joint_state_interface_.getHandle(steer_joint_name_), &cmd_[steer_index_]);
  joint_pos_interface_.registerHandle(pos_handle);

  hardware_interface::JointHandle vel_handle(
      joint_state_interface_.getHandle(wheel_joint_name_), &cmd_[wheel_index_]);
  joint_vel_interface_.registerHandle(vel_handle);

  //註冊到 ROS 框架
  registerInterface(&joint_pos_interface_);
  registerInterface(&joint_vel_interface_);

  #if tcdbg
  ROS_INFO_STREAM(">>>> start tricycle_driver ok !");
  #endif
}

TricycleDriver::~TricycleDriver()
{
  if (stop()) {
    ROS_INFO_STREAM("Stopping motor successfully !");
  }
  else {
    ROS_ERROR_STREAM_ONCE("~TricycleDriver() can not stop motor.");
  }
}

/*
 * @brief  讀取 ROS 上層算好的驅動指令 cmd_[]，對硬體下達驅動指令
 * Ref: https://github.com/TukamotoRyuzo/rostest/wiki/%E5%AE%9F%E6%A9%9F%E3%81%A7%E5%8B%95%E3%81%8F%E3%82%88%E3%81%86%E3%81%AB%E8%A8%AD%E5%AE%9A%E3%81%99%E3%82%8B
 */
void TricycleDriver::write()
{
#if tcdbg
  ROS_INFO_STREAM_ONCE(">>> TricycleDriver::write() once !");
#endif

  // 將 cmd_[STEER], cmd_[WHEEL] 代表的物理量，換算成 canbus 指令，並送出
  front_steer_.set_pos(cmd_[steer_index_]);  // 單位應為徑度
  front_wheel_.set_vel(cmd_[wheel_index_]);   // 單位應為 rad/s

#if tcdbg
  ROS_INFO_STREAM_THROTTLE(1, ">> write:steer cmd_:" << cmd_[steer_index_]);
  ROS_INFO_STREAM_THROTTLE(1, ">> write:wheel cmd_:" << cmd_[wheel_index_]);
#endif
}

/*
 * @brief  計算 odometry ，並寫入到 ROS 上層
 * Ref: https://github.com/TukamotoRyuzo/rostest/wiki/%E5%AE%9F%E6%A9%9F%E3%81%A7%E5%8B%95%E3%81%8F%E3%82%88%E3%81%86%E3%81%AB%E8%A8%AD%E5%AE%9A%E3%81%99%E3%82%8B
 */
void TricycleDriver::read(const ros::Duration period)
{
#if tcdbg
  ROS_INFO_STREAM_ONCE(">>> TricycleDriver::read() once !");
#endif
  // 將關節 steer/wheel 的速度、位置，更新到 vel_[]/pos_[]
  pos_[steer_index_] = front_steer_.get_pos();

  vel_[wheel_index_] = front_wheel_.get_vel();
  pos_[wheel_index_] = front_wheel_.get_vel();

#if tcdbg
  ROS_INFO_STREAM_THROTTLE(1, ">>> read:steer pos_:" << pos_[steer_index_]);
  ROS_INFO_STREAM_THROTTLE(1, ">>> read:wheel vel_:" << vel_[wheel_index_]);
  ROS_INFO_STREAM_THROTTLE(1, ">>> read:wheel pos_:" << pos_[wheel_index_]);
#endif
}

/*
 * @brief  啟用本 driver, 目前還不知道要做什麼 !
 *
*/
bool TricycleDriver::start()
{
  return true;
}

/*
 * @brief  停止本 driver, 通常會做一些歸零、釋放通訊 port 的事情。
 *
*/
bool TricycleDriver::stop()
{
  return true;
}


}  // namespace agv
