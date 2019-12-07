/*
 * tricycle_driver.h
 *
 *  Created on: 2019年10月31日
 *      Author: kc.chang
 */

#include <controller_manager/controller_manager.h>

#include <tricycle_driver.h>

#if USECAN
#include <socketcan_cpp.h>

namespace {
bool is_can_ok = false;
scpp::SocketCan sockat_can;
double last_cmd[2];  // 用來送第二次
}  // namespace

#endif

namespace agv
{
/*
 * @brief  建構函式，註冊自製的控制器到 ROS 框架中，以便 ROS 可以管理。
 */
TricycleDriver::TricycleDriver(const std::string &port_1, const int baud,
      const int timeout, const int bytesize, const int parity)
#if USEIGAIN
  :pid_ctrl_(0.0, 0.1, 0.0, 6)
#endif
{
  (void)port_1;
  (void)baud;
  (void)timeout;
  (void)bytesize;
  (void)parity;

  // 初始化媒介變數
  for (int i = 0 ; i < num_of_joint_; i++) {
    pos_[i] = 0.0;
    vel_[i] = 0.0;
    eff_[i] = 0.0;
    cmd_[i] = 0.0;
  }

  // 連結舵關節 & 媒介變數：狀態發報用
  hardware_interface::JointStateHandle state_handle_steer(steer_joint_name_, &pos_[steer_index_], &vel_[steer_index_],
      &eff_[steer_index_]);
  joint_state_interface_.registerHandle(state_handle_steer);

  hardware_interface::JointStateHandle state_handle_wheel(wheel_joint_name_, &pos_[wheel_index_], &vel_[wheel_index_],
      &eff_[wheel_index_]);
  joint_state_interface_.registerHandle(state_handle_wheel);
  
  hardware_interface::JointStateHandle state_handle_fork(fork_joint_name_, &pos_[fork_index_], &vel_[fork_index_],
      &eff_[fork_index_]);
  joint_state_interface_.registerHandle(state_handle_fork);

  //註冊到 ROS 框架
  registerInterface(&joint_state_interface_);

  // 連結輪關節 & 媒介變數：控制器用
  hardware_interface::JointHandle pos_handle(
      joint_state_interface_.getHandle(steer_joint_name_), &cmd_[steer_index_]);
  joint_pos_interface_.registerHandle(pos_handle);

  hardware_interface::JointHandle vel_handle(
      joint_state_interface_.getHandle(wheel_joint_name_), &cmd_[wheel_index_]);
  joint_vel_interface_.registerHandle(vel_handle);
  
  hardware_interface::JointHandle vel_handle_fork(
      joint_state_interface_.getHandle(fork_joint_name_), &cmd_[fork_index_]);
  joint_vel_interface_.registerHandle(vel_handle_fork);  // 一個 vel interface 可以收容多個關節 handle

  //註冊到 ROS 框架
  registerInterface(&joint_pos_interface_);
  registerInterface(&joint_vel_interface_);

  #if tcdbg
  ROS_INFO_STREAM(">>>> start tricycle_driver ok !");
  #endif

#if USECAN
  // init canbus and autobox
  if (sockat_can.open("can0") == scpp::STATUS_OK) {
    is_can_ok = true;
  }
#endif
}

TricycleDriver::~TricycleDriver()
{
  if (stop()) {
    printf("Stopping motor successfully !");
  }
  else {
    printf("~TricycleDriver() can not stop motor.");
  }

#if USETHREAD
  printf(">> joint thread...");
  read_thread_.join();
#endif
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

#if USECAN
  if(!is_can_ok) {
    return;
  }

  // 將 cmd_[] 適度轉換成為 canbus 封包
  scpp::CanFrame cf_to_write;  //can frame to write
  cf_to_write.id = 123;
  cf_to_write.len = 8;
  cf_to_write.data[0] = 0x00;
  cf_to_write.data[1] = 0x01;
  cf_to_write.data[2] = 0x02;
  cf_to_write.data[3] = 0x03;
  cf_to_write.data[4] = 0x04;
  cf_to_write.data[5] = 0x05;
  cf_to_write.data[6] = 0x06;
  cf_to_write.data[7] = 0x07;

  auto write_sc_status = sockat_can.write(cf_to_write);
  last_cmd[0] = cmd_[0];
  last_cmd[1] = cmd_[1];
  if (write_sc_status != scpp::STATUS_OK) {
    ROS_ERROR_STREAM_ONCE("something went wrong on socket write, error code : " << int32_t(write_sc_status));
  }
  else {
    //ROS_INFO_STREAM("Message was written to the socket \n");
  }
#else
  #if USEIGAIN
  front_steer_.set_pos(cmd_[steer_index_]);  // 單位應為徑度

  pid_ctrl_.set_error(cmd_[wheel_index_] - vel_[wheel_index_]);
  double cmd_pushed = cmd_[wheel_index_] + pid_ctrl_.get_output();  // 推一把速度

  front_wheel_.set_vel(cmd_pushed);   // 單位應為 rad/s

  ROS_INFO(">>> cmd_= %lf, vel_ = %lf, cmd_pushed = %lf\n", cmd_[wheel_index_], vel_[wheel_index_], cmd_pushed);
  #else
  // 將 cmd_[STEER], cmd_[WHEEL] 代表的物理量，換算成 canbus 指令，並送出
  front_steer_.set_pos(cmd_[steer_index_]);  // 單位應為徑度
  front_wheel_.set_vel(cmd_[wheel_index_]);   // 單位應為 rad/s
  #endif
#endif

#if tcdbg
  ROS_INFO_STREAM_THROTTLE(1, ">> write:steer cmd_:" << cmd_[steer_index_]);
  ROS_INFO_STREAM_THROTTLE(1, ">> write:wheel cmd_:" << cmd_[wheel_index_]);
#endif
}

#if USECAN
void TricycleDriver::write_2nd()
{
  // 將 last_cmd[] 送第二次
  // 將 last_cmd[] 適度轉換成為 canbus 封包
  scpp::CanFrame cf_to_write;  //can frame to write
  cf_to_write.id = 123;
  cf_to_write.len = 8;
  cf_to_write.data[0] = 0x00;
  cf_to_write.data[1] = 0x11;
  cf_to_write.data[2] = 0x12;
  cf_to_write.data[3] = 0x13;
  cf_to_write.data[4] = 0x14;
  cf_to_write.data[5] = 0x15;
  cf_to_write.data[6] = 0x16;
  cf_to_write.data[7] = 0x17;

  auto write_sc_status = sockat_can.write(cf_to_write);
  if (write_sc_status != scpp::STATUS_OK) {
    ROS_ERROR_STREAM_ONCE("something went wrong on socket write_2nd, error code :"
        << int32_t(write_sc_status));
  }
  else {
    //ROS_INFO_STREAM("Message was written to the socket \n");
  }
}
#endif

/*
 * @brief  計算 odometry ，並寫入到 ROS 上層
 * Ref: https://github.com/TukamotoRyuzo/rostest/wiki/%E5%AE%9F%E6%A9%9F%E3%81%A7%E5%8B%95%E3%81%8F%E3%82%88%E3%81%86%E3%81%AB%E8%A8%AD%E5%AE%9A%E3%81%99%E3%82%8B
 */
void TricycleDriver::read(const ros::Duration period)
{
#if tcdbg
  ROS_INFO_STREAM_ONCE(">>> TricycleDriver::read() once !");
#endif

#if USECAN
  if(!is_can_ok) {
    return;
  }

  scpp::CanFrame fr;
  bool is_timeout = true;
  while (sockat_can.read(fr) == scpp::STATUS_OK) {
//    printf("len %d byte, id: %d, data: %02x %02x %02x %02x %02x %02x %02x %02x  \n", fr.len, fr.id,
//        fr.data[0], fr.data[1], fr.data[2], fr.data[3], fr.data[4], fr.data[5], fr.data[6],
//        fr.data[7]);

    // 加上 timeout, 跳脫 while
    is_timeout = false;
  }

  if(!is_timeout) {
    // 解讀 frame 封包內容
  }

#else
  // 將關節 steer/wheel 的速度、位置，更新到 vel_[]/pos_[]
  pos_[steer_index_] = front_steer_.get_pos();

  vel_[wheel_index_] = front_wheel_.get_vel();
  pos_[wheel_index_] = front_wheel_.get_vel();
#endif

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
#if USECAN  //ENTER / MODE

#endif

#if USETHREAD
  ROS_INFO_STREAM_ONCE(">> start init thread...");
  read_thread_ = std::thread([this]() {
    thread_func_();
  });
#endif

  return true;
}

/*
 * @brief  停止本 driver, 通常會做一些歸零、釋放通訊 port 的事情。
 *
*/
bool TricycleDriver::stop()
{
#if USECAN
  // EXIT mode

#endif
  return true;
}

#if USETHREAD
void TricycleDriver::thread_func_()
{
  static int i = 0;

  ROS_INFO_STREAM_ONCE(">> into thread_func()");

  while(ros::ok) {
    ROS_INFO_STREAM_THROTTLE(1, ">> thread_func: " << i);
    i++;
  }

  ROS_INFO_STREAM_ONCE(">> exit thread_func()");
}
#endif

}  // namespace agv
