/*
 * parameters.h
 *
 *  Created on: 2020年12月25日
 *      Author: kcchang
 */

#ifndef MY_REACHTRUCK_INCLUDE_PARAMETERS_H_
#define MY_REACHTRUCK_INCLUDE_PARAMETERS_H_
#include <string>
#include <ros/ros.h>

/// @file  負責從 ROS parameter server 收集本 package 所需的參數
namespace amr
{
class Parameters {
public:
  explicit Parameters(const ros::NodeHandle nh);

  /// 如果有參數不存在，return false
  bool get_parameters();

  // ---- 來自於 ~/.ros/user_param.yaml 的參數 -------------------------------------
  //  可在 ~/.ros 執行 $ rosparam load user_param.yaml 載入以下參數到 ROS param server
  // ------------------------------------------------------------------------------
  std::string serial_port_ = "/dev/ttyS0";
  int serial_baudrate_ = 115200;
  int serial_timeout_ms_ = 200;

private:
  ros::NodeHandle nh_;

  // 以字串 key 為索引，從 ROS parameter server 取得對應的數值
  template<class T>
  bool get_one_parameters(std::string key, T *value);

  static constexpr const char* class_name_ = "Parameters";
};

}  // namespace amr




#endif /* MY_REACHTRUCK_INCLUDE_PARAMETERS_H_ */
