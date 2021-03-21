/*
 * parameters.cpp
 *
 *  Created on: 2020年12月25日
 *      Author: kcchang
 */

#include <parameters.h>

namespace amr
{

Parameters::Parameters(const ros::NodeHandle nh)
    : nh_(nh)
{
}

template<class T>
bool Parameters::get_one_parameters(std::string key, T *value)
{
  T val;

  if (!nh_.getParam(key, val)) {
    ROS_ERROR_STREAM("[" << class_name_ << "] find no param: " << key << " !");
    return false;
  }

  (*value) = val;
  ROS_INFO_STREAM("[" << class_name_ << "] " << key << ": " << val);

  return true;
}

bool Parameters::get_parameters()
{
  if (!get_one_parameters("/user_param/serial_port_simulation", &serial_port_)) {
    return false;
  }

  if (!get_one_parameters("/user_param/serial_baudrate", &serial_baudrate_)) {
    return false;
  }

  if (!get_one_parameters("/user_param/serial_timeout_ms", &serial_timeout_ms_)) {
    return false;
  }

  return true;
}

}  // namespace amr
