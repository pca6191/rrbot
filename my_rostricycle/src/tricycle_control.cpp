/*
 * tricycle_driver.h
 *
 *  Created on: 2019年10月31日
 *      Author: kc.chang
 */
#include <tricycle_driver.h>
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>

int main(int argc, char *argv[])
{
  // ROS_INFO_STREAM(agv_control_package_info);
  ros::init(argc, argv, "tricycle_control");
  ros::NodeHandle nh;

#if 0
  double period;
  double period_default = 0.1;
  std::string port_1;
  std::string port_1_default = "/dev/ttyUSB0";
  std::string port_2;
  std::string port_2_default = "/dev/ttyUSB1";
  int baud;
  int baud_default = 115200;
  int timeout;
  int timeout_default = 250;  // unit:ms
  int bytesize;
  int bytesize_default = 8;  // eightbits:8
  int parity;
  int parity_default = 2;  // parity_even:2

  nh.param("period", period, period_default);
  nh.param("port_1", port_1, port_1_default);
  nh.param("port_2", port_2, port_2_default);
  nh.param("baud", baud, baud_default);
  nh.param("timeout", timeout, timeout_default);
  nh.param("bytesize", bytesize, bytesize_default);
  nh.param("parity", parity, parity_default);
#else
  std::string port_1 =  "/dev/ttyUSB0";
  int baud = 115200;
  int timeout = 250;  // unit:ms
  int bytesize = 8;  // eightbits:8
  int parity = 2;  // parity_even:2

  double period = 0.1;

#endif

  agv::TricycleDriver tricycle_driver(port_1, baud, timeout, bytesize, parity);

  if (!tricycle_driver.start()) {
    ROS_ERROR_STREAM_ONCE("Initialize the motors failed.");
    return 0;
  }  // else do the following

  // 將硬體界面註冊到 ROS 框架中的管理器
  controller_manager::ControllerManager cm(&tricycle_driver, nh);

  ros::Rate rate(1.0 / period);
  ros::AsyncSpinner spinner(1);  // Use 1 thread
  spinner.start();

  while (ros::ok()) {
    ros::Time now = ros::Time::now();
    ros::Duration dt = ros::Duration(period);
    tricycle_driver.read(dt);
    cm.update(now, dt);
    tricycle_driver.write();

    rate.sleep();
  }
  spinner.stop();

  return 0;
}
