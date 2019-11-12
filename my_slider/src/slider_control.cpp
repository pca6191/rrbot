/*
 * slide_control.cpp
 *
 *  Created on: 2019年11月12日
 *      Author: kcchang
 *      Ref: http://daikimaekawa.github.io/ros/2014/12/19/ros_control
 */
#include <slider_driver.h>
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "slider_control");
  ros::NodeHandle nh;

  rrbot::SliderDriver slider_driver;

  // 將硬體界面註冊到 ROS 框架中的管理器
  controller_manager::ControllerManager cm(&slider_driver, nh);

  ros::Rate rate(1.0 / 1);
  ros::AsyncSpinner spinner(1);  // Use 1 thread
  spinner.start();

  while (ros::ok()) {
    ros::Time now = ros::Time::now();
    ros::Duration dt = ros::Duration(1);
    slider_driver.read(dt);
    cm.update(now, dt);
    slider_driver.write();

    rate.sleep();
  }
  spinner.stop();

  return 0;
}
