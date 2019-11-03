/*
 * numpin.h
 *
 *  Created on: 2019年10月31日
 *      Author: kc.chang
 */
#ifndef NUMPIN_H_
#define NUMPIN_H_

#include <string>
#include <memory>
#include <utility>

#include <ros/ros.h>
#include <std_msgs/Float64.h>

namespace agv {

/*
 * @file  簡單給一個 topic, 讓 debug 數據顯現。
 */
class Numpin {
public:
  using uptr = std::unique_ptr<Numpin>;

  static uptr create_instance(const std::string &topic, ros::NodeHandle &n)
  {
    return std::make_unique<Numpin>(topic, n);
  }
  
  Numpin(const std::string &topic, ros::NodeHandle &n) {
    pub_ = n.advertise<std_msgs::Float64>(topic, 10);
  }

  void publish(double num) {
    std_msgs::Float64 f64;
    f64.data = num;
    pub_.publish(f64);
  }
  

private:
  ros::Publisher pub_;

};

}  // namespace agv
#endif  // NUMPIN_H_
