/*
 * agv_pid_controller.h
 *
 *  Created on: 2019年12月07日
 *      Author: kc.chang
 */
#ifndef AGV_PID_CONTROLLER_H_
#define AGV_PID_CONTROLLER_H_

#include <string>
#include <vector>

namespace agv {
class PIDController {
public:
  PIDController(const double pgain, const double igain, const double dgain, const int win_size);

  ~PIDController();

  void set_error(double err);
  double get_output();

private:
  double p_gain_ = 1.0;
  double i_gain_ = 0.0;
  double d_gain_ = 0.0;

  int win_size_ = 0;
  int last_sample_pos_ = 0;
  std::vector<double> samples_;  // size = win_size
};

}  // namespace agv
#endif  // AGV_PID_CONTROLLER_H_
