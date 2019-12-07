/*
 * agv_pid_controller.h
 *
 *  Created on: 2019年12月07日
 *      Author: kc.chang
 */

#include <agv_pid_controller.h>

namespace agv {
PIDController::PIDController(const double pgain, const double igain, const double dgain, const int win_size)
  : samples_(win_size, 0.0)
{
  p_gain_ = pgain;
  i_gain_ = igain;
  d_gain_ = dgain;

  win_size_ = win_size;
}

PIDController::~PIDController()
{
}

void PIDController::set_error(double err)
{
  if (last_sample_pos_ < win_size_) {  // buffer 未滿
    samples_[last_sample_pos_] = err;
    last_sample_pos_++;
    last_sample_pos_ = (last_sample_pos_ < win_size_) ? last_sample_pos_ : (win_size_ - 1);
  }
  else {  // 先將 samples 往 [0] 挪動
    for (int i = 1; i < win_size_; i++) {
      samples_[i - 1] = samples_[i];
    }

    samples_[last_sample_pos_] = err;
  }
}

double PIDController::get_output()
{
  double p_err = 0.0;
  double i_err = 0.0;
  double d_err = 0.0;

  // count p error
  p_err = samples_[last_sample_pos_] * p_gain_;

  // count i error
  for (int i = 0; i <= last_sample_pos_; i++) {
    i_err += samples_[i];
  }
  i_err *= i_gain_;

  // count d error
  if (last_sample_pos_ > 0) {
    d_err = samples_[last_sample_pos_] - samples_[last_sample_pos_ - 1];
    d_err *= d_gain_;
  }

  return (p_err + i_err + d_err);
}

}  // namespace agv
