/*
 * tricycle_driver.h
 *
 *  Created on: 2019年10月31日
 *      Author: kc.chang
 */
#ifndef FAKE_MOTOR_H_
#define FAKE_MOTOR_H_
#include <queue>
namespace agv
{
class FakeMotor {
public:
  using uptr = std::unique_ptr<FakeMotor>;

  static uptr create_instance(const int queue_size)
  {
    return std::make_unique<FakeMotor>(queue_size);
  }

  FakeMotor(const int queue_size = 1)
  {
    for (int i = 0; i < queue_size; i++) {
      vel_queue_.push(0.0);
      pos_queue_.push(0.0);
    }
  }

  void set_vel(double vel)
  {
    vel_queue_.pop();
    vel_queue_.push(vel);
  }

  double get_vel() 
  {
    return vel_queue_.front();
  }

  void set_pos(double pos)
  {
    pos_queue_.pop();  // 取得 queue 中最早的數值
    pos_queue_.push(pos);
  }

  double get_pos()
  {
    return pos_queue_.front();
  }

private:
  double vel_ = 0.0;
  double pos_ = 0.0;

  std::queue<double> vel_queue_;
  std::queue<double> pos_queue_;
};

}  // namespace agv

#endif // FAKE_MOTOR_H_
