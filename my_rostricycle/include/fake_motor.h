/*
 * tricycle_driver.h
 *
 *  Created on: 2019年10月31日
 *      Author: kc.chang
 */
#ifndef FAKE_MOTOR_H_
#define FAKE_MOTOR_H_

namespace agv
{
class FakeMotor {
public:
  using uptr = std::unique_ptr<FakeMotor>;

  static uptr create_instance()
  {
    return std::make_unique<FakeMotor>();
  }

  FakeMotor()
  { }

  void set_vel(double vel)
  {
    vel_ = vel;
  }

  double get_vel() 
  {
    return vel_;
  }

  void set_pos(double pos)
  {
    pos_ = pos;
  }

  double get_pos()
  {
    return pos_;
  }
 
  // 上次 update 到這次，經過 dt
  void update(double dt)
  {
    pos_ += vel_*dt;
  }

private:
  double vel_ = 0.0;
  double pos_ = 0.0;
};

}  // namespace agv

#endif // FAKE_MOTOR_H_
