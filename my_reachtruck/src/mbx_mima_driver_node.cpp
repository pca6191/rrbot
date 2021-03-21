/*
 * mbx_mima_driver_node.cpp
 *
 *  Created on: 2021/3/20
 *      Author: kcchang
 */
#include <parameters.h>
#include <mobox_server.h>
#include <ros/ros.h>

/*
 * @brief  本 node 扮演 mobox server 角色，接收 ROS base control (amr_mima) 傳來的封包，
 *         控制 gazebo 內的 trac / steer / fork 物件，用以模擬 PCBA +  叉車的作動。
 */
namespace amr {
class MbxSMimaDriver {
public:
  /// 要從 main 傳入 nh("~")，才能正常抓 param
  explicit MbxSMimaDriver(const ros::NodeHandle nh)
      : nh_(nh), parameters_(nh)
  {
  }

  bool prepare_parameter()
  {
    if (!parameters_.get_parameters()) {
      ROS_ERROR("[%s] prepare_parameter() failed !", class_name_);
      return false;
    }

    return true;
  }

  bool init()
  {
    return true;
  }

  bool start()
  {
    // 將訊息發布到 topic 上
//    auto motion_cb =[this](double trac_mmps, double steer_cdeg,
//        double mdv_trac_mmps, double mdv_steer_cdeg) {
//      report_trac_steer_callback(trac_mmps, steer_cdeg, mdv_trac_mmps, mdv_steer_cdeg);
//    };
//
//    auto health_cb = [this](uint8_t trac_err, uint8_t steer_err,
//        uint8_t pump_err, uint8_t _1353_err, uint8_t _1356p_err,
//        uint8_t bat, uint8_t detectors) {
//      report_health_callback(trac_err, steer_err, pump_err, _1353_err, _1356p_err, bat, detectors);
//    };
//
//    auto fork_sensor_cb = [this](double x_mm, double y_mm, double z_mm) {
//      report_fork_position_callback(x_mm, y_mm, z_mm);
//    };

    mobox_server_->start_listen();
    // mobox_server_->set_recieved_callback(motion_cb, health_cb, fork_sensor_cb);

    ros::Rate rate(loop_rate_);

    // 將舵輪速度、角度 (放在 linear.y / angular.y) 打成封包送到硬體
    while (ros::ok()) {
      rate.sleep();
      ros::spinOnce();
    }

    // ** ROS 若停掉 -> main 退出 --> mobox_server_ 解構 --> stop & 退到手動

    return true;
  }

  static constexpr const char* class_name_ = "MbxServerMima";

private:
  // ---- private 變數區 -------------------------
  ros::NodeHandle nh_;
  Parameters parameters_;
  MoBoxServer::uptr mobox_server_ = nullptr;

  const int loop_rate_ = 50;  // 單位: Hz, 發送 cmd_vel 的頻率
};

}  // namespace amr


///////////////////////////////////////////////////////////////////////////////
int main(int argc, char *argv[])
{
  ros::init(argc, argv, amr::MbxSMimaDriver::class_name_);
  ros::NodeHandle nh("~");

  amr::MbxSMimaDriver ctrl(nh);

  if (!ctrl.prepare_parameter()) {
    ROS_ERROR_STREAM("[" << amr::MbxSMimaDriver::class_name_ << "] prepare_parameter falied !!");
    return EXIT_FAILURE;
  }

  if (!ctrl.init()) {
    ROS_ERROR_STREAM("[" << amr::MbxSMimaDriver::class_name_ << "] init falied !!");
    return EXIT_FAILURE;
  }

  if (!ctrl.start()) {
    // 按下 ctr+c, ros 已經不 ok, ROS_INFO() 失效，只能 cout or printf
    std::cout << "[" << amr::MbxSMimaDriver::class_name_ << "] start() failed, exit !!" << std::endl;
    return EXIT_FAILURE;
  }

  return 0;
}
