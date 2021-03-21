/*
 * mbx_mima_driver_node.cpp
 *
 *  Created on: 2021/3/20
 *      Author: kcchang
 */
#include <parameters.h>
#include <mobox_server.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>

/*
 * @brief  本 node 扮演 mobox server 角色，接收 ROS base control (amr_mima) 傳來的封包，
 *         控制 gazebo 內的 trac / steer / fork 物件，用以模擬 PCBA +  叉車的作動。
 */
namespace amr {
class MbxMimaDriver {
public:
  /// 要從 main 傳入 nh("~")，才能正常抓 param
  explicit MbxMimaDriver(const ros::NodeHandle nh)
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
    trac_cmd_pub_ = nh_.advertise<std_msgs::Float64>("/reachtruck/joint_trac_velocity_controller/command", 2);
    steer_cmd_pub_ = nh_.advertise<std_msgs::Float64>("/reachtruck/joint_steer_position_controller/command", 2);
    fork_x_cmd_pub_ = nh_.advertise<std_msgs::Float64>("/reachtruck/joint_fork_x_velocity_controller/command", 2);
    fork_y_cmd_pub_ = nh_.advertise<std_msgs::Float64>("/reachtruck/joint_fork_y_velocity_controller/command", 2);
    fork_z_cmd_pub_ = nh_.advertise<std_msgs::Float64>("/reachtruck/joint_fork_z_velocity_controller/command", 2);
    fork_rot_cmd_pub_ = nh_.advertise<std_msgs::Float64>("/reachtruck/joint_fork_rot_velocity_controller/command", 2);

    joint_states_sub_ = nh_.subscribe("/reachtruck/joint_states", 2, &MbxMimaDriver::update_joint_states_callback,
        this);

    mobox_server_ = MoBoxServer::create_instance(parameters_.serial_port_, parameters_.serial_baudrate_,
        parameters_.serial_timeout_ms_);

    if (!mobox_server_->start()) {
      return false;
    }
    return true;
  }

  bool start()
  {
    mobox_server_->start_listen();
    ros::Rate rate(loop_rate_);

    // 定時調整模擬器的部件運動狀態
    while (ros::ok()) {
      std_msgs::Float64 val;

      // 設置 trac / steer 動作
      trac_cmd_pub_.publish(get_trac_radps());
      steer_cmd_pub_.publish(get_steer_rad());
      // 設置 fork 動作
      fork_x_cmd_pub_.publish(get_fork_x_effort());
      fork_y_cmd_pub_.publish(get_fork_y_effort());
      fork_z_cmd_pub_.publish(get_fork_z_effort());
      fork_rot_cmd_pub_.publish(get_fork_rot_effort());

      // 回應 (sensor) 狀態
      mobox_server_->set_sensor_trac_mmps(-1000.0);
      mobox_server_->set_sensor_steer_cdeg(-9000.0);
      mobox_server_->set_sensor_fork_y_mm(-2000.0);
      mobox_server_->set_sensor_fork_z_mm(-3000.0);
      if (!mobox_server_->send_packet()) {
        ROS_ERROR("[%s] send_packet() error !", class_name_);
      }

      rate.sleep();
      ros::spinOnce();
    }

    return true;
  }

  static constexpr const char* class_name_ = "MbxMimaDriver";

private:
  std_msgs::Float64 get_trac_radps()
  {
    std_msgs::Float64 v;
    double mmps;
    mmps = mobox_server_->get_trac_mmps();
    v.data = mmps / 1000.0 / mima_wheel_radius_;  // 轉成 rad/s

    return v;  // 傳出角速度，讓模擬器轉動感覺，合乎實際輪子轉起來的感覺
  }

  std_msgs::Float64 get_steer_rad()
  {
    std_msgs::Float64 v;
    double cdeg;
    cdeg = mobox_server_->get_steer_cdeg();
    v.data = cdeg / 100.0 / M_PI * 180;  // 轉成 rad

    return v;  // 傳出 rad
  }

  std_msgs::Float64 get_fork_x_effort()
  {
    std_msgs::Float64 v;
    double mmps;
    mmps = mobox_server_->get_fork_x_mmps();
    // 目前 urdf 使用 effor to velocity，且限制速率上限，所以給力就馬上到達速率極限
    // 未來要換成 velocity to velocity 控制器，才能控制速度
    v.data = (mmps > 0) ? effort_hor_ :
             (mmps < 0) ? -effort_hor_ : 0.0;  // 轉成 NT

    return v;
  }

  std_msgs::Float64 get_fork_y_effort()
  {
    std_msgs::Float64 v;
    double mmps;
    mmps = mobox_server_->get_fork_y_mmps();
    // 目前 urdf 使用 effor to velocity，且限制速率上限，所以給力就馬上到達速率極限
    // 未來要換成 velocity to velocity 控制器，才能控制速度
    v.data = (mmps > 0) ? effort_hor_ :
             (mmps < 0) ? -effort_hor_ : 0.0;  // 轉成 NT

    return v;
  }

  std_msgs::Float64 get_fork_z_effort()
  {
    std_msgs::Float64 v;
    double mmps;
    mmps = mobox_server_->get_fork_z_mmps();
    // 目前 urdf 使用 effor to velocity，且限制速率上限，所以給力就馬上到達速率極限
    // 未來要換成 velocity to velocity 控制器，才能控制速度
    v.data = (mmps > 0) ? effort_ver_up_ :
             (mmps < 0) ? -effort_ver_down_ : effort_ver_stop_;  // 轉成 NT

    return v;
  }

  std_msgs::Float64 get_fork_rot_effort()
  {
    std_msgs::Float64 v;
    double mmps;
    mmps = mobox_server_->get_fork_rot_radps();
    // 目前 urdf 使用 effor to velocity，且限制速率上限，所以給力就馬上到達速率極限
    // 未來要換成 velocity to velocity 控制器，才能控制速度
    v.data = (mmps > 0) ? effort_hor_ :
             (mmps < 0) ? -effort_hor_ : 0.0;  // 轉成 NT

    return v;
  }

  void update_joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg)
  {
    // name: [joint_fork_rot, joint_fork_x, joint_fork_y, joint_fork_z, joint_steer, joint_trac]
    const uint8_t trac_index = 5;
    const uint8_t steer_index = 4;
    const uint8_t fork_y_index = 2;
    const uint8_t fork_z_index = 3;

    // trac: rad/s to mm/s
    double mmps = msg->velocity[trac_index] * mima_wheel_radius_;
    mobox_server_->set_sensor_trac_mmps(mmps);

    // steer: rad to cdeg
    double cdeg = msg->position[steer_index] * 180 / M_PI * 100.0;
    mobox_server_->set_sensor_steer_cdeg(cdeg);

    // fork_y: m to mm
    double y_mm = msg->position[fork_y_index] * 1000.0;
    mobox_server_->set_sensor_fork_y_mm(y_mm);

    // fork_z: m to mm
    double z_mm = msg->position[fork_z_index] * 1000.0;
    mobox_server_->set_sensor_fork_z_mm(y_mm);

    // x / rot 在米碼叉車沒有裝 sensor, 只能用時間目視控制, 這裡不作回授
  }

  // ---- private 變數區 -------------------------
  ros::NodeHandle nh_;
  Parameters parameters_;
  MoBoxServer::uptr mobox_server_ = nullptr;

  ros::Publisher trac_cmd_pub_;  // 發報控制物理量給模擬器中的 trac 機構
  ros::Publisher steer_cmd_pub_;  // 發報控制物理量給模擬器中的 steer 機構
  ros::Publisher fork_x_cmd_pub_;  // 發報控制物理量給模擬器中的 fork_x 機構
  ros::Publisher fork_y_cmd_pub_;  // 發報控制物理量給模擬器中的 fork_y 機構
  ros::Publisher fork_z_cmd_pub_;  // 發報控制物理量給模擬器中的 fork_z 機構
  ros::Publisher fork_rot_cmd_pub_;  // 發報控制物理量給模擬器中的 fork_rot 機構

  ros::Subscriber joint_states_sub_;  // 取得模擬組件的作動物理量

  const int loop_rate_ = 50;  // 單位: Hz, 發送 cmd_vel 的頻率
  const double mima_wheel_radius_ = 0.19;  // 米碼舵輪半徑設計尺寸, 單位：m
  const double effort_hor_ = 1.0;  // 單位: NT, 讓貨叉可以水平方向移動的力量
  const double effort_ver_stop_ = 0.5 * 9.8;  // 單位: NT, 讓貨叉可以垂直撐住不動的力量
                                              //  ex. 0.5 kg * 9.8 NT/kg (重力加速度)
  const double effort_ver_up_ = effort_ver_stop_ + 0.2;  // 單位: NT, 讓貨叉可以垂直上升的力量
  const double effort_ver_down_ = effort_ver_stop_ - 0.2;  // 單位: NT, 讓貨叉可以垂直下降的力量
};

}  // namespace amr


///////////////////////////////////////////////////////////////////////////////
int main(int argc, char *argv[])
{
  ros::init(argc, argv, amr::MbxMimaDriver::class_name_);
  ros::NodeHandle nh("~");

  amr::MbxMimaDriver ctrl(nh);

  if (!ctrl.prepare_parameter()) {
    ROS_ERROR_STREAM("[" << amr::MbxMimaDriver::class_name_ << "] prepare_parameter falied !!");
    return EXIT_FAILURE;
  }

  if (!ctrl.init()) {
    ROS_ERROR_STREAM("[" << amr::MbxMimaDriver::class_name_ << "] init falied !!");
    return EXIT_FAILURE;
  }

  if (!ctrl.start()) {
    // 按下 ctr+c, ros 已經不 ok, ROS_INFO() 失效，只能 cout or printf
    std::cout << "[" << amr::MbxMimaDriver::class_name_ << "] start() failed, exit !!" << std::endl;
    return EXIT_FAILURE;
  }

  return 0;
}
