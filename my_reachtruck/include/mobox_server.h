/*
 * mobox_server.h
 *
 *  Created on: 2021年3月20日
 *      Author: kc.chang
 */

#ifndef MY_REACHTRUCK_INCLUDE_MOBOX_SERVER_H_
#define MY_REACHTRUCK_INCLUDE_MOBOX_SERVER_H_
#include <memory>
#include <vector>
#include <string>
#include <thread>
#include <stdint.h>
#include <serial/serial.h>

#define DUMP_PKT 0  // for debug

/// @file  本物件接收 ros base control 透過 RS232 傳來的封包.
///         通訊協定詳見 motionbox project:: doc/15_mobox 通訊協定.pdf

namespace amr {
class MoBoxServer {
public:
  using uptr = std::unique_ptr<MoBoxServer>;

  static uptr create_instance(const std::string &port, const int baudrate, const int read_timeout_ms)
  {
    return std::make_unique < MoBoxServer > (port, baudrate, read_timeout_ms);
  }

  MoBoxServer(const std::string &port, const int baudrate, const int read_timeout_ms);
  ~MoBoxServer();

  /// 啟動本物件
  bool start();

  /// 啟動非同步 thread 讀取封包
  void start_listen();

  /// 停止本物件
  bool stop();

  /// 停止非同步 thread 讀取封包
  void stop_listen();

  /// 發送封裝好的封包
  bool send_packet();

  /// 給外部取得最新解析到的輪速、舵角命令
  void get_trac_steer_cmd(double *mmps, double *cdeg);

  /// 給外部取得最新解析到的貨叉命令
  void get_fork_cmd(double *x_eff, double *y_eff, double *z_eff, double *rot_eff);

  /// 設置 trac sensor 讀到的 mmps
  void set_sensor_trac_mmps(double mmps);

  /// 設置 steer sensor 讀到的 cdeg
  void set_sensor_steer_cdeg(double cdeg);

  /// 設置 fork_y sensor 讀到的 mm
  void set_sensor_fork_y_mm(double mm);

  /// 設置 fork_z sensor 讀到的 mm
  void set_sensor_fork_z_mm(double mm);

private:
  /// 等待送來的封包, return true: 有等到, false: timeout 了，也沒等到
  bool wait_for_reply();

  /// 從封包取得輪速、舵角命令
  bool parse_trac_steer_cmd(double *mmps, double *cdeg);

  /// 從封包取得貨叉命令
  bool parse_fork_cmd(double *x_eff, double *y_eff, double *z_eff, double *rot_eff);

  /// 受 thread 帶動讀包
  void read_reply_loop();

  const std::string class_name_ = "MoBoxServer";

  serial::Serial serial_;
  const std::string port_ = "";
  const uint32_t baudrate_;
  const int timeout_;

  bool is_thread_running_ = false;
  std::thread read_thread_;

  // 經過呼叫 wait_for_reply() 後，取得的命令封包
  std::vector<uint8_t> packet_recieved_;
  const size_t sizeof_packet_ = 16;
  const uint8_t default_id_ = 0xFD;

  const uint8_t motion_fc_ = 0x01;
  const uint8_t fork_fc_ = 0x04;
  const uint8_t health_fc_ = 0x06;

  const uint8_t default_len_ = 0x05;
  const uint8_t default_cmd_end_ = 0x0A;
  const uint8_t zero_byte_ = 0x00;

  // 設置到模擬器的目標物理量
  double target_trac_mmps_ = 0.0;
  double target_steer_cdeg_ = 0.0;
  double target_fork_x_effort_ = 0.0;  // 單位：nt
  double target_fork_y_effort_ = 0.0;  // 單位：nt
  double target_fork_z_effort_ = 0.0;  // 單位：nt
  double target_fork_rot_effort_ = 0.0;  // 單位：nt

  // 從模擬器取得的回授物理量
  double sensor_trac_mmps_ = 0.0;
  double sensor_steer_cdeg_ = 0.0;
  double sensor_steer_fork_y_mm_ = 0.0;
  double sensor_steer_fork_z_mm_ = 0.0;
};

}  // namespace amr



#endif /* MY_REACHTRUCK_INCLUDE_MOBOX_SERVER_H_ */
