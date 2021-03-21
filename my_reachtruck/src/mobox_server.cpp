/*
 * mobox_server.cpp
 *
 *  Created on: 2021年3月20日
 *      Author: kcchang
 */
#include "mobox_server.h"

#include <cmath>
#include <vector>
#include <stdio.h>
#include <util_crc.h>
#include <ros/ros.h>


namespace amr {

MoBoxServer::MoBoxServer(const std::string &port, const int baudrate, const int read_timeout_ms)
    : port_(port), baudrate_(baudrate), timeout_(read_timeout_ms)
{
}

MoBoxServer::~MoBoxServer()
{
  // 停車 & 停止通訊
  stop();
}

bool MoBoxServer::start()
{
  if (serial_.isOpen()) {
    serial_.close();
  }

  try {
    serial_.setPort(port_);
    serial_.setBaudrate(baudrate_);
    serial::Timeout to = serial::Timeout::simpleTimeout(timeout_);
    serial_.setTimeout(to);
    serial_.open();
  }
  catch (serial::IOException& e) {
    ROS_ERROR_STREAM("[" << class_name_ << "] Unable to open port " << port_);
    return false;
  }

  return true;
}

void MoBoxServer::start_listen()
{
  is_thread_running_ = true;
  read_thread_ = std::thread([this] {
    read_reply_loop();
  });
}

// @brief  相對於 start(), 停止本物件
bool MoBoxServer::stop()
{
  stop_listen();

  // 停止通訊
  serial_.close();

  return true;
}

void MoBoxServer::stop_listen()
{
  is_thread_running_ = false;  // stop thread loop
  read_thread_.join();
}

bool MoBoxServer::send_packet()
{
  auto lo_byte =
      [](double x)->int8_t {int16_t y = static_cast<int16_t>(x); return static_cast<int8_t>(y & 0x00FF);};

  auto hi_byte =
      [](double x)->int8_t {int16_t y = static_cast<int16_t>(x); return static_cast<int8_t>((y & 0xFF00)>>8);};

  std::vector<uint8_t> pkt;
  // 打包、發送運動包 ---------------------------------------
  uint8_t m_pkt[sizeof_packet_] = {
      default_id_, motion_fc_, default_len_,
      zero_byte_, zero_byte_,  // [3,4] FBB
      zero_byte_, zero_byte_,
      zero_byte_, zero_byte_,
      zero_byte_, zero_byte_,  // [ 9,10] sensor trac
      zero_byte_, zero_byte_,  // [11,12] sensor steer
      zero_byte_, zero_byte_,  // [13,14] crc L/H
      default_cmd_end_
  };

  // 配置封包
  m_pkt[9] = lo_byte(target_trac_mmps_);
  m_pkt[10] = hi_byte(target_trac_mmps_);
  m_pkt[11] = lo_byte(target_steer_cdeg_);
  m_pkt[12] = hi_byte(target_steer_cdeg_);

  uint16_t crc = util_crc_calc(m_pkt, static_cast<uint16_t>(sizeof_packet_));
  m_pkt[13] = (crc & 0x00FF);
  m_pkt[14] = ((crc & 0xFF00) >> 8);

  pkt.assign(&m_pkt[0], &m_pkt[sizeof_packet_-1]);

  try {
    size_t sent_size = serial_.write(pkt);
    if (sent_size < pkt.size()) {
      ROS_ERROR_STREAM_THROTTLE(1, "[" << class_name_ << "] send_packet()::serial_.write() motion packet failed !");
    }
  }
  catch (std::exception &err) {
    ROS_ERROR_STREAM("[" << class_name_ << "] send_packet() motion exception !");
    ROS_ERROR_STREAM("[" << class_name_ << "]" << err.what());
    return false;
  }

  // 打包、發送貨叉包 ---------------------------------------
  uint8_t f_pkt[sizeof_packet_] = {
      default_id_, fork_fc_, default_len_,
      zero_byte_, zero_byte_,  // [3,4] reserve
      zero_byte_, zero_byte_,  // [5,6] x_mm
      zero_byte_, zero_byte_,  // [7,8] y_mm
      zero_byte_, zero_byte_,  // [ 9,10] z_mm
      zero_byte_, zero_byte_,  // [11,12] reserve
      zero_byte_, zero_byte_,  // [13,14] crc L/H
      default_cmd_end_
  };

  // 配置封包
  f_pkt[9] = lo_byte(sensor_steer_fork_y_mm_);
  f_pkt[10] = hi_byte(sensor_steer_fork_y_mm_);
  f_pkt[11] = lo_byte(sensor_steer_fork_z_mm_);
  f_pkt[12] = hi_byte(sensor_steer_fork_z_mm_);

  crc = util_crc_calc(f_pkt, static_cast<uint16_t>(sizeof_packet_));
  f_pkt[13] = (crc & 0x00FF);
  f_pkt[14] = ((crc & 0xFF00) >> 8);

  pkt.assign(&f_pkt[0], &f_pkt[sizeof_packet_-1]);

  try {
    size_t sent_size = serial_.write(pkt);
    if (sent_size < pkt.size()) {
      ROS_ERROR_STREAM_THROTTLE(1, "[" << class_name_ << "] send_packet()::serial_.write() fork packet failed !");
    }
  }
  catch (std::exception &err) {
    ROS_ERROR_STREAM("[" << class_name_ << "] send_packet() fork exception !");
    ROS_ERROR_STREAM("[" << class_name_ << "]" << err.what());
    return false;
  }

  return true;
}

double MoBoxServer::get_trac_mmps()
{
  return target_trac_mmps_;
}

double MoBoxServer::get_steer_cdeg()
{
  return target_steer_cdeg_;
}

double MoBoxServer::get_fork_x_mmps()
{
  return target_fork_x_mmps_;
}

double MoBoxServer::get_fork_y_mmps()
{
  return target_fork_y_mmps_;
}

double MoBoxServer::get_fork_z_mmps()
{
  return target_fork_z_mmps_;
}

double MoBoxServer::get_fork_rot_radps()
{
  return target_fork_rot_radps_;
}

void MoBoxServer::set_sensor_trac_mmps(double mmps)
{
  sensor_trac_mmps_ = mmps;
}

void MoBoxServer::set_sensor_steer_cdeg(double cdeg)
{
  sensor_steer_cdeg_ = cdeg;
}

void MoBoxServer::set_sensor_fork_y_mm(double mm)
{
  sensor_steer_fork_y_mm_ = mm;
}

void MoBoxServer::set_sensor_fork_z_mm(double mm)
{
  sensor_steer_fork_z_mm_ = mm;
}

bool MoBoxServer::wait_for_reply()
{
  if (!serial_.isOpen()) {
    return false;
  }

  if (!serial_.waitReadable()) {
    return false;
  }

  while (serial_.available() < static_cast<size_t>(sizeof_packet_)) {
    serial_.waitByteTimes(sizeof_packet_);
  }

  // 持續監控 ID byte 發生
  bool id_got = false;
  try {
    uint8_t id;
    for (int i = 0; i < sizeof_packet_; i++) {
      serial_.read(&id, 1);
      if (id == default_id_) {
        id_got = true;
        break;
      }
    }
  }
  catch (std::exception &e) {
    ROS_ERROR_STREAM("[" << class_name_ << "] " << e.what());
    return false;
  }

  if (!id_got) {
    return false;
  }

  // 抓取後續 bytes
  uint8_t buf[sizeof_packet_];
  buf[0] = default_id_;
  try {
    size_t len = serial_.read(&buf[1], sizeof_packet_ - 1);
    if (len != static_cast<size_t>(sizeof_packet_ - 1)) {
      ROS_WARN_STREAM("[" << class_name_ << "] packet short, read size " << len);
      return false;
    }
  }
  catch (serial::SerialException &e) {
    ROS_ERROR_STREAM("[" << class_name_ << "] " << e.what());
    return false;
  }

  // 確認 crc 正確
  uint16_t crc = util_crc_calc(buf, sizeof_packet_ - 3);

  if ((buf[sizeof_packet_ - 3] != (crc & 0xFF)) || (buf[sizeof_packet_ - 2] != ((crc >> 8) & 0xFF))) {
    ROS_ERROR_STREAM("[" << class_name_ << "] crc error");
    return false;
  }

  packet_recieved_.assign(buf, buf + sizeof_packet_);
  return true;
}

bool MoBoxServer::parse_trac_steer_cmd(double *mmps, double *cdeg)
{
  if (packet_recieved_.size() < static_cast<size_t>(sizeof_packet_)) {
    // 資料不可信，回傳 0 以策安全
    (*mmps) = 0.0;
    (*cdeg) = 0.0;
    return false;
  }

  if (packet_recieved_[1] != motion_fc_) {
    return false;
  }

  // 先轉成有號定點數, 再轉成浮點數
  int16_t t = packet_recieved_[9] | (packet_recieved_[10] << 8);
  int16_t s = packet_recieved_[11] | (packet_recieved_[12] << 8);
  (*mmps) = static_cast<double>(t);
  (*cdeg) = static_cast<double>(s);

  return true;
}

bool MoBoxServer::parse_fork_cmd(double *x_mmps, double *y_mmps, double *z_mmps, double *rot_radps)
{
  (*x_mmps) = 0.0;
  (*y_mmps) = 0.0;
  (*z_mmps) = 0.0;
  (*rot_radps) = 0.0;

  if (packet_recieved_.size() < static_cast<size_t>(sizeof_packet_)) {
    // 資料不可信
    return false;
  }

  if (packet_recieved_[1] != fork_fc_) {
    return false;
  }

  // ** 假設 pump 2000 rpm 對應 200 mm/s, 閥門 255 對應下降 20 cm/s **
  int16_t valve = packet_recieved_[5] | (packet_recieved_[6] << 8);
  int16_t pump_rpm = packet_recieved_[7] | (packet_recieved_[8] << 8);
  int16_t swptn = packet_recieved_[9] | (packet_recieved_[10] << 8);

  double pump_mmps = pump_rpm * pump_rpm_to_mmps_;
  double valve_mmps = valve * valve_to_mmps_;
  double pump_radps = pump_rpm * pump_rpm_to_radps_;

  // 根據 pattern 決定 x/y/z/rot 哪個有速度:
  // bit: [0]下 [1]後 [2]前 [3]右 [4]左 [5]傾 [6]仰 [7]升
  if (((swptn >> 0) & 0x01) == 0x01) {
    *z_mmps = -valve_mmps;
  }
  else if (((swptn >> 1) & 0x01) == 0x01) {
    *y_mmps = -pump_mmps;
  }
  else if (((swptn >> 2) & 0x01) == 0x01) {
    *y_mmps = pump_mmps;
  }
  else if (((swptn >> 3) & 0x01) == 0x01) {
    *x_mmps = pump_mmps;
  }
  else if (((swptn >> 4) & 0x01) == 0x01) {
    *x_mmps = -pump_mmps;
  }
  else if (((swptn >> 5) & 0x01) == 0x01) {
    *rot_radps = -pump_radps;
  }
  else if (((swptn >> 6) & 0x01) == 0x01) {
    *rot_radps = pump_radps;
  }
  else if (((swptn >> 7) & 0x01) == 0x01) {
    *z_mmps = valve_mmps;
  }
  else {
    // do nothing
  }

  return true;
}

void MoBoxServer::read_reply_loop()
{
  ROS_INFO_ONCE("[%s] start read_can_thread_loop()", class_name_.c_str());

  while (ros::ok()) {
    if (wait_for_reply()) {
      if (parse_trac_steer_cmd(&target_trac_mmps_, &target_steer_cdeg_)) {
        ROS_INFO_THROTTLE(1, "[%s](1Hz) trac cmd (m/s): %lf", class_name_.c_str(), target_trac_mmps_ / 1000.0);
        ROS_INFO_THROTTLE(1, "[%s](1Hz) steer cmd (deg): %lf", class_name_.c_str(), target_steer_cdeg_ / 100.0);
      }

      if (parse_fork_cmd(&target_fork_x_mmps_, &target_fork_y_mmps_, &target_fork_z_mmps_,
          &target_fork_rot_radps_)) {
        ROS_INFO_THROTTLE(1, "[%s](1Hz) fork x/y/z/rot (m/s): %lf %lf %lf %lf", class_name_.c_str(),
            target_fork_x_mmps_, target_fork_y_mmps_, target_fork_z_mmps_, target_fork_rot_radps_);
      }
    }  // wait_for_reply
  }  // while
}

}  // namespace amr
