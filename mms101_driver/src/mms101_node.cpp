/**
 * MMS101 Force Sensor Driver for ROS 2
 * Modified for High-Frequency (1000Hz) & High-Precision Synchronization
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <vector>
#include <cstring>
#include <thread>
#include <atomic>

class MMS101Node : public rclcpp::Node
{
public:
  MMS101Node() : Node("mms101_driver")
  {
    this->declare_parameter("port", "/dev/ttyUSB1");
    this->declare_parameter("frame_id", "sensor_link");
    this->declare_parameter("force_scale", 0.001); 
    this->declare_parameter("torque_scale", 0.00001);

    port_name_ = this->get_parameter("port").as_string();
    frame_id_ = this->get_parameter("frame_id").as_string();
    force_scale_ = this->get_parameter("force_scale").as_double();
    torque_scale_ = this->get_parameter("torque_scale").as_double();

    publisher_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("force_torque", 100);

    if (open_serial_port()) {
      if (initialize_sensor()) {
        RCLCPP_INFO(this->get_logger(), "Sensor initialized successfully. Starting read thread.");
        
        // 計測開始前にOSのシリアル受信バッファに溜まった古いゴミデータを消去
        tcflush(serial_fd_, TCIFLUSH);
        
        running_ = true;
        read_thread_ = std::thread(&MMS101Node::read_loop, this);
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize sensor.");
      }
    }
  }

  ~MMS101Node() {
    running_ = false;
    if (read_thread_.joinable()) {
      read_thread_.join();
    }
    stop_measurement();
    if (serial_fd_ != -1) close(serial_fd_);
  }

private:
  int serial_fd_ = -1;
  std::string port_name_;
  std::string frame_id_;
  double force_scale_;
  double torque_scale_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr publisher_;
  
  std::thread read_thread_;
  std::atomic<bool> running_{false};

  // 時刻同期用変数
  bool is_first_packet_ = true;
  rclcpp::Time pc_base_time_;
  uint32_t sensor_base_time_ = 0;

  bool open_serial_port()
  {
    serial_fd_ = open(port_name_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (serial_fd_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "Error opening %s: %s", port_name_.c_str(), strerror(errno));
      return false;
    }

    struct termios tty;
    if (tcgetattr(serial_fd_, &tty) != 0) return false;

    cfsetospeed(&tty, B1000000);
    cfsetispeed(&tty, B1000000);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    
    // 【重要】25バイト揃うまでread関数でブロックして待機する
    tty.c_cc[VMIN]  = 25; 
    tty.c_cc[VTIME] = 1;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) return false;
    return true;
  }

  void send_command(const std::vector<uint8_t>& cmd) {
    write(serial_fd_, cmd.data(), cmd.size());
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

  void flush_response() {
    uint8_t buf[64];
    read(serial_fd_, buf, sizeof(buf));
  }

  bool initialize_sensor()
  {
    RCLCPP_INFO(this->get_logger(), "Initializing MMS101...");

    send_command({0x54, 0x02, 0x10, 0x00});
    flush_response();

    send_command({0x54, 0x03, 0x36, 0x00, 0x01});
    flush_response();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    send_command({0x54, 0x03, 0x36, 0x05, 0x01});
    flush_response();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    for (uint8_t axis = 0; axis < 6; axis++) {
      send_command({0x54, 0x02, 0x1C, axis});
      flush_response();
      send_command({0x53, 0x02, 0x57, 0x94});
      flush_response();
      std::this_thread::sleep_for(std::chrono::milliseconds(15));
    }

    send_command({0x54, 0x01, 0xB0});
    flush_response();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // 【重要】Interval Measureコマンドを 1ms間隔 (1000Hz) に設定
    // 1000(us) = 0x03E8
    send_command({0x54, 0x04, 0x43, 0x00, 0x03, 0xE8}); 
    flush_response();

    send_command({0x54, 0x02, 0x23, 0x00});
    
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    flush_response();

    return true;
  }

  void stop_measurement() {
    send_command({0x54, 0x01, 0x33});
  }

  int32_t parse_24bit_be(const uint8_t* data) {
    int32_t val = (data[0] << 16) | (data[1] << 8) | data[2];
    if (val & 0x800000) {
      val |= 0xFF000000;
    }
    return val;
  }

  void read_loop()
  {
    uint8_t buf[25];
    while (running_ && rclcpp::ok()) {
      int n = read(serial_fd_, buf, 25);

      if (n == 25) {
          // Status Check & Header Check
          if (buf[0] != 0x00 || buf[1] != 0x17) continue;

          // センサの24bitタイムスタンプ(ミリ秒)を抽出
          uint32_t current_sensor_time = (buf[22] << 16) | (buf[23] << 8) | buf[24];
          rclcpp::Time exact_stamp;

          // ベースライン・オフセット補正による高精度タイムスタンプの計算
          if (is_first_packet_) {
              pc_base_time_ = this->now();
              sensor_base_time_ = current_sensor_time;
              exact_stamp = pc_base_time_;
              is_first_packet_ = false;
          } else {
              uint32_t delta_ms;
              if (current_sensor_time >= sensor_base_time_) {
                  delta_ms = current_sensor_time - sensor_base_time_;
              } else {
                  // 24bitカウンタ(16,777,215)のラップアラウンド対策
                  delta_ms = (0xFFFFFF - sensor_base_time_) + current_sensor_time + 1;
              }
              exact_stamp = pc_base_time_ + rclcpp::Duration(std::chrono::milliseconds(delta_ms));
          }

          int32_t raw_fx = parse_24bit_be(&buf[4]);
          int32_t raw_fy = parse_24bit_be(&buf[7]);
          int32_t raw_fz = parse_24bit_be(&buf[10]);
          int32_t raw_mx = parse_24bit_be(&buf[13]);
          int32_t raw_my = parse_24bit_be(&buf[16]);
          int32_t raw_mz = parse_24bit_be(&buf[19]);

          auto msg = geometry_msgs::msg::WrenchStamped();
          msg.header.stamp = exact_stamp;
          msg.header.frame_id = frame_id_;

          msg.wrench.force.x = raw_fx * force_scale_;
          msg.wrench.force.y = raw_fy * force_scale_;
          msg.wrench.force.z = raw_fz * force_scale_;
          msg.wrench.torque.x = raw_mx * torque_scale_;
          msg.wrench.torque.y = raw_my * torque_scale_;
          msg.wrench.torque.z = raw_mz * torque_scale_;

          publisher_->publish(msg);
      }
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MMS101Node>());
  rclcpp::shutdown();
  return 0;
}