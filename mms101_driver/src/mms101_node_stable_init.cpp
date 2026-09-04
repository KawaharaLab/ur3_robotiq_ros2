/**
 * MMS101 Force Sensor Driver for ROS 2
 * Robust initialization and streaming version.
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <vector>

class MMS101Node : public rclcpp::Node
{
public:
  MMS101Node() : Node("mms101_driver")
  {
    declare_parameter("port", "/dev/ttyUSB1");
    declare_parameter("frame_id", "sensor_link");
    declare_parameter("force_scale", 0.001);
    declare_parameter("torque_scale", 0.00001);
    declare_parameter("initialization_retries", 3);
    declare_parameter("stream_start_timeout_ms", 2000);

    port_name_ = get_parameter("port").as_string();
    frame_id_ = get_parameter("frame_id").as_string();
    force_scale_ = get_parameter("force_scale").as_double();
    torque_scale_ = get_parameter("torque_scale").as_double();
    initialization_retries_ = static_cast<int>(
      get_parameter("initialization_retries").as_int());
    stream_start_timeout_ms_ = static_cast<int>(
      get_parameter("stream_start_timeout_ms").as_int());

    initialization_retries_ = std::max(initialization_retries_, 1);
    stream_start_timeout_ms_ = std::max(stream_start_timeout_ms_, 100);

    auto qos = rclcpp::SensorDataQoS().keep_last(2048);
    publisher_ = create_publisher<geometry_msgs::msg::WrenchStamped>(
      "force_torque", qos);

    if (!open_serial_port()) {
      throw std::runtime_error("Failed to open serial port");
    }

    bool startup_ok = false;
    for (int attempt = 1; attempt <= initialization_retries_; ++attempt) {
      RCLCPP_INFO(
        get_logger(), "Startup attempt %d/%d on %s",
        attempt, initialization_retries_, port_name_.c_str());

      stop_read_thread();
      first_packet_received_.store(false, std::memory_order_relaxed);
      rx_buffer_.clear();
      rx_pending_.store(0, std::memory_order_relaxed);

      if (configure_initialization_read()) {
        tcflush(serial_fd_, TCIOFLUSH);

        if (initialize_sensor() && configure_streaming_read()) {
          // Remove only residual command-response bytes before measurement parsing.
          tcflush(serial_fd_, TCIFLUSH);
          start_read_thread();

          if (wait_for_first_packet()) {
            startup_ok = true;
            break;
          }

          RCLCPP_WARN(
            get_logger(),
            "No valid measurement packet received within %d ms on %s",
            stream_start_timeout_ms_, port_name_.c_str());
        }
      }

      stop_read_thread();
      if (configure_initialization_read()) {
        send_stop_command_without_drain();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        tcflush(serial_fd_, TCIOFLUSH);
      }

      if (attempt < initialization_retries_) {
        RCLCPP_WARN(get_logger(), "Retrying after 500 ms...");
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
      }
    }

    if (!startup_ok) {
      throw std::runtime_error("MMS101 startup failed after all retry attempts");
    }

    diagnostics_timer_ = create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&MMS101Node::report_diagnostics, this));

    RCLCPP_INFO(
      get_logger(), "MMS101 streaming started: port=%s frame=%s",
      port_name_.c_str(), frame_id_.c_str());
  }

  ~MMS101Node() override
  {
    stop_read_thread();

    if (serial_fd_ >= 0) {
      if (configure_initialization_read()) {
        send_stop_command_without_drain();
      }
      close(serial_fd_);
      serial_fd_ = -1;
    }
  }

private:
  static constexpr std::size_t PACKET_SIZE = 25;
  static constexpr uint8_t HEADER0 = 0x00;
  static constexpr uint8_t HEADER1 = 0x17;

  int serial_fd_{-1};
  std::string port_name_;
  std::string frame_id_;
  double force_scale_{0.001};
  double torque_scale_{0.00001};
  int initialization_retries_{3};
  int stream_start_timeout_ms_{2000};

  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;

  std::thread read_thread_;
  std::atomic<bool> running_{false};
  std::atomic<bool> first_packet_received_{false};

  std::vector<uint8_t> rx_buffer_;

  std::atomic<uint64_t> bytes_received_{0};
  std::atomic<uint64_t> read_calls_{0};
  std::atomic<uint64_t> valid_packets_{0};
  std::atomic<uint64_t> discarded_bytes_{0};
  std::atomic<uint64_t> read_errors_{0};
  std::atomic<std::size_t> rx_pending_{0};

  uint64_t previous_bytes_{0};
  uint64_t previous_packets_{0};
  uint64_t previous_discarded_{0};
  uint64_t previous_errors_{0};

  bool open_serial_port()
  {
    serial_fd_ = open(port_name_.c_str(), O_RDWR | O_NOCTTY);
    if (serial_fd_ < 0) {
      RCLCPP_ERROR(
        get_logger(), "open(%s) failed: %s",
        port_name_.c_str(), std::strerror(errno));
      return false;
    }

    if (!configure_initialization_read()) {
      close(serial_fd_);
      serial_fd_ = -1;
      return false;
    }

    tcflush(serial_fd_, TCIOFLUSH);
    return true;
  }

  bool apply_common_serial_settings(termios & tty)
  {
    if (cfsetospeed(&tty, B1000000) != 0 ||
        cfsetispeed(&tty, B1000000) != 0)
    {
      RCLCPP_ERROR(get_logger(), "Failed to set baud rate: %s", std::strerror(errno));
      return false;
    }

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);
    tty.c_iflag = 0;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    return true;
  }

  bool configure_initialization_read()
  {
    if (serial_fd_ < 0) return false;

    termios tty{};
    if (tcgetattr(serial_fd_, &tty) != 0) {
      RCLCPP_ERROR(get_logger(), "tcgetattr failed: %s", std::strerror(errno));
      return false;
    }
    if (!apply_common_serial_settings(tty)) return false;

    // Initialization reads return after 100 ms even if no bytes arrive.
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
      RCLCPP_ERROR(
        get_logger(), "tcsetattr(initialization) failed: %s",
        std::strerror(errno));
      return false;
    }
    return true;
  }

  bool configure_streaming_read()
  {
    termios tty{};
    if (tcgetattr(serial_fd_, &tty) != 0) {
      RCLCPP_ERROR(get_logger(), "tcgetattr failed: %s", std::strerror(errno));
      return false;
    }
    if (!apply_common_serial_settings(tty)) return false;

    // Wake when at least one byte arrives; packet framing is handled in software.
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
      RCLCPP_ERROR(
        get_logger(), "tcsetattr(streaming) failed: %s",
        std::strerror(errno));
      return false;
    }
    return true;
  }

  bool write_all(const std::vector<uint8_t> & command)
  {
    std::size_t sent = 0;
    while (sent < command.size()) {
      const ssize_t n = write(
        serial_fd_, command.data() + sent, command.size() - sent);

      if (n > 0) {
        sent += static_cast<std::size_t>(n);
      } else if (n < 0 && errno == EINTR) {
        continue;
      } else {
        RCLCPP_ERROR(get_logger(), "serial write failed: %s", std::strerror(errno));
        return false;
      }
    }

    if (tcdrain(serial_fd_) != 0) {
      RCLCPP_WARN(get_logger(), "tcdrain failed: %s", std::strerror(errno));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    return true;
  }

  void drain_command_response(
    std::chrono::milliseconds quiet_period = std::chrono::milliseconds(20),
    std::chrono::milliseconds max_duration = std::chrono::milliseconds(200))
  {
    std::array<uint8_t, 512> temp{};
    const auto start_time = std::chrono::steady_clock::now();
    auto last_data_time = start_time;
    std::size_t total_bytes = 0;

    while (rclcpp::ok()) {
      const auto now = std::chrono::steady_clock::now();

      // Absolute timeout prevents an old continuous stream from blocking forever.
      if (now - start_time >= max_duration) {
        if (total_bytes > 0) {
          RCLCPP_WARN(
            get_logger(),
            "Response drain timeout: %zu bytes discarded (port=%s)",
            total_bytes, port_name_.c_str());
        }
        break;
      }

      const ssize_t n = read(serial_fd_, temp.data(), temp.size());
      if (n > 0) {
        total_bytes += static_cast<std::size_t>(n);
        last_data_time = std::chrono::steady_clock::now();
        continue;
      }

      if (n == 0) {
        if (std::chrono::steady_clock::now() - last_data_time >= quiet_period) {
          break;
        }
        continue;
      }

      if (errno == EINTR || errno == EAGAIN) continue;

      RCLCPP_WARN(
        get_logger(), "response drain read failed: %s",
        std::strerror(errno));
      break;
    }
  }

  bool send_command(
    const std::vector<uint8_t> & command,
    bool drain_response = true)
  {
    if (!write_all(command)) return false;
    if (drain_response) drain_command_response();
    return true;
  }

  void send_stop_command_without_drain()
  {
    const std::vector<uint8_t> stop_command{0x54, 0x01, 0x33};
    if (!write_all(stop_command)) {
      RCLCPP_WARN(
        get_logger(), "Failed to write stop command on %s",
        port_name_.c_str());
    }
  }

  bool initialize_sensor()
  {
    RCLCPP_INFO(get_logger(), "Initializing MMS101 on %s", port_name_.c_str());

    RCLCPP_INFO(get_logger(), "[Init 0] Stop previous measurement stream");
    send_stop_command_without_drain();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    drain_command_response(
      std::chrono::milliseconds(20),
      std::chrono::milliseconds(200));
    tcflush(serial_fd_, TCIOFLUSH);

    RCLCPP_INFO(get_logger(), "[Init 1] Send command 0x10");
    if (!send_command({0x54, 0x02, 0x10, 0x00})) return false;

    RCLCPP_INFO(get_logger(), "[Init 2] Configure 0x36 address 0x00");
    if (!send_command({0x54, 0x03, 0x36, 0x00, 0x01})) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    RCLCPP_INFO(get_logger(), "[Init 3] Configure 0x36 address 0x05");
    if (!send_command({0x54, 0x03, 0x36, 0x05, 0x01})) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    for (uint8_t axis = 0; axis < 6; ++axis) {
      RCLCPP_INFO(
        get_logger(), "[Init 4] Configure axis %u",
        static_cast<unsigned int>(axis));

      if (!send_command({0x54, 0x02, 0x1C, axis})) return false;
      if (!send_command({0x53, 0x02, 0x57, 0x94})) return false;
      std::this_thread::sleep_for(std::chrono::milliseconds(15));
    }

    RCLCPP_INFO(get_logger(), "[Init 5] Apply settings");
    if (!send_command({0x54, 0x01, 0xB0})) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    RCLCPP_INFO(get_logger(), "[Init 6] Set interval to 1000 us");
    if (!send_command({0x54, 0x04, 0x43, 0x00, 0x03, 0xE8})) return false;

    tcflush(serial_fd_, TCIFLUSH);

    RCLCPP_INFO(get_logger(), "[Init 7] Start continuous measurement");
    // Do not drain here: subsequent bytes are measurement packets.
    if (!send_command({0x54, 0x02, 0x23, 0x00}, false)) return false;

    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    RCLCPP_INFO(get_logger(), "[Init 8] Initialization commands completed");
    return true;
  }

  void start_read_thread()
  {
    if (running_.load(std::memory_order_relaxed)) return;
    running_.store(true, std::memory_order_relaxed);
    read_thread_ = std::thread(&MMS101Node::read_loop, this);
  }

  void stop_read_thread()
  {
    running_.store(false, std::memory_order_relaxed);
    if (read_thread_.joinable()) read_thread_.join();
  }

  bool wait_for_first_packet()
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::milliseconds(stream_start_timeout_ms_);

    while (std::chrono::steady_clock::now() < deadline) {
      if (first_packet_received_.load(std::memory_order_relaxed)) return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return first_packet_received_.load(std::memory_order_relaxed);
  }

  static int32_t parse_24bit_be(const uint8_t * data)
  {
    int32_t value =
      (static_cast<int32_t>(data[0]) << 16) |
      (static_cast<int32_t>(data[1]) << 8) |
      static_cast<int32_t>(data[2]);

    if ((value & 0x800000) != 0) {
      value |= static_cast<int32_t>(0xFF000000);
    }
    return value;
  }

  void process_packet(const uint8_t * packet)
  {
    geometry_msgs::msg::WrenchStamped message;
    message.header.stamp = now();
    message.header.frame_id = frame_id_;

    message.wrench.force.x = parse_24bit_be(&packet[4]) * force_scale_;
    message.wrench.force.y = parse_24bit_be(&packet[7]) * force_scale_;
    message.wrench.force.z = parse_24bit_be(&packet[10]) * force_scale_;
    message.wrench.torque.x = parse_24bit_be(&packet[13]) * torque_scale_;
    message.wrench.torque.y = parse_24bit_be(&packet[16]) * torque_scale_;
    message.wrench.torque.z = parse_24bit_be(&packet[19]) * torque_scale_;

    publisher_->publish(message);
    valid_packets_.fetch_add(1, std::memory_order_relaxed);
    first_packet_received_.store(true, std::memory_order_relaxed);
  }

  void parse_available_packets()
  {
    while (rx_buffer_.size() >= 2) {
      std::size_t header_pos = rx_buffer_.size();

      for (std::size_t i = 0; i + 1 < rx_buffer_.size(); ++i) {
        if (rx_buffer_[i] == HEADER0 && rx_buffer_[i + 1] == HEADER1) {
          header_pos = i;
          break;
        }
      }

      if (header_pos == rx_buffer_.size()) {
        const bool keep_last_zero = rx_buffer_.back() == HEADER0;
        const std::size_t remove_count =
          rx_buffer_.size() - (keep_last_zero ? 1U : 0U);

        discarded_bytes_.fetch_add(
          static_cast<uint64_t>(remove_count), std::memory_order_relaxed);
        rx_buffer_.erase(
          rx_buffer_.begin(),
          rx_buffer_.begin() + static_cast<std::ptrdiff_t>(remove_count));
        rx_pending_.store(rx_buffer_.size(), std::memory_order_relaxed);
        return;
      }

      if (header_pos > 0) {
        discarded_bytes_.fetch_add(
          static_cast<uint64_t>(header_pos), std::memory_order_relaxed);
        rx_buffer_.erase(
          rx_buffer_.begin(),
          rx_buffer_.begin() + static_cast<std::ptrdiff_t>(header_pos));
      }

      if (rx_buffer_.size() < PACKET_SIZE) {
        rx_pending_.store(rx_buffer_.size(), std::memory_order_relaxed);
        return;
      }

      process_packet(rx_buffer_.data());
      rx_buffer_.erase(
        rx_buffer_.begin(),
        rx_buffer_.begin() + static_cast<std::ptrdiff_t>(PACKET_SIZE));
      rx_pending_.store(rx_buffer_.size(), std::memory_order_relaxed);
    }
  }

  void read_loop()
  {
    std::array<uint8_t, 512> temp{};
    rx_buffer_.reserve(4096);

    while (running_.load(std::memory_order_relaxed) && rclcpp::ok()) {
      const ssize_t n = read(serial_fd_, temp.data(), temp.size());
      read_calls_.fetch_add(1, std::memory_order_relaxed);

      if (n > 0) {
        bytes_received_.fetch_add(
          static_cast<uint64_t>(n), std::memory_order_relaxed);
        rx_buffer_.insert(
          rx_buffer_.end(), temp.begin(), temp.begin() + n);
        parse_available_packets();

        if (rx_buffer_.size() > 64 * 1024) {
          RCLCPP_ERROR(
            get_logger(), "Serial receive buffer exceeded 64 KiB; clearing it");
          discarded_bytes_.fetch_add(
            static_cast<uint64_t>(rx_buffer_.size()),
            std::memory_order_relaxed);
          rx_buffer_.clear();
          rx_pending_.store(0, std::memory_order_relaxed);
        }
      } else if (n == 0) {
        continue;
      } else if (errno == EINTR || errno == EAGAIN) {
        continue;
      } else {
        read_errors_.fetch_add(1, std::memory_order_relaxed);
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "serial read failed: %s", std::strerror(errno));
      }
    }
  }

  void report_diagnostics()
  {
    const uint64_t bytes = bytes_received_.load(std::memory_order_relaxed);
    const uint64_t packets = valid_packets_.load(std::memory_order_relaxed);
    const uint64_t discarded = discarded_bytes_.load(std::memory_order_relaxed);
    const uint64_t errors = read_errors_.load(std::memory_order_relaxed);

    const uint64_t byte_rate = bytes - previous_bytes_;
    const uint64_t packet_rate = packets - previous_packets_;
    const uint64_t discarded_delta = discarded - previous_discarded_;
    const uint64_t error_delta = errors - previous_errors_;

    previous_bytes_ = bytes;
    previous_packets_ = packets;
    previous_discarded_ = discarded;
    previous_errors_ = errors;

    RCLCPP_INFO(
      get_logger(),
      "[MMS101 diag] port=%s packets=%lu/s bytes=%lu/s "
      "discarded=%lu/s read_errors=%lu/s rx_pending=%zu",
      port_name_.c_str(),
      static_cast<unsigned long>(packet_rate),
      static_cast<unsigned long>(byte_rate),
      static_cast<unsigned long>(discarded_delta),
      static_cast<unsigned long>(error_delta),
      rx_pending_.load(std::memory_order_relaxed));
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<MMS101Node>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("mms101_driver"),
      "Fatal error: %s", error.what());
  }
  rclcpp::shutdown();
  return 0;
}
