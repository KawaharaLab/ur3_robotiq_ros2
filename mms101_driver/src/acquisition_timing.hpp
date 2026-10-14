#pragma once

#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string>

namespace acquisition_timing
{
struct Stats
{
  uint64_t count{0};
  double sum{0}, minimum{0}, maximum{0};
  void add(double value)
  {
    if (count == 0) minimum = maximum = value;
    if (value < minimum) minimum = value;
    if (value > maximum) maximum = value;
    ++count;
    sum += value;
  }
  std::string json() const
  {
    if (!count) return "{\"count\":0,\"mean\":null,\"min\":null,\"max\":null}";
    std::ostringstream out;
    out << std::setprecision(12) << "{\"count\":" << count
        << ",\"mean\":" << sum / count << ",\"min\":" << minimum
        << ",\"max\":" << maximum << "}";
    return out.str();
  }
};

inline std::string quote(const std::string & text)
{
  std::ostringstream out;
  out << '"';
  for (const unsigned char character : text) {
    if (character == '"' || character == '\\') out << '\\' << character;
    else if (character < 0x20) {
      out << "\\u" << std::hex << std::setw(4) << std::setfill('0')
          << static_cast<unsigned int>(character) << std::dec;
    } else out << character;
  }
  out << '"';
  return out.str();
}

struct Window
{
  Stats read_block_ms, read_return_interval_ms, bytes_per_read,
    packets_per_read, parse_publish_ms, tty_unread_bytes;
  uint64_t positive_reads{0}, multi_packet_reads{0};
  int64_t last_read_end_ros_ns{0};
  void add(double block_ms, double interval_ms, double bytes, double packets,
    double parse_ms, int unread, int64_t read_end_ros_ns)
  {
    ++positive_reads;
    if (packets > 1) ++multi_packet_reads;
    read_block_ms.add(block_ms);
    if (interval_ms >= 0) read_return_interval_ms.add(interval_ms);
    bytes_per_read.add(bytes);
    packets_per_read.add(packets);
    parse_publish_ms.add(parse_ms);
    if (unread >= 0) tty_unread_bytes.add(unread);
    last_read_end_ros_ns = read_end_ros_ns;
  }
  std::string json(double seconds, int64_t report_ros_ns, const std::string & node,
    const std::string & port, const std::string & frame) const
  {
    std::ostringstream out;
    out << std::setprecision(12)
        << "{\"schema_version\":1,\"kind\":\"mms101\",\"node\":" << quote(node)
        << ",\"port\":" << quote(port) << ",\"frame_id\":" << quote(frame)
        << ",\"report_ros_ns\":" << report_ros_ns
        << ",\"last_read_end_ros_ns\":" << last_read_end_ros_ns
        << ",\"window_seconds\":" << seconds
        << ",\"positive_reads\":" << positive_reads
        << ",\"multi_packet_reads\":" << multi_packet_reads
        << ",\"metrics\":{\"read_block_ms\":" << read_block_ms.json()
        << ",\"read_return_interval_ms\":" << read_return_interval_ms.json()
        << ",\"bytes_per_read\":" << bytes_per_read.json()
        << ",\"packets_per_read\":" << packets_per_read.json()
        << ",\"parse_publish_ms\":" << parse_publish_ms.json()
        << ",\"tty_unread_bytes\":" << tty_unread_bytes.json()
        << "},\"note\":\"Host-side diagnostics; blocking read includes waiting for data. Not ADC timestamps or calibrated sensor latency.\"}";
    return out.str();
  }
};
}  // namespace acquisition_timing
