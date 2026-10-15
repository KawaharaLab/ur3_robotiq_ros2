#pragma once
#include "acquisition_timing.hpp"
#include <array>
#include <cstdint>
#include <deque>
#include <mutex>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace acquisition_records
{
using ClockSample = std::array<int64_t, 3>;
struct Sample {uint64_t sequence; int64_t header_ns; uint64_t packet_byte_offset;};
struct Read
{
  uint64_t sequence{0}, byte_offset{0};
  int startup_attempt{0};
  ClockSample start{}, end{};
  std::vector<uint8_t> bytes;
  std::vector<Sample> samples;
  std::size_t pending_before{0}, pending_after{0};
  uint64_t discarded_bytes{0};
};
struct Snapshot
{
  std::vector<Read> reads;
  uint64_t dropped_reads{0}, dropped_samples{0};
};
class Buffer
{
public:
  explicit Buffer(std::size_t limit = 4096) : limit_(limit) {}
  void push(Read record)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (limit_ == 0) {++dropped_reads_; dropped_samples_ += record.samples.size(); return;}
    if (reads_.size() >= limit_) {
      ++dropped_reads_; dropped_samples_ += reads_.front().samples.size(); reads_.pop_front();
    }
    reads_.push_back(std::move(record));
  }
  Snapshot take(std::size_t maximum = 256)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    Snapshot result;
    while (!reads_.empty() && result.reads.size() < maximum) {
      result.reads.push_back(std::move(reads_.front())); reads_.pop_front();
    }
    result.dropped_reads = dropped_reads_; result.dropped_samples = dropped_samples_;
    return result;
  }
private:
  std::mutex mutex_;
  std::deque<Read> reads_;
  std::size_t limit_;
  uint64_t dropped_reads_{0}, dropped_samples_{0};
};
inline void clock_json(std::ostream & out, const ClockSample & value)
{out << '[' << value[0] << ',' << value[1] << ',' << value[2] << ']';}
inline std::string json(const Snapshot & snapshot, const std::string & stream_id,
  uint64_t batch_sequence, const std::string & node, const std::string & port,
  const std::string & frame, int clock_type, bool use_sim_time, double force_scale, double torque_scale,
  uint64_t failed_reads, uint64_t failed_samples)
{
  std::ostringstream out;
  out << std::setprecision(17)
      << "{\"schema_version\":1,\"kind\":\"mms101_reads\",\"stream_id\":"
      << acquisition_timing::quote(stream_id)
      << ",\"batch_seq\":" << batch_sequence << ",\"node\":" << acquisition_timing::quote(node)
      << ",\"port\":" << acquisition_timing::quote(port) << ",\"frame_id\":" << acquisition_timing::quote(frame)
      << ",\"clock_type\":" << clock_type
      << ",\"use_sim_time\":" << (use_sim_time ? "true" : "false")
      << ",\"host_monotonic_clock\":\"CLOCK_MONOTONIC\""
      << ",\"clock_sample_layout\":[\"monotonic_before_ns\",\"ros_ns\",\"monotonic_after_ns\"]"
      << ",\"sample_layout\":[\"sample_seq\",\"header_ns\",\"packet_byte_offset\"]"
      << ",\"configured_interval_us\":1000,\"force_scale\":" << force_scale << ",\"torque_scale\":" << torque_scale
      << ",\"dropped_reads_total\":" << snapshot.dropped_reads
      << ",\"dropped_samples_total\":" << snapshot.dropped_samples
      << ",\"failed_publish_reads_total\":" << failed_reads
      << ",\"failed_publish_samples_total\":" << failed_samples << ",\"reads\":[";
  bool first_read = true;
  const char hex[] = "0123456789abcdef";
  for (const auto & read : snapshot.reads) {
    if (!first_read) out << ',';
    first_read = false;
    out << "{\"read_seq\":" << read.sequence << ",\"startup_attempt\":" << read.startup_attempt
        << ",\"byte_offset\":" << read.byte_offset << ",\"read_start\":";
    clock_json(out, read.start); out << ",\"read_end\":"; clock_json(out, read.end);
    out << ",\"pending_before\":" << read.pending_before << ",\"pending_after\":" << read.pending_after
        << ",\"discarded_bytes\":" << read.discarded_bytes << ",\"bytes_hex\":\"";
    for (auto byte : read.bytes) out << hex[byte >> 4] << hex[byte & 15];
    out << "\",\"samples\":[";
    bool first_sample = true;
    for (const auto & sample : read.samples) {
      if (!first_sample) out << ',';
      first_sample = false;
      out << '[' << sample.sequence << ',' << sample.header_ns << ',' << sample.packet_byte_offset << ']';
    }
    out << "]}";
  }
  out << "]}";
  return out.str();
}
}  // namespace acquisition_records
