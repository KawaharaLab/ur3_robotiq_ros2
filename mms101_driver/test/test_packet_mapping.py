"""Compile the actual parser methods with a fake clock/publisher (no ROS runtime)."""
from pathlib import Path
import os
import subprocess
import tempfile
import unittest


def method(source, signature):
    start=source.index(signature)
    brace=source.index('{',start);depth=1;end=brace+1
    while depth:
        depth += (source[end]=='{')-(source[end]=='}');end+=1
    return source[start:end]


class PacketMappingTest(unittest.TestCase):
    def test_fragmented_and_coalesced_packets_keep_values_and_byte_offsets(self):
        root=Path(__file__).resolve().parents[1]
        source=(root/'src/mms101_node.cpp').read_text()
        methods='\n'.join(method(source,s) for s in [
            '  static int32_t parse_24bit_be(', '  void process_packet(', '  void parse_available_packets(', '  void read_loop('])
        prefix=r'''
#include "acquisition_records.hpp"
#include <atomic>
#include <cassert>
#include <memory>
#include <unistd.h>
#include <sys/ioctl.h>
#include <chrono>
namespace rclcpp {inline bool ok() {return true;}}
#define RCLCPP_ERROR_THROTTLE(...) ((void)0)
#define RCLCPP_ERROR(...) ((void)0)
namespace geometry_msgs { namespace msg {
struct WrenchStamped {
 struct Stamp {int32_t sec; uint32_t nanosec;};
 struct {Stamp stamp; std::string frame_id;} header;
 struct Vec {double x,y,z;};
 struct {Vec force,torque;} wrench;
};
}}
struct Publisher {
 std::vector<geometry_msgs::msg::WrenchStamped> output;
 void publish(const geometry_msgs::msg::WrenchStamped & msg) {output.push_back(msg);}
};
class Driver {
public:
 static constexpr std::size_t PACKET_SIZE=25;
 static constexpr uint8_t HEADER0=0,HEADER1=0x17;
 double force_scale_{.001},torque_scale_{.00001};
 std::string frame_id_{"sensor"};
 std::vector<uint8_t> rx_buffer_;
 std::atomic<uint64_t> bytes_received_{0},valid_packets_{0},discarded_bytes_{0};
 std::atomic<std::size_t> rx_pending_{0};
 std::atomic<bool> first_packet_received_{false};
 std::shared_ptr<Publisher> publisher_{std::make_shared<Publisher>()};
 acquisition_records::Read * active_acquisition_read_{nullptr};
 struct Time {
  int32_t sec=1792054592;uint32_t nanosec=123456789;
  operator geometry_msgs::msg::WrenchStamped::Stamp() const {return {sec,nanosec};}
  int64_t nanoseconds() const {return static_cast<int64_t>(sec)*1000000000LL+nanosec;}
 };
 bool timing_enabled_{false},acquisition_records_enabled_{false};
 std::atomic<bool> running_{false};int serial_fd_{-1},acquisition_startup_attempt_{1};
 std::atomic<uint64_t> read_calls_{0},read_errors_{0};
 uint64_t acquisition_read_sequence_{0};
 std::mutex timing_mutex_;acquisition_timing::Window timing_window_;
 acquisition_records::Buffer acquisition_buffer_;
 acquisition_records::ClockSample acquisition_clock() {return {{1,2,3}};}
 Time now() {return {}; }
 std::vector<uint8_t> next_read_bytes;
 ssize_t read(int, void * output, std::size_t maximum) {
  assert(next_read_bytes.size()<=maximum);
  std::copy(next_read_bytes.begin(),next_read_bytes.end(),static_cast<uint8_t *>(output));
  running_.store(false);return static_cast<ssize_t>(next_read_bytes.size());
 }
'''
        suffix=r'''
 void feed(std::vector<uint8_t> bytes, bool enabled, std::vector<acquisition_records::Sample> & mapping) {
   acquisition_records::Read r;
   active_acquisition_read_=enabled?&r:nullptr;
   bytes_received_ += bytes.size();
   rx_buffer_.insert(rx_buffer_.end(),bytes.begin(),bytes.end());
   parse_available_packets(); active_acquisition_read_=nullptr;
   mapping.insert(mapping.end(),r.samples.begin(),r.samples.end());
 }
};
int main() {
 std::vector<uint8_t> packet(25,0);packet[1]=0x17;
 for (auto p: {4,7,10,13,16,19}) {packet[p]=0xff;packet[p+1]=0xfb;packet[p+2]=0x2e;}
 std::vector<uint8_t> stream={0x33,0x44};
 stream.insert(stream.end(),packet.begin(),packet.end());
 stream.insert(stream.end(),packet.begin(),packet.end());
 Driver loop;loop.next_read_bytes=stream;loop.acquisition_records_enabled_=true;
 loop.running_.store(true);loop.read_loop();auto records=loop.acquisition_buffer_.take();
 assert(records.reads.size()==1 && records.reads[0].bytes==stream);
 assert(records.reads[0].samples.size()==2 && records.reads[0].samples[0].packet_byte_offset==2);
 assert(records.reads[0].samples[1].packet_byte_offset==27);
 assert(records.reads[0].discarded_bytes==2 && records.reads[0].pending_after==0);
 Driver plain;plain.next_read_bytes=stream;plain.running_.store(true);plain.read_loop();
 assert(plain.acquisition_buffer_.take().reads.empty());
 assert(plain.publisher_->output.size()==loop.publisher_->output.size());
 Driver on,off;std::vector<acquisition_records::Sample> mapping,unused;
 for (std::size_t i=0;i<stream.size();) {
   std::size_t n=std::min<std::size_t>((i==0?3:(i==3?8:100)),stream.size()-i);
   std::vector<uint8_t> chunk(stream.begin()+i,stream.begin()+i+n);
   on.feed(chunk,true,mapping);off.feed(chunk,false,unused);i+=n;
 }
 assert(on.publisher_->output.size()==2 && off.publisher_->output.size()==2);
 assert(mapping.size()==2 && unused.empty());
 assert(mapping[0].packet_byte_offset==2 && mapping[1].packet_byte_offset==27);
 assert(mapping[0].sequence==1 && mapping[1].sequence==2);
 assert(mapping[0].header_ns==1792054592123456789LL);
 assert(on.discarded_bytes_==2);
 for (std::size_t i=0;i<2;++i) {
   const auto &a=on.publisher_->output[i];const auto &b=off.publisher_->output[i];
   assert(a.wrench.force.y==-1234*.001 && a.wrench.torque.z==-1234*.00001);
   assert(a.wrench.force.y==b.wrench.force.y && a.header.stamp.nanosec==b.header.stamp.nanosec);
 }
}
'''
        with tempfile.TemporaryDirectory() as temp:
            f=Path(temp)/'test.cpp';f.write_text(prefix+methods+suffix)
            includes=[str(root/'src')]
            fallback=os.environ.get('MMS101_TEST_INCLUDE')
            if fallback: includes.append(fallback)
            command=[os.environ.get('CXX','c++'),'-std=c++14','-Wall','-Wextra','-Werror']
            for inc in includes: command+=['-I',inc]
            subprocess.run(command+[str(f),'-o',str(Path(temp)/'test')],check=True)
            subprocess.run([str(Path(temp)/'test')],check=True)


if __name__=='__main__':unittest.main()
