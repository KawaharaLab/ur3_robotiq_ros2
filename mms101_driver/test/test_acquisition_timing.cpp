#include "../src/acquisition_timing.hpp"
#include <cassert>
#include <iostream>

int main()
{
  acquisition_timing::Window window;
  assert(window.packets_per_read.count == 0);
  window.add(1.0, -1, 25, 1, 0.1, 0, 123);
  window.add(3.0, 2.0, 75, 3, 0.3, 25, 456);
  assert(window.positive_reads == 2);
  assert(window.multi_packet_reads == 1);
  assert(window.read_return_interval_ms.count == 1);
  assert(window.packets_per_read.sum == 4);
  assert(window.packets_per_read.maximum == 3);
  std::cout << window.json(1.0, 999, "test", "port\"\\\n", "left") << '\n';
  window = acquisition_timing::Window{};
  assert(window.positive_reads == 0);
  assert(window.read_block_ms.count == 0);
  std::cout << window.json(1.0, 1000, "test", "port", "left") << '\n';
}
