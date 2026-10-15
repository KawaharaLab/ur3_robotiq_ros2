#include "../src/acquisition_records.hpp"
#include <cassert>
#include <iostream>
#include <thread>
int main()
{
  using namespace acquisition_records;
  Buffer bounded(2);
  for (uint64_t i = 1; i <= 3; ++i) {
    Read read; read.sequence = i; read.samples.push_back({i, 1792054592915155240LL, 0});
    bounded.push(std::move(read));
  }
  auto first = bounded.take(1);
  assert(first.reads.size() == 1 && first.reads[0].sequence == 2);
  assert(first.dropped_reads == 1 && first.dropped_samples == 1);
  assert(bounded.take().reads[0].sequence == 3);
  assert(bounded.take().reads.empty());
  Buffer concurrent;
  std::thread writer([&concurrent]() {
    for (uint64_t i=1; i<=1000; ++i) {Read r; r.sequence=i; concurrent.push(std::move(r));}
  });
  writer.join();
  std::size_t total=0;
  for (;;) {auto s=concurrent.take(); if (s.reads.empty()) break; total+=s.reads.size();}
  assert(total==1000);
  Snapshot snapshot;
  Read r; r.sequence=1; r.byte_offset=9; r.start={{10,1792054592915155240LL,12}};
  r.end={{20,1792054592915155250LL,22}}; r.bytes={0,0x17,0xff};
  r.samples.push_back({1,1792054592915155240LL,0}); snapshot.reads.push_back(r);
  std::cout << json(snapshot,"stream\"id",1,"node","/dev/mms101_left","frame",1,false,.001,.00001,0,0) << '\n';
}
