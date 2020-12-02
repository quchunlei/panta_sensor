#pragma once

#include "panta_sensor/driver/odom/controlcan.h"
#include <pthread.h>
#include <vector>
#include <mutex>
#include <unordered_map>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/signal.h>
#include <sys/stat.h>
#include <panta_common/debug/prompt.h>

struct canbusData
{
  std::string frame_id;
  uint32_t id;
  std::array<uint8_t, 8> data;
};

using std::mutex;
using std::unique_lock;
using std::vector;

namespace robosense
{
namespace sensor
{
class CanBridge
{
public:
  CanBridge(unsigned int can_index);
  ~CanBridge();
  int Read(vector<canbusData>& frames);
  void CanOpen(int timeing0, int timing1);

private:
  VCI_BOARD_INFO board_info_;
  static void* ReceiveFunc(void* param);
  void CanClose();
  pthread_t thread_0_;
  static void* FuncHelper(void* a)
  {
    return (void*)ReceiveFunc(&run_flag_);
  }
  static unsigned int device_index_;
  static unsigned int can_index_;
  static int run_flag_;
  static vector<canbusData> can_buf_;
  static canbusData can_tmp_;
  static mutex can_mutex_sync_;
};
}  // namespace sensor
}  // namespace robosense
