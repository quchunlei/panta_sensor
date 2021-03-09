/**
******************************************************************************
* Copyright (C) 2018 Joey.Liu <lty2226262@gmail.com>
* Distributed under terms of the MIT license.
******************************************************************************
*/

//#define DEBUG
//#define DEBUG_BRIDGE

#include "panta_sensor/driver/odom/can_bridge.h"

using std::mutex;
using std::string;
using std::to_string;
using std::vector;

unsigned int robosense::sensor::CanBridge::device_index_ = 0;
unsigned int robosense::sensor::CanBridge::can_index_ = 0;
int robosense::sensor::CanBridge::run_flag_ = 0;

vector<canbusData> robosense::sensor::CanBridge::can_buf_ = vector<canbusData>();
canbusData robosense::sensor::CanBridge::can_tmp_;

mutex robosense::sensor::CanBridge::can_mutex_sync_;

namespace robosense
{
namespace sensor
{
CanBridge::CanBridge(unsigned int can_index)
{
  can_index_ = can_index;
}

CanBridge::~CanBridge()
{
  CanClose();
}

int CanBridge::Read(vector<canbusData>& frames)
{
  frames.clear();
  {
    unique_lock<mutex> lock(can_mutex_sync_);
    for (auto it : can_buf_)
    {
      frames.push_back(it);
    }
    can_buf_.clear();
  }
  return frames.size();
}

void* CanBridge::ReceiveFunc(void* param)
{
  int receive_length;
  VCI_CAN_OBJ receive_buffer[2500];

  int* run = static_cast<int*>(param);

  while ((*run) & 0x0f)
  {
    if ((receive_length = VCI_Receive(VCI_USBCAN2, device_index_, can_index_, receive_buffer, 1000, 100)) > 0)
    {
      for (int i = 0; i < receive_length; i++)
      {
        can_tmp_.frame_id = "/can";
        can_tmp_.id = receive_buffer[i].ID;
        for (int j = 0; j < receive_buffer[i].DataLen; j++)
        {
          can_tmp_.data[j] = receive_buffer[i].Data[j];
        }
        {
          unique_lock<mutex> lock(can_mutex_sync_);
          can_buf_.emplace_back(can_tmp_);
        }
      }
    }
  }
  pthread_exit(0);
}

void CanBridge::CanOpen(int timeing0, int timeing1)
{
  if (run_flag_ != 0)
  {
#if (DEBUG_LEVEL > 0)
    WARNING << "CanBus: Can tool has been open" << REND;
#endif
    return;
  }

  if (VCI_OpenDevice(VCI_USBCAN2, device_index_, 0) == 1)
  {
#if (DEBUG_LEVEL > 1)
    INFO << "CanBus: Open device success" << REND;
#endif
  }
  else
  {
    ERROR << "CanBus: Open device fail" << REND;
    exit(-1);
  }
#if (DEBUG_LEVEL > 1)
  if (VCI_ReadBoardInfo(VCI_USBCAN2, device_index_, &board_info_) == 1)
  {
    INFO << "CanBus: Get VCI_ReadBoardInfo success!" << REND;

    std::string output_string("");
    for (int i = 0; i < 20; i++)
    {
      output_string += board_info_.str_Serial_Num[i];
    }

    INFO << "CanBus: Serial_Num: " << output_string << REND;
    output_string.clear();

    for (int i = 0; i < 10; i++)
    {
      output_string += board_info_.str_hw_Type[i];
    }

    INFO << "CanBus: hw_Type:" << output_string << REND;
  }

#endif
  VCI_INIT_CONFIG config;
  config.AccCode = 0;
  config.AccMask = 0xffffffff;
  config.Filter = 1;
  config.Mode = 0;

  /*500 Kbps  0x00  0x1C*/
  config.Timing0 = timeing0;
  config.Timing1 = timeing1;

  if (VCI_InitCAN(VCI_USBCAN2, device_index_, can_index_, &config) != 1)
  {
    ERROR << "CanBus: init CAN error" << REND;
    exit(-1);
    VCI_CloseDevice(VCI_USBCAN2, device_index_);
  }

  if (VCI_StartCAN(VCI_USBCAN2, device_index_, can_index_) != 1)
  {
    ERROR << "CanBus: Start CAN error" << REND;
    exit(-1);
    VCI_CloseDevice(VCI_USBCAN2, device_index_);
  }

  // // open CAN2 for loop test
  // if (VCI_InitCAN(VCI_USBCAN2, device_index_, 1, &config) != 1)
  // {

  //     ERROR << "CanBus: init CAN 2 error" << REND;
  //     exit(-1);
  //     VCI_CloseDevice(VCI_USBCAN2, device_index_);
  // }

  // if (VCI_StartCAN(VCI_USBCAN2, device_index_, 1) != 1)
  // {

  //     ERROR << "CanBus: Start CAN 2 error" << REND;
  //     exit(-1);
  //     VCI_CloseDevice(VCI_USBCAN2, device_index_);
  // }

  run_flag_ = 1;
  pthread_create(&thread_0_, NULL, CanBridge::FuncHelper, this);
}

void CanBridge::CanClose()
{
  run_flag_ = 0;
  pthread_join(thread_0_, NULL);
  VCI_CloseDevice(VCI_USBCAN2, device_index_);
}
}  // namespace sensor
}  // namespace robosense
