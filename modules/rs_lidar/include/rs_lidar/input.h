/******************************************************************************
 * Copyright 2017 robosense All rights reserved.
 * Suteng Innovation Technology Co., Ltd. www.robosense.ai

 * This software is provided to you directly by robosense and might
 * only be used to access robosense LiDAR. Any compilation,
 * modification, exploration, reproduction and redistribution are
 * restricted without robosense's prior consent.

 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESSED OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL robosense BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/
#pragma once
#include <panta_common/common.h>
#include <cstdint>
#include <memory>
#include <string>
#include <pcap.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <array>
#include <cmath>
#include <cstring>
#include <algorithm>
#include <functional>
#include <iterator>
#include <vector>
#include <iostream>
#include <chrono>
namespace robosense
{
namespace sensor
{
const int RSLIDAR_PKT_LEN = 1248;

enum InputState
{
  INPUT_OK = 0,
  INPUT_TIMEOUT = 1,
  INPUT_ERROR = 2,
  INPUT_DIFOP = 4,
  INPUT_MSOP = 8,
  INPUT_EXIT = 16
};

class Input : public common::CommonBase
{
public:
  Input() = default;
  ~Input();

  InputState getPacket(uint8_t *pkt, uint32_t timeout);
  void init(const YAML::Node &yaml_param);

private:
  int setUpSocket(uint16_t port);

  uint16_t msop_port_;
  uint16_t difop_port_;
  int msop_fd_;
  int difop_fd_;

  pcap_t *pcap_;
  bpf_program pcap_msop_filter_;
  bpf_program pcap_difop_filter_;

  std::string device_ip_;
  std::string pcap_file_dir_;
  bool read_pcap_;
};
} // namespace sensor
} // namespace robosense
