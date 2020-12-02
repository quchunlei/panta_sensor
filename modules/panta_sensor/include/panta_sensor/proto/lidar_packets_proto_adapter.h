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

#ifdef PROTO_FOUND

#include <panta_common/interface/sensor/lidar_packets_interface.h>
#include <panta_common/msg/proto_msg_translator.h>
#include <panta_common/proto/proto_base.hpp>
#include <condition_variable>
#include <mutex>

namespace robosense
{
namespace sensor
{
class LidarPacketsProtoAdapter : virtual public common::LidarPacketsInterface
{
public:
  LidarPacketsProtoAdapter() = default;
  ~LidarPacketsProtoAdapter() { stop(); }

  common::ErrCode init(const YAML::Node &config);
  common::ErrCode start();
  common::ErrCode stop();

  inline void regRecvCallback(const std::function<void(const common::LidarScanMsg &)> callBack)
  {
    msop_cb_.emplace_back(callBack);
  }
  inline void regRecvCallback(const std::function<void(const common::LidarPacketMsg &)> callBack)
  {
    difop_cb_.emplace_back(callBack);
  }
  inline void regExceptionCallback(const std::function<void(const common::ErrCode &)> excallBack)
  {
    excb_ = excallBack;
  }
  void send_msop(const common::LidarScanMsg &msg);
  void send_difop(const common::LidarPacketMsg &msg);

private:
  inline void localMsopCallback(const common::LidarScanMsg &rs_msg)
  {
    for (auto &cb : msop_cb_)
    {
      cb(rs_msg);
    }
  }
  inline void localDifopCallback(const common::LidarPacketMsg &rs_msg)
  {
    for (auto &cb : difop_cb_)
    {
      cb(rs_msg);
    }
  }
  inline void reportError(const common::ErrCode &error)
  {
    if (excb_ != NULL)
    {
      excb_(error);
    }
  }
  void send_msop_packets();

private:
  enum state
  {
    IDLE = 0,
    RECEIVE,
    SENDMSG,
  };
  void msopStateMachine();
  void difopStateMachine();

private:
  std::vector<std::function<void(const common::LidarScanMsg &)>> msop_cb_;
  std::vector<std::function<void(const common::LidarPacketMsg &)>> difop_cb_;
  std::function<void(const common::ErrCode &)> excb_;
  std::unique_ptr<std::thread> msop_send_thread_;
  std::unique_ptr<std::thread> msop_recv_thread_;
    std::unique_ptr<std::thread> difop_recv_thread_;


  bool thread_flag_;
  bool send_thread_flag_;
  state msop_state_;
  state difop_state_;
  std::mutex msop_send_mutex_;
  std::condition_variable msop_send_cv_;
  Proto_msg::LidarScan proto_msop_msg_;
  std::unique_ptr<common::ProtoBase> msop_proto_ptr_;
  std::unique_ptr<common::ProtoBase> difop_proto_ptr_;

private:
  static const uint16_t supported_api_ = 0x0010;
}; // namespace sensor
} // namespace sensor
} //namespace robosense
#endif //PROTO_FOUND