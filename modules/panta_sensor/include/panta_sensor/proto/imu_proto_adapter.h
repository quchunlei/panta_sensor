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

#include <panta_common/interface/sensor/imu_interface.h>
#include <panta_common/msg/proto_msg/Proto_msg.Imu.pb.h>
#include <panta_common/msg/proto_msg_translator.h>
#include <panta_common/proto/proto_base.hpp>
namespace robosense
{
namespace sensor
{
class ImuProtoAdapter : virtual public common::ImuInterface, virtual public common::ProtoBase
{
public:
  ImuProtoAdapter() = default;
  ~ImuProtoAdapter() { stop(); }

  common::ErrCode init(const YAML::Node &config);
  common::ErrCode start();
  common::ErrCode stop();

  inline void regRecvCallback(const std::function<void(const common::ImuMsg &)> callBack)
  {
    imu_cb_.emplace_back(callBack);
  }
  inline void regExceptionCallback(const std::function<void(const common::ErrCode &)> excallBack)
  {
    excb_ = excallBack;
  }
  void send(const common::ImuMsg &msg);

private:
  inline void localCallback(const common::ImuMsg &rs_msg)
  {
    for (auto &cb : imu_cb_)
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

private:
  enum state
  {
    IDLE = 0,
    RECEIVE,
    SENDMSG,
  };
  void stateMachine();

private:
  std::vector<std::function<void(const common::ImuMsg &)>> imu_cb_;
  std::function<void(const common::ErrCode &)> excb_;
  std::unique_ptr<std::thread> imu_thread_;
  bool thread_flag_;
  state self_state_;
private:
  static const uint16_t supported_api_ = 0x0004;
};
} // namespace sensor
} //namespace robosense
#endif //PROTO_FOUND