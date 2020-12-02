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

#include "panta_sensor/driver/odom/can_bridge.h"
#include <panta_common/interface/sensor/odom_interface.h>
namespace robosense
{
namespace sensor
{
class OdomBase : virtual public common::OdomInterface
{
public:
  OdomBase() = default;
  ~OdomBase()
  {
    stop();
  }
  virtual common::ErrCode init(const YAML::Node& config);
  common::ErrCode start();
  common::ErrCode stop();

  inline void regRecvCallback(const std::function<void(const common::OdomMsg&)> callBack)
  {
    odomcb_.emplace_back(callBack);
  }
  inline void regExceptionCallback(const std::function<void(const common::ErrCode&)> excallBack)
  {
    excb_ = excallBack;
  }
  static inline uint16_t getApi()
  {
    return supported_api_;
  }

protected:
  virtual void prepareMsg(vector<canbusData> buf) = 0;

protected:
  enum state
  {
    IDLE = 0,
    READ,
    READ_DATA,
  };
  void stateMachine();

protected:
  struct Odom_Parameter
  {
    std::string device_type;
    std::string frame_id;
  };

protected:
  inline void reportError(const common::ErrCode& error)
  {
    if (excb_ != NULL)
    {
      excb_(error);
    }
  }
  inline void runCallBack(const common::OdomMsg& msg)
  {
    for (auto& it : odomcb_)
    {
      it(msg);
    }
  }

protected:
  std::function<void(const common::ErrCode&)> excb_;
  std::vector<std::function<void(const common::OdomMsg&)>> odomcb_;
  std::shared_ptr<CanBridge> ptrCanbridge_ptr_;
  std::shared_ptr<std::thread> odom_thread_ptr_;
  bool thread_flag_;
  state self_state_;
  uint32_t odom_seq_;
  Odom_Parameter odom_parameter_;

private:
  static const uint16_t supported_api_ = 0x0001;
};
}  // namespace sensor
}  // namespace robosense
