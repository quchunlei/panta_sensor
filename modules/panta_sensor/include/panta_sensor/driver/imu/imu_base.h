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
#include "panta_sensor/driver/serial/rs_serial.h"
#include <panta_common/interface/sensor/imu_interface.h>
namespace robosense
{
namespace sensor
{
class ImuBase : virtual public common::ImuInterface
{
public:
  ImuBase() = default;
  ~ImuBase()
  {
    stop();
  }
  virtual common::ErrCode init(const YAML::Node& config);
  common::ErrCode start();
  common::ErrCode stop();

  inline void regExceptionCallback(const std::function<void(const common::ErrCode&)> excallBack)
  {
    excb_ = excallBack;
  }

  inline void regRecvCallback(const std::function<void(const common::ImuMsg&)> callBack)
  {
    imucb_.emplace_back(callBack);
  }
  static uint16_t getApi()
  {
    return supported_api_;
  }

protected:
  virtual void prepareMsg() = 0;
  virtual bool autoConnect() = 0;
  virtual int checkMarkBit() = 0;

protected:
  inline void reportError(const common::ErrCode& error)
  {
    if (excb_ != NULL)
    {
      excb_(error);
    }
  }
  inline void runCallBack(const common::ImuMsg& imu_msg)
  {
    for (auto& it : imucb_)
    {
      it(imu_msg);
    }
  }

protected:
  enum state
  {
    IDLE = 0,
    CONNECT,
    CHECK_MARK_BIT,
    READ_DATA,
    CHECK_CONNECTION,
  };
  void stateMachine();

protected:
  struct Imu_Parameter
  {
    std::string device_type = "";
    bool auto_scan_port = false;
    std::string port_name = "";
    int baudrate = 0;
    std::string frame_id = "";
    bool do_reconnect = false;
    int reconnect_interval = 0;
    int reconnect_attemps = 0;
    double warning_gyro_z = 0;
    double timeout = 0;
    int imu_correction = 0;
  };

protected:
  std::vector<std::function<void(const common::ImuMsg&)>> imucb_;
  std::function<void(const common::ErrCode&)> excb_;
  std::shared_ptr<Serial> imu_ser_;
  std::shared_ptr<std::thread> imu_thread_;
  bool thread_flag_;
  state self_state_;
  uint32_t imu_seq_;

  Imu_Parameter imu_parameter_;

private:
  static const uint16_t supported_api_ = 0x0004;
};
}  // namespace sensor
}  // namespace robosense
