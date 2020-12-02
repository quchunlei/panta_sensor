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
#include <panta_common/interface/sensor/gnss_interface.h>
#include <panta_common/interface/sensor/odom_interface.h>
namespace robosense
{
namespace sensor
{
class GnssBase : public common::GnssInterface, public common::OdomInterface
{
public:
  GnssBase() = default;
  ~GnssBase()
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
  inline void regRecvCallback(const std::function<void(const common::GnssMsg&)> callBack)
  {
    gnsscb_.emplace_back(callBack);
  }
  inline void regRecvCallback(const std::function<void(const common::OdomMsg&)> callBack)
  {
    odomcb_.emplace_back(callBack);
  }
  static uint16_t getApi()
  {
    return supported_api_;
  }

protected:
  virtual void prepareMsg(const std::vector<std::string>& msg, bool use_gnss_clock) = 0;
  virtual bool autoConnect() = 0;

protected:
  inline void runCallBack(const common::GnssMsg& gnss_msg)
  {
    for (auto& it : gnsscb_)
    {
      it(gnss_msg);
    }
  }
  inline void runCallBack(const common::OdomMsg& odom_msg)
  {
    for (auto& it : odomcb_)
    {
      it(odom_msg);
    }
  }
  inline void reportError(const common::ErrCode& error)
  {
    if (excb_ != NULL)
    {
      excb_(error);
    }
  }
  bool processData(std::vector<uint8_t>& data_buf, std::vector<std::string>& data);
  bool processGSOFData(std::vector<uint8_t>& data_buf, std::vector<std::string>& data);

protected:
  enum state
  {
    IDLE = 0,
    CONNECT,
    PREPARE_DATA,
    READ_DATA,
    CHECK_CONNECTION,
  };
  void stateMachine();

protected:
  struct Parameter
  {
    std::string device_type;
    bool auto_scan_port;
    std::string port_name;
    int baudrate;
    std::string frame_id;
    bool do_reconnect;
    int reconnect_interval;
    int reconnect_attempts;
    int min_satellite;
    double warning_gyro_z;
    double timeout;
    bool use_gnss_clock;
  };

protected:
  std::vector<std::function<void(const common::GnssMsg&)>> gnsscb_;
  std::vector<std::function<void(const common::OdomMsg&)>> odomcb_;
  std::function<void(const common::ErrCode&)> excb_;
  std::shared_ptr<std::thread> gnss_thread_;
  std::shared_ptr<Serial> gnss_ser_;
  bool thread_flag_;
  state self_state_;
  uint32_t gnss_seq_;
  uint32_t odom_seq_;
  Parameter parameter_;

private:
  static const uint16_t supported_api_ = 0x0003;
};
}  // namespace sensor
}  // namespace robosense
