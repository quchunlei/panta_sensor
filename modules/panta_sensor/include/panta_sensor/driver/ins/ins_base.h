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
#include "panta_sensor/driver/gnss/gnss_base.h"
#include <panta_common/interface/sensor/imu_interface.h>

namespace robosense
{
namespace sensor
{
class InsBase : public GnssBase, public common::ImuInterface
{
public:
  InsBase()
  {
    leaps_ = { 46828800,  78364801,  109900802, 173059203, 252028804, 315187205, 346723206,  393984007,  425520008,
               457056009, 504489610, 551750411, 599184012, 820108813, 914803214, 1025136015, 1119744016, 1167264017 };
  }
  ~InsBase()
  {
    stop();
  }
  virtual common::ErrCode init(const YAML::Node& config)
  {
    imu_seq_ = 0;
    return this->GnssBase::init(config);
  }
  inline common::ErrCode start()
  {
    return this->GnssBase::start();
  }
  inline common::ErrCode stop()
  {
    return this->GnssBase::stop();
  }
  inline void regExceptionCallback(const std::function<void(const common::ErrCode&)> excallBack)
  {
    this->GnssBase::regExceptionCallback(excallBack);
  }
  inline void regRecvCallback(const std::function<void(const common::GnssMsg&)> callBack)
  {
    this->GnssBase::regRecvCallback(callBack);
  }
  inline void regRecvCallback(const std::function<void(const common::OdomMsg&)> callBack)
  {
    this->GnssBase::regRecvCallback(callBack);
  }
  inline void regRecvCallback(const std::function<void(const common::ImuMsg&)> callBack)
  {
    imucb_.emplace_back(callBack);
  }
  static inline uint16_t getApi()
  {
    return supported_api_;
  }

protected:
  inline void runCallBack(const common::ImuMsg& imu_msg)
  {
    for (auto it : imucb_)
    {
      it(imu_msg);
    }
  }

  inline bool isleap(const double& gpstime)
  {
    bool isleap = false;
    for (auto iter : leaps_)
    {
      if (gpstime == iter)
        isleap = true;
    }
    return isleap;
  }
  inline int countleaps(const double& gpstime)
  {
    int nleaps = 0;  // number of leap seconds prior to gpsTime
    for (auto iter : leaps_)
    {
      if (gpstime >= iter)
      {
        nleaps++;
      }
    }
    return nleaps;
  }

  inline double gps2unix(const double& gpstime)
  {
    double unixtime = gpstime + 315964800;
    int nleaps = countleaps(gpstime);
    unixtime = unixtime - nleaps;
    if (isleap(gpstime))
    {
      unixtime = unixtime + 0.5;
    }
    return unixtime;
  }
  std::vector<std::function<void(const common::ImuMsg&)>> imucb_;
  uint32_t imu_seq_;
  std::array<double, 18> leaps_;

private:
  static const uint16_t supported_api_ = 0x0007;  // 0000 0000 0000 0111 (support GNSS,IMU,ODOM)
};
}  // namespace sensor
}  // namespace robosense
