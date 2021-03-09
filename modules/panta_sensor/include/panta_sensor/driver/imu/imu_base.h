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
namespace WitImu
{
const double g_ = 9.8;
const int msg_len = 3;
const int acc_scale = 4;
const int gyr_scale = 500;
const int angle_scale = 180;
const double deg2rad = 0.01745329;
const int max_range = 32768;
struct Time
{
  int year;
  int month;
  int day;
  int hour;
  int minute;
  int sec;
};

struct Acc
{
  double acc_x;
  double acc_y;
  double acc_z;
  double T;
};

struct Gyr
{
  double gyr_x;
  double gyr_y;
  double gyr_z;
};

struct Eular_Angle
{
  double roll;
  double pitch;
  double yaw;
};

struct Mag
{
  double h_x;
  double h_y;
  double h_z;
};

struct Press_Height
{
  double presure;
  double height;
};

struct Gps
{
  double Longitude;
  double Latitude;
};

struct Quad
{
  double q0, q1, q2, q3;
};

struct ImuData
{
  Time time_;
  Acc acc_;
  Gyr gyr_;
  Eular_Angle eular_angle_;
  Mag mag_;
  Press_Height ph_;
  Gps gps_;
  Quad quad_;

  ImuData operator+(const ImuData& b)
  {
    ImuData temp_;
    temp_.acc_.acc_x = this->acc_.acc_x + b.acc_.acc_x;
    temp_.acc_.acc_y = this->acc_.acc_y + b.acc_.acc_y;
    temp_.acc_.acc_z = this->acc_.acc_z + b.acc_.acc_z - g_;

    temp_.gyr_.gyr_x = this->gyr_.gyr_x + b.gyr_.gyr_x;
    temp_.gyr_.gyr_y = this->gyr_.gyr_y + b.gyr_.gyr_y;
    temp_.gyr_.gyr_z = this->gyr_.gyr_z + b.gyr_.gyr_z;

    temp_.eular_angle_.roll = this->eular_angle_.roll + b.eular_angle_.roll;
    temp_.eular_angle_.pitch = this->eular_angle_.pitch + b.eular_angle_.pitch;
    temp_.eular_angle_.yaw = this->eular_angle_.yaw + b.eular_angle_.yaw;
    return temp_;
  }

  ImuData operator-(const ImuData& b)
  {
    ImuData temp_;
    temp_.acc_.acc_x = this->acc_.acc_x - b.acc_.acc_x;
    temp_.acc_.acc_y = this->acc_.acc_y - b.acc_.acc_y;
    temp_.acc_.acc_z = this->acc_.acc_z - b.acc_.acc_z;

    temp_.gyr_.gyr_x = this->gyr_.gyr_x - b.gyr_.gyr_x;
    temp_.gyr_.gyr_y = this->gyr_.gyr_y - b.gyr_.gyr_y;
    temp_.gyr_.gyr_z = this->gyr_.gyr_z - b.gyr_.gyr_z;

    temp_.eular_angle_.roll = this->eular_angle_.roll - b.eular_angle_.roll;
    temp_.eular_angle_.pitch = this->eular_angle_.pitch - b.eular_angle_.pitch;
    temp_.eular_angle_.yaw = this->eular_angle_.yaw - b.eular_angle_.yaw;
    return temp_;
  }

  ImuData operator=(int n)
  {
    ImuData temp_;
    temp_.acc_.acc_x = n;
    temp_.acc_.acc_y = n;
    temp_.acc_.acc_z = n;

    temp_.gyr_.gyr_x = n;
    temp_.gyr_.gyr_y = n;
    temp_.gyr_.gyr_z = n;

    temp_.eular_angle_.roll = n;
    temp_.eular_angle_.pitch = n;
    temp_.eular_angle_.yaw = n;
    return temp_;
  }

  ImuData operator/(int n)
  {
    ImuData temp_;
    temp_.acc_.acc_x = this->acc_.acc_x / n;
    temp_.acc_.acc_y = this->acc_.acc_y / n;
    temp_.acc_.acc_z = this->acc_.acc_z / n;

    temp_.gyr_.gyr_x = this->gyr_.gyr_x / n;
    temp_.gyr_.gyr_y = this->gyr_.gyr_y / n;
    temp_.gyr_.gyr_z = this->gyr_.gyr_z / n;

    temp_.eular_angle_.roll = this->eular_angle_.roll / n;
    temp_.eular_angle_.pitch = this->eular_angle_.pitch / n;
    temp_.eular_angle_.yaw = this->eular_angle_.yaw / n;
    return temp_;
  }
};
typedef ImuData YJ901_Data;
}  // namespace WitImu
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
