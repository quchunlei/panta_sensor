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
#include "panta_sensor/driver/imu/imu_base.h"
#include <iomanip>
namespace robosense
{
namespace WitImu
{
const double dy_g = 9.8;
const int dy_msg_len = 31;
const int dy_acc_scale = 4;
const int dy_gyr_scale = 250;
const int dy_angle_scale = 360;
const int dy_temp_scale = 200;
const double dy_deg2rad = 0.01745329;
const int dy_max_range = 32768;

const int year_s[2] = { 365 * 24 * 60 * 60, 366 * 24 * 60 * 60 };
const int month_s[2][12] = { { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 },
                             { 31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 } };
const int day_s = 24 * 60 * 60;
const int hour_s = 60 * 60;
const int minute_s = 60;

struct ImuData1
{
  Time time_;
  Acc acc_;
  Gyr gyr_;
  Eular_Angle eular_angle_;
  double temperature_;

  ImuData1 operator+(const ImuData1& b)
  {
    ImuData1 temp_;
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

  ImuData1 operator-(const ImuData1& b)
  {
    ImuData1 temp_;
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

  ImuData1 operator=(int n)
  {
    ImuData1 temp_;
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

  ImuData1 operator/(int n)
  {
    ImuData1 temp_;
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
typedef ImuData1 DY551_Data;
}  // namespace WitImu

namespace sensor
{
class ImuDY551 : virtual public ImuBase
{
public:
  ImuDY551();
  ~ImuDY551() = default;

private:
  inline common::ErrCode init(const YAML::Node& config)
  {
    setName("Imu ImuDY551");
    return this->ImuBase::init(config);
  }

  bool IsRound(int year)
  {
    if ((year % 100) && (year % 4 == 0))
      return 1;
    if ((year % 100 == 0) && (year % 400 == 0))
      return 1;
    return 0;
  }

  void secondToYMDHMS(int temp)
  {
    using namespace robosense::WitImu;
    Time time_c;
    time_c.year = 1970;
    time_c.month = 1;
    time_c.day = 1;
    time_c.hour = 0;
    time_c.minute = 0;
    time_c.sec = 0;

    while (temp >= 60)
    {
      int flag = IsRound(time_c.year);
      if (temp >= year_s[flag])
      {
        time_c.year++;
        temp -= year_s[flag];
      }
      else if (temp >= day_s)
      {
        int days = temp / day_s;
        temp = temp % day_s;
        int i = 0;
        int flag = IsRound(time_c.year);
        int hh = 31;
        while (days >= hh)
        {
          days -= month_s[flag][i++];
          hh = month_s[flag][i];
        }
        time_c.month += i;
        time_c.day += days;
      }
      else if (temp >= hour_s)
      {
        time_c.hour = temp / hour_s;
        temp %= hour_s;
      }
      else if (temp >= minute_s)
      {
        time_c.minute = temp / minute_s;
        temp %= minute_s;
      }
    }
    time_c.sec = temp;
    yj9_data_.time_ = time_c;
  }

  void prepareMsg();
  int checkMarkBit();
  bool autoConnect();
  void analysis(std::vector<uint8_t>& buf_);
  void decode_msg(int num, std::vector<uint8_t>& buf_rec_);
  robosense::WitImu::DY551_Data yj9_data_;
  std::vector<uint8_t> read_rec_;
  std::vector<uint8_t> write_rec_;

  void translation_4d(std::array<int16_t, 4>& vec_hec)
  {
    for (int i = 0; i < 4; i++)
    {
      if (vec_hec[i] & 0x8000)
      {
        vec_hec[i] -= 1;
        vec_hec[i] = ~vec_hec[i];
        vec_hec[i] &= 0x7fff;
        vec_hec[i] = -vec_hec[i];
      }
    }
  }

  void translation_3d(std::array<int16_t, 3>& vec_hec)
  {
    for (int i = 0; i < 3; i++)
    {
      if (vec_hec[i] & 0x8000)
      {
        vec_hec[i] -= 1;
        vec_hec[i] = ~vec_hec[i];
        vec_hec[i] &= 0x7fff;
        vec_hec[i] = -vec_hec[i];
      }
    }
  }

  void translation_2d(std::array<int32_t, 2>& vec_hec)
  {
    for (int i = 0; i < 2; i++)
    {
      if (vec_hec[i] & 0x80000000)
      {
        vec_hec[i] -= 1;
        vec_hec[i] = ~vec_hec[i];
        vec_hec[i] &= 0x7fffffff;
        vec_hec[i] = -vec_hec[i];
      }
    }
  }
};
}  // namespace sensor
}  // namespace robosense
