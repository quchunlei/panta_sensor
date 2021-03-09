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
namespace sensor
{
class ImuHWT605 : virtual public ImuBase
{
public:
  ImuHWT605() = default;
  ~ImuHWT605() = default;

private:
  inline common::ErrCode init(const YAML::Node& config)
  {
    setName("Imu ImuHWT605");
    return this->ImuBase::init(config);
  }
  void prepareMsg();
  int checkMarkBit();
  bool autoConnect();
  void analysis(std::vector<uint8_t> buf_);
  void decode_msg(int num, std::vector<uint8_t> buf_rec_);
  robosense::WitImu::YJ901_Data yj9_data_;

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
