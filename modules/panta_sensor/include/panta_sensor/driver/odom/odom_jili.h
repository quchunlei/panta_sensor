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
#include "panta_sensor/driver/odom/odom_base.h"

namespace robosense
{
namespace sensor
{
class OdomJILI : virtual public OdomBase
{
public:
  OdomJILI() = default;
  ~OdomJILI() = default;

private:
  inline common::ErrCode init(const YAML::Node& config)
  {
    setName("Odom_JILI");
    direction_ = 0;
    gear_current_mode_ = 0;
    return this->OdomBase::init(config);
  }
  void prepareMsg(vector<canbusData> buf);
  int direction_;
  int gear_current_mode_;
};
}  // namespace sensor
}  // namespace robosense
