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
#include "panta_sensor/driver/odom/odom_base.h"
namespace robosense
{
namespace sensor
{
using namespace robosense::common;
ErrCode OdomBase::init(const YAML::Node& config)
{
  setinitFlag(true);
  int timeing0;
  int timeing1;
  unsigned int can_index;
  YAML::Node driver_config = yamlSubNodeAbort(config, "driver");
  yamlRead<std::string>(driver_config, "device_type", odom_parameter_.device_type, "");
  yamlRead<std::string>(driver_config, "frame_id", odom_parameter_.frame_id, "/odom");
  yamlReadAbort<int>(driver_config, "timeing0", timeing0);
  yamlReadAbort<int>(driver_config, "timeing1", timeing1);
  yamlRead<unsigned int>(driver_config, "can_index", can_index, 0);
  odom_thread_ptr_ = std::make_shared<std::thread>();
  ptrCanbridge_ptr_ = std::make_shared<CanBridge>(can_index);
  ptrCanbridge_ptr_->CanOpen(timeing0, timeing1);
  odom_seq_ = 0;
  odomcb_.reserve(10);
  excb_ = NULL;
  thread_flag_ = false;
  self_state_ = IDLE;
  return ErrCode_Success;
}

ErrCode OdomBase::start()
{
  if (odomcb_.empty())
  {
    ERROR << "OdomBase: Please register at least one callback function first!" << REND;
    exit(-1);
  }
  if (thread_flag_ == false)
  {
    odom_seq_ = 0;
    thread_flag_ = true;
    self_state_ = IDLE;
    const auto& func1 = [this] { stateMachine(); };
    odom_thread_ptr_ = std::make_shared<std::thread>(func1);
#if (DEBUG_LEVEL > 1)
    INFO << "OdomBase Start!" << REND;
#endif
  }
  return ErrCode_Success;
}

ErrCode OdomBase::stop()
{
  if (thread_flag_ == true)
  {
    thread_flag_ = false;
    odom_thread_ptr_->join();
  }
#if (DEBUG_LEVEL > 1)
  INFO << "OdomBase Stop" << REND;
#endif
  return ErrCode_Success;
}

void OdomBase::stateMachine()
{
  vector<canbusData> buf;
  while (thread_flag_)
  {
    switch (self_state_)
    {
      case IDLE: {
        self_state_ = READ;
        break;
      }
      case READ: {
        usleep(1000);
        ptrCanbridge_ptr_->Read(buf);
        if (buf.empty())
          break;
        self_state_ = READ_DATA;
        break;
      }
      case READ_DATA: {
        prepareMsg(buf);
        self_state_ = READ;
        break;
      }
    }
  }
}
}  // namespace sensor
}  // namespace robosense