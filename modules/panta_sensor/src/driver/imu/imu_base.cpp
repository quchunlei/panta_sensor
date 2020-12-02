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
#include "panta_sensor/driver/imu/imu_base.h"
namespace robosense
{
namespace sensor
{
using namespace robosense::common;
ErrCode ImuBase::init(const YAML::Node& config)
{
  setinitFlag(true);
  YAML::Node driver_config = yamlSubNodeAbort(config, "driver");
  yamlRead<std::string>(driver_config, "device_type", imu_parameter_.device_type, "");
  yamlRead<bool>(driver_config, "auto_scan_port", imu_parameter_.auto_scan_port, true);
  yamlRead<std::string>(driver_config, "port_name", imu_parameter_.port_name, "/dev/ttyUSB0");
  yamlRead<int>(driver_config, "baudrate", imu_parameter_.baudrate, 115200);
  yamlRead<std::string>(driver_config, "frame_id", imu_parameter_.frame_id, "/imu");
  yamlRead<bool>(driver_config, "do_reconnect", imu_parameter_.do_reconnect, false);
  yamlRead<int>(driver_config, "reconnect_interval", imu_parameter_.reconnect_interval, 3);
  yamlRead<int>(driver_config, "reconnect_attempts", imu_parameter_.reconnect_attemps, 3);
  yamlRead<double>(driver_config, "warning_gyro_z", imu_parameter_.warning_gyro_z, 3.14);
  yamlRead<double>(driver_config, "timeout", imu_parameter_.timeout, 0.1);

  imu_thread_ = std::make_shared<std::thread>();
  imu_ser_ = std::make_shared<Serial>();
  imu_seq_ = 0;
  imucb_.reserve(10);
  excb_ = NULL;
  thread_flag_ = false;
  self_state_ = IDLE;
  return ErrCode_Success;
}

ErrCode ImuBase::start()
{
  if (imucb_.empty())
  {
    ERROR << "ImuBase: Please register at least one callback function first!" << REND;
    exit(-1);
  }
  if (thread_flag_ == false)
  {
    imu_seq_ = 0;
    thread_flag_ = true;
    self_state_ = IDLE;
    const auto& func1 = [this] { stateMachine(); };
    imu_thread_ = std::make_shared<std::thread>(func1);
#if (DEBUG_LEVEL > 1)
    INFO << "ImuBase Start !" << REND;
#endif
  }
  return ErrCode_Success;
}

ErrCode ImuBase::stop()
{
  if (thread_flag_ == true)
  {
    thread_flag_ = false;
    imu_thread_->join();
  }
#if (DEBUG_LEVEL > 1)
  INFO << "ImuBase stop" << REND;
#endif
  return ErrCode_Success;
}

void ImuBase::stateMachine()
{
  int count = 0;
  while (thread_flag_)
  {
    switch (self_state_)
    {
      case IDLE: {
        self_state_ = CONNECT;
        break;
      }
      case CONNECT: {
        if (imu_parameter_.auto_scan_port)
        {
          if (!autoConnect())
          {
#if (DEBUG_LEVEL > 0)
            WARNING << "IMU AUTO_CONNECT failed" << REND;
#endif
            reportError(ErrCode_ImuDriverConnectfail);
            self_state_ = CHECK_CONNECTION;
          }
          else
          {
#if (DEBUG_LEVEL > 1)
            INFO << "IMU: Auto connect successfully!" << REND;
#endif
            count = 0;
            self_state_ = CHECK_MARK_BIT;
          }
          break;
        }
        else
        {
          if (!imu_ser_->connect(imu_parameter_.port_name, imu_parameter_.baudrate, 8, 'N', 1))
          {
#if (DEBUG_LEVEL > 0)
            WARNING << "IMU MANUAL_CONNECT failed" << REND;
#endif
            reportError(ErrCode_ImuDriverConnectfail);
            self_state_ = CHECK_CONNECTION;
          }
          else
          {
#if (DEBUG_LEVEL > 1)
            INFO << "IMU: Manually connect successfully!" << REND;
#endif
            count = 0;
            self_state_ = CHECK_MARK_BIT;
          }
          break;
        }
      }
      case CHECK_MARK_BIT: {
        switch (checkMarkBit())
        {
          case (0):
            self_state_ = CHECK_CONNECTION;
            break;
          case (1):
            break;
          case (2):
            self_state_ = READ_DATA;
            break;
        }
        break;
      }
      case READ_DATA: {
        prepareMsg();
        self_state_ = CHECK_MARK_BIT;
        break;
      }
      case CHECK_CONNECTION: {
        if (imu_parameter_.do_reconnect)
        {
          count++;
          if (count > imu_parameter_.reconnect_attemps)
          {
            reportError(ErrCode_ImuDriverDisconnect);
            sleep(1);
            break;
          }
#if (DEBUG_LEVEL > 0)
          WARNING << "IMU driver will start reconnect in " << imu_parameter_.reconnect_interval << " seconds" << REND;
#endif
          sleep(imu_parameter_.reconnect_interval);
          self_state_ = CONNECT;
        }
        else
        {
          reportError(ErrCode_ImuDriverDisconnect);
          sleep(1);
          break;
        }
      }
    }
  }
}
}  // namespace sensor
}  // namespace robosense