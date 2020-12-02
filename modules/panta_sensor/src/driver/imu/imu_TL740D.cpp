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
#include "panta_sensor/driver/imu/imu_TL740D.h"
#define working_mode 0x70
#define G_ 9.80665;
#if (working_mode == 0x70)
#define MARKBYTE_1 0x68
#define MARKBYTE_2 0x1f
#define MARKBYTE_3 0x00
#define MARKBYTE_4 0x84
#endif
#if (working_mode == 0x71)
#define MARKBYTE_1 0x68
#define MARKBYTE_2 0x0D
#define MARKBYTE_3 0x00
#define MARKBYTE_4 0x84
#endif
#if (working_mode == 0x73)
#define MARKBYTE_1 0x68
#define MARKBYTE_2 0x10
#define MARKBYTE_3 0x00
#define MARKBYTE_4 0x84
#endif

namespace robosense
{
namespace sensor
{
using namespace robosense::common;
void ImuTL740D::prepareMsg()
{
  ImuMsg imu_msg;
  if (working_mode == 0x70)
  {
    std::vector<uint8_t> buf;
    if ((int)imu_ser_->serialRead(buf, 28, imu_parameter_.timeout) < 0)
    {
      reportError(ErrCode_ImuDriverInterrupt);
      self_state_ = CHECK_CONNECTION;
    }
    imu_seq_++;
    imu_msg.orien[0] = char_to_Float(buf, 0) / 180.0 * M_PI;
    imu_msg.orien[1] = -char_to_Float(buf, 3) / 180.0 * M_PI;
    imu_msg.orien[2] = char_to_Float(buf, 6) / 180.0 * M_PI;
    imu_msg.acc[0] = char_to_Float(buf, 9) / (float)10 * (float)G_;
    imu_msg.acc[1] = char_to_Float(buf, 12) / (float)10 * (float)G_;
    imu_msg.acc[2] = char_to_Float(buf, 15) / (float)10 * (float)G_;
    imu_msg.angular_vel[0] = char_to_Float(buf, 18) / 180.0 * M_PI;
    imu_msg.angular_vel[1] = char_to_Float(buf, 21) / 180.0 * M_PI;
    imu_msg.angular_vel[2] = char_to_Float(buf, 24) / 180.0 * M_PI;
    imu_msg.timestamp = getTime();
    imu_msg.seq = imu_seq_;
    imu_msg.frame_id = imu_parameter_.frame_id;
    imu_msg.parent_frame_id = imu_msg.frame_id;

    if (std::abs(imu_msg.angular_vel[2]) > imu_parameter_.warning_gyro_z)
    {
      reportError(ErrCode_ImuDriverGyroztoolarge);
    }  // Gyro Z is too large!
  }
  else if (working_mode == 0x73)
  {
    std::vector<uint8_t> buf;
    if ((int)imu_ser_->serialRead(buf, 13, imu_parameter_.timeout) < 0)
    {
      reportError(ErrCode_ImuDriverInterrupt);
      self_state_ = CHECK_CONNECTION;
    }
    imu_seq_++;
    imu_msg.angular_vel[2] = char_to_Float(buf, 0) / 180.0 * M_PI;
    imu_msg.acc[0] = char_to_Float(buf, 3) / (float)10 * (float)G_;
    imu_msg.acc[1] = char_to_Float(buf, 6) / (float)10 * (float)G_;
    imu_msg.orien[2] = char_to_Float(buf, 9) / 180.0 * M_PI;
    imu_msg.timestamp = getTime();
    imu_msg.seq = imu_seq_;
    imu_msg.frame_id = imu_parameter_.frame_id;
    imu_msg.parent_frame_id = imu_msg.frame_id;
  }

  else if (working_mode == 0x71)
  {
    std::vector<uint8_t> buf;
    if ((int)imu_ser_->serialRead(buf, 10, imu_parameter_.timeout) < 0)
    {
      reportError(ErrCode_ImuDriverInterrupt);
      self_state_ = CHECK_CONNECTION;
    }
    imu_seq_++;
    imu_msg.angular_vel[2] = char_to_Float(buf, 0) / 180.0 * M_PI;
    imu_msg.acc[0] = char_to_Float(buf, 3) / (float)10 * (float)G_;
    imu_msg.orien[2] = char_to_Float(buf, 6) / 180.0 * M_PI;
    imu_msg.timestamp = getTime();
    imu_msg.seq = imu_seq_;
    imu_msg.frame_id = imu_parameter_.frame_id;
    imu_msg.parent_frame_id = imu_msg.frame_id;
  }
  runCallBack(imu_msg);
}

int ImuTL740D::checkMarkBit()
{
  std::vector<uint8_t> buf;
  if ((int)imu_ser_->serialRead(buf, 1, imu_parameter_.timeout) < 0)
  {
    reportError(ErrCode_ImuDriverConnectfail);
    return 0;
  }
  if (buf[0] != (uint8_t)(MARKBYTE_1))
    return 1;
  buf.clear();
  if ((int)imu_ser_->serialRead(buf, 3, imu_parameter_.timeout) < 0)
  {
    reportError(ErrCode_ImuDriverConnectfail);
    return 0;
  }
  if (buf[0] == (uint8_t)(MARKBYTE_2) && buf[1] == (uint8_t)(MARKBYTE_3) && buf[2] == (uint8_t)(MARKBYTE_4))
  {
    return 2;
  }
  return 1;
}

bool ImuTL740D::autoConnect()
{
  auto x = [&]() {
    for (int j = 0; j < 32; j++)
    {
      std::vector<uint8_t> buf;
      if (imu_ser_->serialRead(buf, 1, imu_parameter_.timeout) < 0)
      {
        imu_ser_->closePort();
        return false;
      }
      if (buf[0] == (uint8_t)(MARKBYTE_1))
      {
        buf.clear();
        if (imu_ser_->serialRead(buf, 3, imu_parameter_.timeout) < 0)
        {
          imu_ser_->closePort();
          return false;
        }
        if (buf[0] == (uint8_t)(MARKBYTE_2) && buf[1] == (uint8_t)(MARKBYTE_3) && buf[2] == (uint8_t)(MARKBYTE_4))
        {
          return true;
        }
      }
    }
    imu_ser_->closePort();
    return false;
  };
  return imu_ser_->autoConnect(x, imu_ser_, imu_parameter_.baudrate);
}
float ImuTL740D::char_to_Float(std::vector<uint8_t>& _buf, int start_point)
{
  int p = (int)_buf[start_point + 2] | (int)(_buf[start_point + 1]) << 8 | (int)(_buf[start_point]) << 16;
  int base = 1;
  float val = 0;
  for (int i = 0; i < 5; i++)
  {
    val += base * (p & 0x0F);
    p >>= 4;
    base *= 10;
  }
  val = (p == 1) ? -val / 100.0f : val / 100.0f;
  return val;
}
}  // namespace sensor
}  // namespace robosense