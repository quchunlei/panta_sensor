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
#include "panta_sensor/driver/imu/imu_DY551.h"
#define MARKBYTE_1 0xBD
#define MARKBYTE_2 0xDB
#define MARKBYTE_3 0x04
#include <ctime>
namespace robosense
{
namespace sensor
{
using namespace robosense::common;
using namespace robosense::WitImu;
ImuDY551::ImuDY551()
{
  read_rec_.resize(dy_msg_len);
  write_rec_.resize(dy_msg_len);
}
void ImuDY551::prepareMsg()
{
  int count = (int)imu_ser_->serialRead(read_rec_, dy_msg_len, imu_parameter_.timeout);
  if (count < 0)
  {
    reportError(ErrCode_ImuDriverInterrupt);
    self_state_ = CHECK_CONNECTION;
  }
  if (!read_rec_.empty())
  {
    write_rec_.swap(read_rec_);
    decode_msg(count, write_rec_);
  }

  imu_seq_++;
  ImuMsg imu_msg;
  imu_msg.orien[0] = yj9_data_.eular_angle_.roll / 180.0 * M_PI;
  imu_msg.orien[1] = yj9_data_.eular_angle_.pitch / 180.0 * M_PI;
  imu_msg.orien[2] = yj9_data_.eular_angle_.yaw / 180.0 * M_PI;
  imu_msg.acc[0] = yj9_data_.acc_.acc_x;
  imu_msg.acc[1] = yj9_data_.acc_.acc_y;
  imu_msg.acc[2] = yj9_data_.acc_.acc_z;
  imu_msg.angular_vel[0] = yj9_data_.gyr_.gyr_x;
  imu_msg.angular_vel[1] = yj9_data_.gyr_.gyr_y;
  imu_msg.angular_vel[2] = yj9_data_.gyr_.gyr_z;
  std::cerr << "TIME:" << yj9_data_.time_.year << "-" << yj9_data_.time_.month << "-" << yj9_data_.time_.day;
  std::cerr << yj9_data_.time_.hour << "-" << yj9_data_.time_.minute << "-" << yj9_data_.time_.sec << std::endl;
  std::cerr << "-------------------" << std::endl;
  imu_msg.timestamp = getTime();
  imu_msg.seq = imu_seq_;
  imu_msg.frame_id = imu_parameter_.frame_id;
  imu_msg.parent_frame_id = imu_msg.frame_id;
  if (std::abs(imu_msg.angular_vel[2]) > imu_parameter_.warning_gyro_z)
  {
    reportError(ErrCode_ImuDriverGyroztoolarge);
  }  // Gyro Z is too large!

  runCallBack(imu_msg);
}

int ImuDY551::checkMarkBit()
{
  std::vector<uint8_t> buf;
  if ((int)imu_ser_->serialRead(buf, 3, imu_parameter_.timeout) < 0)
  {
    reportError(ErrCode_ImuDriverConnectfail);
    return 0;
  }
  if (buf[0] == (uint8_t)(MARKBYTE_1) && buf[1] == (uint8_t)(MARKBYTE_2) && buf[2] == (uint8_t)(MARKBYTE_3))
  {
    return 2;
  }
  return 1;
}

bool ImuDY551::autoConnect()
{
  auto x = [&]() {
    for (int j = 0; j < 34; j++)
    {
      std::vector<uint8_t> buf;
      if (imu_ser_->serialRead(buf, 3, imu_parameter_.timeout) < 0)
      {
        imu_ser_->closePort();
        return false;
      }
      if (buf[0] != (uint8_t)(MARKBYTE_1) || buf[1] != (uint8_t)(MARKBYTE_2) || buf[2] != (uint8_t)(MARKBYTE_3))
      {
        continue;
      }
      else
      {
        return true;
      }
    }
    imu_ser_->closePort();
    return false;
  };
  return imu_ser_->autoConnect(x, imu_ser_, imu_parameter_.baudrate);
}

void ImuDY551::decode_msg(int count, std::vector<uint8_t>& buf_rec)
{
  analysis(buf_rec);
  buf_rec.clear();
}

void ImuDY551::analysis(std::vector<uint8_t>& buf_)
{
  // Angle out
  std::array<int16_t, 3> angle_hex;
  angle_hex[0] = (buf_[1] << 8 | buf_[0]);
  angle_hex[1] = (buf_[3] << 8 | buf_[2]);
  angle_hex[2] = (buf_[5] << 8 | buf_[4]);
  translation_3d(angle_hex);
  yj9_data_.eular_angle_.roll = (double)angle_hex[0] / dy_max_range * dy_angle_scale;
  yj9_data_.eular_angle_.pitch = (double)angle_hex[1] / dy_max_range * dy_angle_scale;
  yj9_data_.eular_angle_.yaw = (double)angle_hex[2] / dy_max_range * dy_angle_scale;

  // Gyr out;
  std::array<int16_t, 3> gyr_hex;
  gyr_hex[0] = (buf_[11] << 8 | buf_[10]);
  gyr_hex[1] = (buf_[13] << 8 | buf_[12]);
  gyr_hex[2] = (buf_[15] << 8 | buf_[14]);
  translation_3d(gyr_hex);
  yj9_data_.gyr_.gyr_x = (double)gyr_hex[0] / dy_max_range * dy_gyr_scale * dy_deg2rad;
  yj9_data_.gyr_.gyr_y = (double)gyr_hex[1] / dy_max_range * dy_gyr_scale * dy_deg2rad;
  yj9_data_.gyr_.gyr_z = (double)gyr_hex[2] / dy_max_range * dy_gyr_scale * dy_deg2rad;

  // Acc out;
  std::array<int16_t, 3> acc_hex;
  acc_hex[0] = (buf_[17] << 8 | buf_[16]);
  acc_hex[1] = (buf_[19] << 8 | buf_[18]);
  acc_hex[2] = (buf_[21] << 8 | buf_[20]);
  translation_3d(acc_hex);
  yj9_data_.acc_.acc_x = (double)acc_hex[0] / dy_max_range * dy_acc_scale * dy_g;
  yj9_data_.acc_.acc_y = (double)acc_hex[1] / dy_max_range * dy_acc_scale * dy_g;
  yj9_data_.acc_.acc_z = (double)acc_hex[2] / dy_max_range * dy_acc_scale * dy_g;

  // temperature
  int16_t tem_hex;
  tem_hex = (buf_[23] << 8 | buf_[22]);
  yj9_data_.temperature_ = (double)tem_hex / dy_max_range * dy_temp_scale;
  // std::cerr << "Temperature: " << yj9_data_.temperature_ << std::endl;

  // Time calculate
  int32_t time_hex;
  time_hex = (buf_[27] << 24 | buf_[26] << 16 | buf_[25] << 8 | buf_[24]);
  int32_t relative_time = time_hex / 1000;  // 从传感器启动到现在的秒数
  secondToYMDHMS(relative_time);
}

}  // namespace sensor
}  // namespace robosense
