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
#include "panta_sensor/driver/ins/ins_XWG13668.h"
namespace robosense
{
namespace sensor
{
using namespace robosense::common;
bool InsXWG13668::autoConnect()
{
  auto x = [&]() {
    std::vector<uint8_t> data_buf;
    std::vector<std::string> data;
    if (gnss_ser_->serialRead(data_buf, 256, parameter_.timeout) < 0)
    {
      gnss_ser_->closePort();
      return false;
    }
    if (processData(data_buf, data))
    {
      if (data[0] == "$GPFPD" || data[0] == "$GTIMU" || data[0] == "$GPGGA")
      {
        return true;
      }
    }
    gnss_ser_->closePort();
    return false;
  };
  return gnss_ser_->autoConnect(x, gnss_ser_, parameter_.baudrate);
}

void InsXWG13668::prepareMsg(const std::vector<std::string>& data, bool use_gnss_clock)
{
  ImuMsg imu_msg;
  OdomMsg odom_msg;
  if (data[0] == "$GPFPD" && data.size() > 15)
  {
    gnss_seq_++;
    odom_seq_++;
    gnss_msg_.orien[0] = atof(data[5].c_str()) / 180.0 * M_PI;
    gnss_msg_.orien[1] = atof(data[4].c_str()) / 180.0 * M_PI;
    double yaw = atof(data[3].c_str());
    if (yaw >= 0.0 && yaw <= 270.0)
      gnss_msg_.orien[2] = (90.0 - yaw) / 180.0 * M_PI;
    else
      gnss_msg_.orien[2] = (450.0 - yaw) / 180.0 * M_PI;
    if (gnss_msg_.orien[2] < -M_PI)
      gnss_msg_.orien[2] = gnss_msg_.orien[2] + 2 * M_PI;
    else if (gnss_msg_.orien[2] > M_PI)
      gnss_msg_.orien[2] = gnss_msg_.orien[2] - 2 * M_PI;

    gnss_msg_.pos[0] = atof(data[6].c_str());
    gnss_msg_.pos[1] = atof(data[7].c_str());
    gnss_msg_.pos[2] = atof(data[8].c_str());

    gnss_msg_.linear_vel[0] = atof(data[9].c_str());
    gnss_msg_.linear_vel[1] = atof(data[10].c_str());
    gnss_msg_.linear_vel[2] = atof(data[11].c_str());
    gnss_msg_.sat_cnt = atof(data[13].c_str()) + atof(data[14].c_str());

    if (!use_gnss_clock)
    {
      gnss_msg_.timestamp = getTime();
    }
    else
    {
      double gnss_time = atof(data[1].c_str()) * 604800 + atof(data[2].c_str());
      int gnss_time_int = floor(gnss_time);
      float gnss_time_float = gnss_time - gnss_time_int;
      double unix_time = gps2unix(gnss_time_int);
      gnss_msg_.timestamp = unix_time + gnss_time_float;
    }

    gnss_msg_.seq = gnss_seq_;
    gnss_msg_.type = 'R';
    gnss_msg_.frame_id = parameter_.frame_id;
    gnss_msg_.parent_frame_id = gnss_msg_.frame_id;

    switch (data[15][1])
    {
      case 'C':
      case '4':
      case '5':
        gnss_msg_.status = 0;
        break;
      case 'B':
        gnss_msg_.status = 1;
        break;
      default:
        gnss_msg_.status = -1;
        break;
    }

    if (gnss_msg_.sat_cnt < parameter_.min_satellite)
    {
      reportError(ErrCode_GnssDriverLacksatellite);
    }

    odom_msg.linear_vel[0] = gnss_msg_.linear_vel[0];
    odom_msg.linear_vel[1] = gnss_msg_.linear_vel[1];
    odom_msg.linear_vel[2] = gnss_msg_.linear_vel[2];
    odom_msg.timestamp = gnss_msg_.timestamp;
    odom_msg.seq = odom_seq_;
    odom_msg.frame_id = gnss_msg_.frame_id;
    odom_msg.parent_frame_id = odom_msg.frame_id;

    GnssBase::runCallBack(gnss_msg_);
    GnssBase::runCallBack(odom_msg);
  }
  else if (data[0] == "$GTIMU" && data.size() > 9)
  {
    imu_seq_++;

    imu_msg.orien[0] = gnss_msg_.orien[0];
    imu_msg.orien[1] = gnss_msg_.orien[1];
    imu_msg.orien[2] = gnss_msg_.orien[2];

    imu_msg.angular_vel[0] = atof(data[3].c_str()) / 180.0 * M_PI;
    imu_msg.angular_vel[1] = atof(data[4].c_str()) / 180.0 * M_PI;
    imu_msg.angular_vel[2] = atof(data[5].c_str()) / 180.0 * M_PI;

    imu_msg.acc[0] = atof(data[6].c_str());
    imu_msg.acc[1] = atof(data[7].c_str());
    imu_msg.acc[2] = atof(data[8].c_str());
    if (!use_gnss_clock)
    {
      imu_msg.timestamp = getTime();
    }
    else
    {
      double gnss_time = atof(data[1].c_str()) * 604800 + atof(data[2].c_str());
      int gnss_time_int = floor(gnss_time);
      float gnss_time_float = gnss_time - gnss_time_int;
      double unix_time = gps2unix(gnss_time_int);
      imu_msg.timestamp = unix_time + gnss_time_float;
    }
    imu_msg.seq = imu_seq_;
    imu_msg.frame_id = parameter_.frame_id;
    imu_msg.parent_frame_id = imu_msg.frame_id;

    if (std::abs(imu_msg.angular_vel[2]) > parameter_.warning_gyro_z)  /////////////
    {
      reportError(ErrCode_GnssDriverGyroztoolarge);
    }
    runCallBack(imu_msg);
  }
  else if (data[0] == "$GPGGA" && data.size() > 17)
  {
    gnss_msg_.pos_cov[0] = (atof(data[8].c_str()) * 0.02) * (atof(data[8].c_str()) * 0.02);
    gnss_msg_.pos_cov[4] = (atof(data[8].c_str()) * 0.02) * (atof(data[8].c_str()) * 0.02);
    gnss_msg_.pos_cov[8] = (atof(data[8].c_str()) * 0.08) * (atof(data[8].c_str()) * 0.08);
    GnssBase::runCallBack(gnss_msg_);
  }
}
}  // namespace sensor
}  // namespace robosense