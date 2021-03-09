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
#include "panta_sensor/driver/gnss/gnss_BT708.h"
namespace robosense
{
namespace sensor
{
using namespace robosense::common;
bool GnssBT708::autoConnect()
{
  auto x = [&]() {
    std::vector<uint8_t> data_buf;
    std::vector<std::string> data;
    usleep(100000);
    if (gnss_ser_->serialRead(data_buf, 256, parameter_.timeout) < 0)
    {
      gnss_ser_->closePort();
      return false;
    }
    if (processData(data_buf, data))
    {
      if (data[0] == "$GPTXT" || data[0] == "$GPGGA" || data[0] == "$GPRMC" || data[0] == "$GPGSV")
      {
        return true;
      }
    }
    gnss_ser_->closePort();
    return false;
  };
  return gnss_ser_->autoConnect(x, gnss_ser_, parameter_.baudrate);
}

void GnssBT708::prepareMsg(const std::vector<std::string>& data, bool use_gnss_clock)
{
  GnssMsg gnss_msg;
  OdomMsg odom_msg;
  if (data[0] == "$GPGGA" && data.size() > 10)
  {
    gnss_seq_++;
    double in_latitude = 0;
    double in_longitude = 0;
    double snglat = (atof(data[2].c_str()) / 100);
    double snglatmins = fmod(snglat, 1);
    snglat = snglat - snglatmins;
    snglatmins = snglatmins * 100 / 60.0;
    in_latitude = snglat + snglatmins;
    if (data[3] == "S")
    {
      in_latitude = 0 - in_latitude;
    }
    double snglon = (atof(data[4].c_str()) / 100);
    double snglonmins = fmod(snglon, 1);
    snglon = snglon - snglonmins;
    snglonmins = snglonmins * 100 / 60.0;
    in_longitude = snglon + snglonmins;
    if (data[5] == "W")
    {
      in_longitude = 0 - in_longitude;
    }
    gnss_msg.pos[0] = in_latitude;
    gnss_msg.pos[1] = in_longitude;
    gnss_msg.pos[2] = atof(data[9].c_str());
    gnss_msg.sat_cnt = atof(data[7].c_str());
    gnss_msg.timestamp = getTime();
    gnss_msg.seq = gnss_seq_;
    gnss_msg.type = 'G';
    gnss_msg.frame_id = parameter_.frame_id;
    gnss_msg.parent_frame_id = gnss_msg.frame_id;
    gnss_msg.status = 1;
    odom_msg.frame_id = gnss_msg.frame_id;
    odom_msg.parent_frame_id = odom_msg.frame_id;
    odom_msg.seq = gnss_msg.seq;
    odom_msg.timestamp = gnss_msg.timestamp;
    odom_msg.linear_vel[0] = gnss_msg.linear_vel[0];
    odom_msg.linear_vel[1] = gnss_msg.linear_vel[1];
    odom_msg.linear_vel[2] = gnss_msg.linear_vel[2];
    if (gnss_msg.sat_cnt < parameter_.min_satellite)
    {
      reportError(ErrCode_GnssDriverLacksatellite);
    }
    runCallBack(odom_msg);
    runCallBack(gnss_msg);
  }
}
}  // namespace sensor
}  // namespace robosense