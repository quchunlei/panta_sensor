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

#ifndef __ROBOSENSE_PREPROCESS_INTERFACE_H__
#define __ROBOSENSE_PREPROCESS_INTERFACE_H__

#include <panta_common/common.h>
#include <panta_common/msg/rs_msg/gnss_msg.h>
#include <panta_common/msg/rs_msg/imu_msg.h>
#include <panta_common/msg/rs_msg/lidar_packet_msg.h>
#include <panta_common/msg/rs_msg/lidar_points_msg.h>
#include <panta_common/msg/rs_msg/odom_msg.h>
#include <thread>

namespace robosense {
namespace common {
class PreprocessInterface : virtual public CommonBase {
 public:
  PreprocessInterface() = default;
  virtual ~PreprocessInterface() = default;

  virtual ErrCode init(const YAML::Node &config) = 0;
  virtual ErrCode reset() = 0;
  virtual ErrCode start() = 0;
  virtual ErrCode stop() = 0;

  virtual void regExceptionCallback(
      const std::function<void(const ErrCode &)> excallBack) = 0;

  virtual void imuCallback(const ImuMsg &msg) = 0;
  virtual void gnssCallback(const GnssMsg &msg) = 0;
  virtual void odomCallback(const OdomMsg &msg) = 0;
  virtual void lidarPointsCallback(const LidarPointsMsg &msg) = 0;

  virtual void regRecvCallback(
      const std::function<void(const ImuMsg &)> callBack) = 0;
  virtual void regRecvCallback(
      const std::function<void(const GnssMsg &)> callBack) = 0;
  virtual void regRecvCallback(
      const std::function<void(const OdomMsg &)> callBack) = 0;
  virtual void regRecvCallback(
      const std::function<void(const LidarPointsMsg &)> callBack,const bool &motion_correct, const std::string &frame_id) = 0;
};
}  // namespace common
}  // namespace robosense

#endif /*__ROBOSENSE_PREPROCESS_INTERFACE_H__*/
