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

#include <memory>
#include <panta_common/common.h>
#include <rs_lidar/lidar_base.h>
#include "panta_sensor/driver/gnss/gnss_BT708.h"
#include "panta_sensor/driver/gnss/gnss_POSLVX.h"
#include "panta_sensor/driver/imu/imu_TL740D.h"
#include "panta_sensor/driver/imu/imu_HWT605.h"
#include "panta_sensor/driver/imu/imu_DY551.h"
#include "panta_sensor/driver/ins/ins_XWG13668.h"
#include "panta_sensor/driver/odom/odom_byd.h"
#include "panta_sensor/driver/odom/odom_jili.h"
#include "panta_sensor/driver/odom/odom_bieke.h"
#include "panta_sensor/ros/imu_ros_adapter.h"
#include "panta_sensor/ros/gnss_ros_adapter.h"
#include "panta_sensor/ros/odom_ros_adapter.h"
#include "panta_sensor/ros/lidar_points_ros_adapter.h"
#include "panta_sensor/ros/lidar_packets_ros_adapter.h"
#include "panta_sensor/proto/imu_proto_adapter.h"
#include "panta_sensor/proto/lidar_points_proto_adapter.h"
#include "panta_sensor/proto/lidar_packets_proto_adapter.h"
#include "panta_sensor/proto/gnss_proto_adapter.h"
#include "panta_sensor/proto/odom_proto_adapter.h"

namespace robosense
{
namespace sensor
{
enum MessageSource
{
  MessageSourceNotUsed = 0,
  MessageSourceRsDriver = 1,
  MessageSourceRos = 2,
  MessageSourceProto = 3
};
class SensorManager : virtual public common::CommonBase
{
public:
  SensorManager();
  ~SensorManager();

  common::ErrCode init(const YAML::Node& sensor_config, bool use_ros, bool use_proto, std::string config_path);
  common::ErrCode start();
  common::ErrCode stop();

  inline void regExceptionCallback(const std::function<void(const common::ErrCode&)>& callBack)
  {
    std::lock_guard<std::mutex> lock(mutex_exception_);
    excbs_.emplace_back(callBack);
  }
  inline void regRecvCallback(const std::function<void(const common::ImuMsg&)>& callBack)
  {
    std::lock_guard<std::mutex> lock(mutex_imu_);
    imucbs_.emplace_back(callBack);
  }
  inline void regRecvCallback(const std::function<void(const common::GnssMsg&)>& callBack)
  {
    std::lock_guard<std::mutex> lock(mutex_gnss_);
    gnsscbs_.emplace_back(callBack);
  }
  inline void regRecvCallback(const std::function<void(const common::OdomMsg&)>& callBack)
  {
    std::lock_guard<std::mutex> lock(mutex_odom_);
    odomcbs_.emplace_back(callBack);
  }
  inline void regRecvCallback(const std::function<void(const common::LidarPointsMsg&)>& callBack)
  {
    std::lock_guard<std::mutex> lock(mutex_points_);
    lidarPointscbs_.emplace_back(callBack);
  }
  inline bool getUseImu()
  {
    return use_imu_;
  }
  inline bool getUseGnss()
  {
    return use_gnss_;
  }
  inline bool getUseOdom()
  {
    return use_odom_;
  }
  inline bool getUseLidar()
  {
    return use_lidar_;
  }

private:
  template <typename T>
  T* configReceiver(const YAML::Node& sensor_config, const std::string& type, const int& msg_source);

  template <typename T>
  T* configTransmitter(const YAML::Node& sensor_config, const std::string& type, bool send_msg_ros,
                       bool send_msg_proto);

private:
  common::ErrCode initImu(const YAML::Node& config);
  common::ErrCode initGnss(const YAML::Node& config);
  common::ErrCode initOdom(const YAML::Node& config);
  common::ErrCode initLidar(const YAML::Node& config);

private:
  inline void localImuCallback(const common::ImuMsg& msg)
  {
    std::lock_guard<std::mutex> lock(mutex_imu_);
    if (!imucbs_.empty())
    {
      for (auto& cb : imucbs_)
        cb(msg);
    }
  }
  inline void localGnssCallback(const common::GnssMsg& msg)
  {
    std::lock_guard<std::mutex> lock(mutex_gnss_);
    if (!gnsscbs_.empty())
    {
      for (auto& cb : gnsscbs_)
        cb(msg);
    }
  }
  inline void localOdomCallback(const common::OdomMsg& msg)
  {
    std::lock_guard<std::mutex> lock(mutex_odom_);
    if (!odomcbs_.empty())
    {
      for (auto& cb : odomcbs_)
        cb(msg);
    }
  }
  inline void localLidarPointsCallback(const common::LidarPointsMsg& msg)
  {
    std::lock_guard<std::mutex> lock(mutex_points_);
    if (!lidarPointscbs_.empty())
    {
      for (auto& cb : lidarPointscbs_)
        cb(msg);
    }
  }

  inline void localExceptionCallback(const common::ErrCode& code)
  {
    std::lock_guard<std::mutex> lock(mutex_exception_);
    if (!excbs_.empty())
    {
      for (auto& excb : excbs_)
        excb(code);
    }
  }

  std::vector<std::function<void(const common::ErrCode&)>> excbs_;
  std::vector<std::function<void(const common::ImuMsg&)>> imucbs_;
  std::vector<std::function<void(const common::GnssMsg&)>> gnsscbs_;
  std::vector<std::function<void(const common::OdomMsg&)>> odomcbs_;
  std::vector<std::function<void(const common::LidarPointsMsg&)>> lidarPointscbs_;

private:
  bool use_ros_;
  bool use_proto_;
  bool run_flag_;
  bool lidarpkts_run_flag_;
  bool lidarpoints_run_flag_;
  std::string config_path_;
  bool use_imu_;
  bool use_gnss_;
  bool use_odom_;
  bool use_lidar_;

private:
  common::ImuInterface* imu_receiver_;
  common::GnssInterface* gnss_receiver_;
  common::OdomInterface* odom_receiver_;
  std::vector<common::LidarPacketsInterface*> lidar_packets_receivers_;
  std::vector<common::LidarPointsInterface*> lidar_points_receivers_;
  common::ImuInterface* imu_ros_transmitter_;
  common::ImuInterface* imu_proto_transmitter_;
  common::GnssInterface* gnss_ros_transmitter_;
  common::GnssInterface* gnss_proto_transmitter_;
  common::OdomInterface* odom_ros_transmitter_;
  common::OdomInterface* odom_proto_transmitter_;
  std::vector<common::LidarPacketsInterface*> lidar_packets_ros_transmitters_;
  std::vector<common::LidarPacketsInterface*> lidar_packets_proto_transmitters_;
  std::vector<common::LidarPointsInterface*> lidar_points_ros_transmitters_;
  std::vector<common::LidarPointsInterface*> lidar_points_proto_transmitters_;
  std::shared_ptr<std::thread> ros_thread_ptr_;

private:
  std::mutex mutex_exception_;
  std::mutex mutex_odom_;
  std::mutex mutex_gnss_;
  std::mutex mutex_imu_;
  std::mutex mutex_points_;
  std::mutex mutex_scan_;
  std::mutex mutex_packets_;

private:
  template <class R>
  R* construct(const std::string& device_type, const std::string& frame_id);
  template <class R, class T>
  inline R* localConstruct(uint16_t api_request)
  {
    if ((api_request | T::getApi()) != 0)
    {
      return dynamic_cast<R*>(new T);
    }
    else
    {
      return NULL;
    }
  }
  std::map<std::string, std::map<std::string, common::CommonBase*>> sensors_;
};
};  // namespace sensor
}  // namespace robosense
