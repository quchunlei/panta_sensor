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

#ifdef ROS_FOUND

#include <panta_common/interface/sensor/lidar_points_interface.h>
#include <panta_common/msg/ros_msg_translator.h>
#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <sensor_msgs/PointCloud2.h>

namespace robosense
{
namespace sensor
{
class LidarPointsRosAdapter : virtual public common::LidarPointsInterface
{
public:
  LidarPointsRosAdapter() = default;
  ~LidarPointsRosAdapter() {stop();}

  common::ErrCode init(const YAML::Node &config);
  inline common::ErrCode start()
  {
    #if (DEBUG_LEVEL > 1)
    INFO << "LidarPointsRosAdapter start!" << REND;
    #endif
    return common::ErrCode_Success;
  }
  inline common::ErrCode stop()
  {
    #if (DEBUG_LEVEL > 1)
    INFO << "LidarPointsRosAdapter stop!" << REND;
    #endif
    return common::ErrCode_Success;
  }
  void regRecvCallback(const std::function<void(const common::LidarPointsMsg &)> callBack);
  inline void regExceptionCallback(const std::function<void(const common::ErrCode &)> excallBack)
  {
    #if (DEBUG_LEVEL > 0)
    WARNING << "LidarPointsRosAdapter : Exception is not supported !!" << REND;
    #endif
  }
  void send(const common::LidarPointsMsg &msg);

private:
  void localLidarPointsCallback(const sensor_msgs::PointCloud2 &msg);

private:
  std::shared_ptr<ros::NodeHandle> nh_;
  std::vector<std::function<void(const common::LidarPointsMsg &)>> lidarPointscbs_;
  ros::Publisher lidar_points_pub_;
  ros::Subscriber lidar_points_sub_;
  std::string frame_id_;
private:
  static const uint16_t supported_api_ = 0x0020;
};
} // namespace sensor
} // namespace robosense
#endif // ROS_FOUND