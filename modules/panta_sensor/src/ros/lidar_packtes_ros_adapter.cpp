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

#ifdef ROS_FOUND
#include "panta_sensor/ros/lidar_packets_ros_adapter.h"
namespace robosense
{
namespace sensor
{
using namespace robosense::common;
ErrCode LidarPacketsRosAdapter::init(const YAML::Node& config)
{
  setName("LiDAR_packets_RosAdapter");
  setinitFlag(true);
  int msg_source;
  bool send_packets_ros;
  YAML::Node ros_config = yamlSubNodeAbort(config, "ros");
  nh_ = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle());
  std::string ros_recv_topic;
  yamlReadAbort<std::string>(ros_config, "ros_recv_packets_topic", ros_recv_topic);
  std::string ros_send_topic;
  yamlReadAbort<std::string>(ros_config, "ros_send_packets_topic", ros_send_topic);
  yamlRead<int>(config, "msg_source", msg_source);
  yamlRead<bool>(config, "send_packets_ros", send_packets_ros, false);
  if (msg_source == 2)
  {
    lidar_packets_difop_sub_ =
        nh_->subscribe(ros_recv_topic + "_difop", 1, &LidarPacketsRosAdapter::localLidarPacketsdifopCallback, this);
    lidar_packets_msop_sub_ =
        nh_->subscribe(ros_recv_topic, 1, &LidarPacketsRosAdapter::localLidarPacketsmsopCallback, this);
  }
  if (send_packets_ros)
  {
    lidar_packets_difop_pub_ = nh_->advertise<rslidar_msgs::rslidarPacket>(ros_send_topic + "_difop", 10);
    lidar_packets_msop_pub_ = nh_->advertise<rslidar_msgs::rslidarScan>(ros_send_topic, 10);
  }
  return ErrCode_Success;
}

void LidarPacketsRosAdapter::regRecvCallback(const std::function<void(const common::LidarScanMsg&)> callBack)
{
  lidar_packets_msop_cbs_.emplace_back(callBack);
}
void LidarPacketsRosAdapter::regRecvCallback(const std::function<void(const common::LidarPacketMsg&)> callBack)
{
  lidar_packets_difop_cbs_.emplace_back(callBack);
}
void LidarPacketsRosAdapter::send_msop(const LidarScanMsg& msg)  // Will send NavSatStatus and Odometry
{
  lidar_packets_msop_pub_.publish(toRosMsg(msg));
}
void LidarPacketsRosAdapter::send_difop(const LidarPacketMsg& msg)  // Will send NavSatStatus and Odometry
{
  lidar_packets_difop_pub_.publish(toRosMsg(msg));
}

void LidarPacketsRosAdapter::localLidarPacketsmsopCallback(const rslidar_msgs::rslidarScan& msg)
{
  for (auto& cb : lidar_packets_msop_cbs_)
  {
    cb(toRsMsg(msg));
  }
}
void LidarPacketsRosAdapter::localLidarPacketsdifopCallback(const rslidar_msgs::rslidarPacket& msg)
{
  for (auto& cb : lidar_packets_difop_cbs_)
  {
    cb(toRsMsg(msg));
  }
}
}  // namespace sensor
}  // namespace robosense

#endif  // ROS_FOUND