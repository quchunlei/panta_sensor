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
#include "panta_sensor/ros/gnss_ros_adapter.h"
namespace robosense
{
namespace sensor
{
using namespace robosense::common;
ErrCode GnssRosAdapter::init(const YAML::Node& config)
{
  setName("GnssRosAdapter");
  setinitFlag(true);
  int msg_source;
  bool send_msg_ros;
  std::string ros_recv_topic;
  std::string ros_send_topic;
  YAML::Node ros_config = yamlSubNodeAbort(config, "ros");
  YAML::Node common_config = yamlSubNodeAbort(config, "common");
  yamlRead<int>(common_config, "msg_source", msg_source);
  yamlRead<bool>(common_config, "send_msg_ros", send_msg_ros, false);
  yamlReadAbort<std::string>(ros_config, "ros_recv_topic", ros_recv_topic);
  yamlReadAbort<std::string>(ros_config, "ros_send_topic", ros_send_topic);
  nh_ = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle());
  if (msg_source == 2)
  {
    gnss_sub_ = nh_->subscribe(ros_recv_topic, 1, &GnssRosAdapter::localgnssCallback, this);
  }
  if (send_msg_ros)
  {
    gnss_pub_ = nh_->advertise<sensor_msgs::NavSatFix>(ros_send_topic, 10);
    gnss_odom_pub_ = nh_->advertise<nav_msgs::Odometry>(ros_send_topic + "/odom", 10);
  }
  return ErrCode_Success;
}

void GnssRosAdapter::regRecvCallback(const std::function<void(const GnssMsg&)> callBack)
{
  gnss_cb_.emplace_back(callBack);
}

void GnssRosAdapter::send(const GnssMsg& msg)
{
  gnss_pub_.publish(toRosMsg(msg));
  gnss_odom_pub_.publish(toRosMsgOdom(msg));
}

void GnssRosAdapter::localgnssCallback(const sensor_msgs::NavSatFix& msg)
{
  for (auto& cb : gnss_cb_)
  {
    cb(toRsMsg(msg));
  }
}
}  // namespace sensor
}  // namespace robosense

#endif  // ROS_FOUND