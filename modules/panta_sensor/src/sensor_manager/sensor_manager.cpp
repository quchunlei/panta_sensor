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

#include "panta_sensor/sensor_manager/sensor_manager.h"

namespace robosense
{
namespace sensor
{
using namespace robosense::common;

SensorManager::SensorManager()
{
  use_ros_ = false;
  use_proto_ = false;
}
SensorManager::~SensorManager()
{
  stop();
  for (auto& sensor_itr : sensors_)
  {
    for (auto& dev_itr : sensor_itr.second)
    {
      delete dev_itr.second;
    }
  }
  sensors_.clear();
}

ErrCode SensorManager::init(const YAML::Node& sensor_config, bool use_ros, bool use_proto, std::string config_path)
{
  ///////////////////////////////////////////////////////////////
  //////////////////    Basic init part       //////////////////
  /////////////////////////////////////////////////////////////
  setName("sensor_manager");
  setinitFlag(true);
  config_path_ = config_path;
  use_ros_ = use_ros;
  use_proto_ = use_proto;
  run_flag_ = false;
  lidarpkts_run_flag_ = false;
  lidarpoints_run_flag_ = false;
  use_imu_ = false;
  use_gnss_ = false;
  use_odom_ = false;
  use_lidar_ = false;
  imu_receiver_ = nullptr;
  gnss_receiver_ = nullptr;
  odom_receiver_ = nullptr;
  imu_ros_transmitter_ = nullptr;
  gnss_ros_transmitter_ = nullptr;
  odom_ros_transmitter_ = nullptr;
  imu_proto_transmitter_ = nullptr;
  gnss_proto_transmitter_ = nullptr;
  odom_proto_transmitter_ = nullptr;

  ///////////////////////////////////////////////////////////////
  //////////////////    Check Ros init      /////////////////////
  ///////////////////////////////////////////////////////////////
#ifdef ROS_FOUND
  if (use_ros_)
  {
    if (!ros::isInitialized())
    {
      ERROR << "SensorManger: use_ros is set to TRUE but ros is not initialized()! Abort!" << REND;
      exit(-1);
    }
  }
#endif /*ROS_FOUND*/

  ///////////////////////////////////////////////////////////////
  //////////////    Massage interface init part     /////////////
  ///////////////////////////////////////////////////////////////
  initLidar(sensor_config);
  initImu(sensor_config);
  initGnss(sensor_config);
  initOdom(sensor_config);

  return ErrCode_Success;
}

ErrCode SensorManager::start()
{
  if (imu_receiver_ != nullptr)
  {
    imu_receiver_->start();
  }
  if (gnss_receiver_ != nullptr)
  {
    gnss_receiver_->start();
  }
  if (odom_receiver_ != nullptr)
  {
    odom_receiver_->start();
  }
  if (lidarpoints_run_flag_)
  {
    for (auto& iter : lidar_points_receivers_)
    {
      if (iter != nullptr)
        iter->start();
    }
  }
  if (lidarpkts_run_flag_)
  {
    for (auto& iter : lidar_packets_receivers_)
    {
      if (iter != nullptr)
      {
        iter->start();
      }
    }
  }

  return ErrCode_Success;
}

ErrCode SensorManager::stop()
{
#if (DEBUG_LEVEL > 1)
  INFO << "Sensor Manager stop" << REND;
#endif
  if (imu_receiver_ != nullptr)
  {
    imu_receiver_->stop();
  }
  if (gnss_receiver_ != nullptr)
  {
    gnss_receiver_->stop();
  }
  if (odom_receiver_ != nullptr)
  {
    odom_receiver_->stop();
  }
  for (auto& iter : lidar_points_receivers_)
  {
    if (iter != nullptr)
      iter->stop();
  }
  for (auto& iter : lidar_packets_receivers_)
  {
    if (iter != nullptr)
    {
      iter->stop();
    }
  }
  return ErrCode_Success;
}

template <typename T>
T* SensorManager::configReceiver(const YAML::Node& sensor_config, const std::string& type, const int& msg_source)
{
  bool success = false;
  std::string receiver_type = "";
  std::string frame_id = "";
  YAML::Node receiver_config;
  if (msg_source == MessageSourceRsDriver)
  {
    receiver_config = yamlSubNodeAbort(sensor_config, "driver");
    yamlReadAbort<std::string>(receiver_config, "device_type", receiver_type);
    yamlReadAbort<std::string>(receiver_config, "frame_id", frame_id);
  }

  else if (msg_source == MessageSourceRos && use_ros_)
  {
#ifdef ROS_FOUND
    receiver_type = type + "_ros";
    frame_id = "/" + type + "_ros";
#else
    ERROR << "ROS not found! Could not use ros-relate runctions! Abort!" << REND;
    exit(-1);
#endif  // ROS_FOUND
  }

  else if (msg_source == MessageSourceProto && use_proto_)
  {
#ifdef PROTO_FOUND
    receiver_type = type + "_proto";
    frame_id = "/" + type + "_proto";
#else
    ERROR << "Proto not found! Could not use proto-relate runctions! Abort!" << REND;
    exit(-1);
#endif  // PROTO_FOUND
  }

  T* receiver = construct<T>(receiver_type, frame_id);

  if (receiver)
  {
    if (!receiver->isInitialized())
    {
      receiver->init(sensor_config);  // TODO: Check if init success
    }
    success = true;
  }
  if (success == false)
  {
    ERROR << "SensorManager : Failed to creat a " << type
          << " message receiver. Aborting!!!Check the use_ros or use_proto!" << REND;
    exit(-1);
  }

  receiver->regExceptionCallback(std::bind(&SensorManager::localExceptionCallback, this, std::placeholders::_1));

  return receiver;
}

template <typename T>
T* SensorManager::configTransmitter(const YAML::Node& sensor_config, const std::string& type, bool send_msg_ros,
                                    bool send_msg_proto)
{
  bool success = false;
  std::string transmitter_type = "";
  std::string frame_id = "";

  if (use_ros_ && send_msg_ros)  // 2: ROS
  {
#ifdef ROS_FOUND
    transmitter_type = type + "_ros";
    frame_id = "/" + type + "_ros";
#else
    ERROR << "ROS not found! Could not use ros-relate runctions! Abort!" << REND;
    exit(-1);
#endif  // ROS_FOUND
  }

  if (use_proto_ && send_msg_proto)
  {
#ifdef PROTO_FOUND
    transmitter_type = type + "_proto";
    frame_id = "/" + type + "_proto";
#else
    ERROR << "Proto not found! Could not use proto-relate runctions! Abort!" << REND;
    exit(-1);
#endif  // PROTO_FOUND
  }
  T* transmitter = construct<T>(transmitter_type, frame_id);
  if (transmitter)
  {
    if (!transmitter->isInitialized())
    {
      transmitter->init(sensor_config);  // TODO: Check if init success
    }
    success = true;
  }

  if (success == false)
  {
    ERROR << "SensorManager : Failder to creat a  " << type
          << " message transmitter! Aborting!!!Check the use_ros or use_proto!" << REND;
    exit(-1);
  }
  transmitter->regExceptionCallback(std::bind(&SensorManager::localExceptionCallback, this, std::placeholders::_1));
  return transmitter;
}

ErrCode SensorManager::initImu(const YAML::Node& config)
{
  YAML::Node imu_config = yamlSubNodeAbort(config, "imu");
  YAML::Node imu_common_config = yamlSubNodeAbort(imu_config, "common");
  bool send_msg_ros;
  bool send_msg_proto;
  int msg_source;
  yamlRead<int>(imu_common_config, "msg_source", msg_source, 0);
  yamlRead<bool>(imu_common_config, "send_msg_ros", send_msg_ros, false);
  yamlRead<bool>(imu_common_config, "send_msg_proto", send_msg_proto, false);
  if (msg_source == 0)
  {
#if (DEBUG_LEVEL > 1)
    DEBUG << "sensor_manager: Not using Imu" << REND;
#endif
    if (send_msg_proto || send_msg_ros)
    {
#if (DEBUG_LEVEL > 0)
      WARNING << "sensor_manager:Not using Imu but try to send Imu msg!" << REND;
#endif
    }
    return ErrCode_Success;
  }
  use_imu_ = true;
  imu_receiver_ = configReceiver<ImuInterface>(imu_config, "imu", msg_source);
  imu_receiver_->regRecvCallback(std::bind(&SensorManager::localImuCallback, this, std::placeholders::_1));
  if (send_msg_ros)
  {
    imu_ros_transmitter_ = configTransmitter<ImuInterface>(imu_config, "imu", send_msg_ros, false);
    imu_receiver_->regRecvCallback(std::bind(&ImuInterface::send, imu_ros_transmitter_, std::placeholders::_1));
  }
  if (send_msg_proto)
  {
    imu_proto_transmitter_ = configTransmitter<ImuInterface>(imu_config, "imu", false, send_msg_proto);
    imu_receiver_->regRecvCallback(std::bind(&ImuInterface::send, imu_proto_transmitter_, std::placeholders::_1));
  }
  return ErrCode_Success;
}

ErrCode SensorManager::initOdom(const YAML::Node& config)
{
  YAML::Node odom_config = yamlSubNodeAbort(config, "odom");
  YAML::Node odom_common_config = yamlSubNodeAbort(odom_config, "common");
  bool send_msg_ros;
  bool send_msg_proto;
  int msg_source;
  yamlRead<int>(odom_common_config, "msg_source", msg_source, 0);
  yamlRead<bool>(odom_common_config, "send_msg_ros", send_msg_ros, false);
  yamlRead<bool>(odom_common_config, "send_msg_proto", send_msg_proto, false);

  if (msg_source == 0)
  {
#if (DEBUG_LEVEL > 1)
    DEBUG << "sensor_manager: Not using Odom" << REND;
#endif
    if (send_msg_proto || send_msg_ros)
    {
#if (DEBUG_LEVEL > 0)
      WARNING << "sensor_manager:Not using Odom but try to send Odom msg!" << REND;
#endif
    }
    return ErrCode_Success;
  }
  use_odom_ = true;
  odom_receiver_ = configReceiver<OdomInterface>(odom_config, "odom", msg_source);
  odom_receiver_->regRecvCallback(std::bind(&SensorManager::localOdomCallback, this, std::placeholders::_1));

  if (send_msg_ros)
  {
    odom_ros_transmitter_ = configTransmitter<OdomInterface>(odom_config, "odom", send_msg_ros, false);
    odom_receiver_->regRecvCallback(std::bind(&OdomInterface::send, odom_ros_transmitter_, std::placeholders::_1));
  }
  if (send_msg_proto)
  {
    odom_proto_transmitter_ = configTransmitter<OdomInterface>(odom_config, "odom", false, send_msg_proto);
    odom_receiver_->regRecvCallback(std::bind(&OdomInterface::send, odom_proto_transmitter_, std::placeholders::_1));
  }

  return ErrCode_Success;
}

ErrCode SensorManager::initGnss(const YAML::Node& config)
{
  YAML::Node gnss_config = yamlSubNodeAbort(config, "gnss");
  YAML::Node gnss_common_config = yamlSubNodeAbort(gnss_config, "common");
  bool send_msg_ros;
  bool send_msg_proto;
  int msg_source;
  yamlRead<int>(gnss_common_config, "msg_source", msg_source, 0);
  yamlRead<bool>(gnss_common_config, "send_msg_ros", send_msg_ros, false);
  yamlRead<bool>(gnss_common_config, "send_msg_proto", send_msg_proto, false);
  if (msg_source == 0)
  {
#if (DEBUG_LEVEL > 1)
    DEBUG << "sensor_manager: Not using Gnss" << REND;
#endif
    if (send_msg_proto || send_msg_ros)
    {
#if (DEBUG_LEVEL > 0)
      WARNING << "sensor_manager:Not using Gnss but try to send Gnss msg!" << REND;
#endif
    }
    return ErrCode_Success;
  }
  use_gnss_ = true;
  gnss_receiver_ = configReceiver<GnssInterface>(gnss_config, "gnss", msg_source);
  gnss_receiver_->regRecvCallback(std::bind(&SensorManager::localGnssCallback, this, std::placeholders::_1));
  if (send_msg_ros)
  {
    gnss_ros_transmitter_ = configTransmitter<GnssInterface>(gnss_config, "gnss", send_msg_ros, false);
    gnss_receiver_->regRecvCallback(std::bind(&GnssInterface::send, gnss_ros_transmitter_, std::placeholders::_1));
  }
  if (send_msg_proto)
  {
    gnss_proto_transmitter_ = configTransmitter<GnssInterface>(gnss_config, "gnss", false, send_msg_proto);
    gnss_receiver_->regRecvCallback(std::bind(&GnssInterface::send, gnss_proto_transmitter_, std::placeholders::_1));
  }

  return ErrCode_Success;
}

ErrCode SensorManager::initLidar(const YAML::Node& config)
{
  YAML::Node lidars_config = yamlSubNodeAbort(config, "lidar");
  int msg_source = 0;
  bool send_points_ros = false;
  bool send_packets_ros = false;
  bool send_points_proto = false;
  bool send_packets_proto = false;

  YAML::Node lidar_common_config = yamlSubNodeAbort(lidars_config, "common");
  yamlRead<int>(lidar_common_config, "msg_source", msg_source, 0);
  if (use_ros_)
  {
    yamlRead<bool>(lidar_common_config, "send_packets_ros", send_packets_ros, false);
    yamlRead<bool>(lidar_common_config, "send_points_ros", send_points_ros, false);
  }
  if (use_proto_)
  {
    yamlRead<bool>(lidar_common_config, "send_points_proto", send_points_proto, false);
    yamlRead<bool>(lidar_common_config, "send_packets_proto", send_packets_proto, false);
  }
  YAML::Node lidar_basic_config = yamlSubNodeAbort(lidars_config, "lidar");
  for (uint8_t i = 0; i < lidar_basic_config.size(); ++i)
  {
    if (msg_source == 0)
    {
      return ErrCode_Success;
    }
    use_lidar_ = true;
    /*Receiver*/
    switch (msg_source)
    {
      case 1:  // use driver
        lidarpoints_run_flag_ = true;
        lidarpkts_run_flag_ = false;
        lidar_basic_config[i]["msg_source"] = 1;
        lidar_points_receivers_.emplace_back(
            configReceiver<LidarPointsInterface>(lidar_basic_config[i], "lidar_points_" + std::to_string(i), 1));
        lidar_points_receivers_[i]->setPath(config_path_);
        if (send_packets_ros || send_packets_proto)
          lidar_packets_receivers_.emplace_back(
              configReceiver<LidarPacketsInterface>(lidar_basic_config[i], "lidar_pkts_" + std::to_string(i), 1));
        else
          lidar_packets_receivers_.emplace_back(nullptr);
        break;

      case 2:  // pkt from ros
        lidarpoints_run_flag_ = false;
        lidarpkts_run_flag_ = true;
        lidar_basic_config[i]["msg_source"] = 2;
        lidar_points_receivers_.emplace_back(
            configReceiver<LidarPointsInterface>(lidar_basic_config[i], "lidar_points_" + std::to_string(i), 1));
        lidar_packets_receivers_.emplace_back(
            configReceiver<LidarPacketsInterface>(lidar_basic_config[i], "lidar_pkts_" + std::to_string(i), 2));
        lidar_points_receivers_[i]->setPath(config_path_);
        lidar_packets_receivers_[i]->regRecvCallback(
            std::bind(&LidarPointsInterface::processMsopPackets, lidar_points_receivers_[i], std::placeholders::_1));
        lidar_packets_receivers_[i]->regRecvCallback(
            std::bind(&LidarPointsInterface::processDifopPackets, lidar_points_receivers_[i], std::placeholders::_1));
        break;

      case 3:  // points from ros
        lidarpoints_run_flag_ = true;
        lidarpkts_run_flag_ = false;
        lidar_basic_config[i]["msg_source"] = 3;
        lidar_points_receivers_.emplace_back(
            configReceiver<LidarPointsInterface>(lidar_basic_config[i], "lidar_points_" + std::to_string(i), 2));
        if (send_packets_ros || send_packets_proto)
        {
          WARNING << "sensor_manager: Impossible to send packets because of the message source" << REND;
        }
        lidar_packets_receivers_.emplace_back(nullptr);
        break;

      case 4:  // points from proto
        lidarpoints_run_flag_ = false;
        lidarpkts_run_flag_ = true;
        lidar_basic_config[i]["msg_source"] = 4;
        lidar_points_receivers_.emplace_back(
            configReceiver<LidarPointsInterface>(lidar_basic_config[i], "lidar_points_" + std::to_string(i), 1));
        lidar_packets_receivers_.emplace_back(
            configReceiver<LidarPacketsInterface>(lidar_basic_config[i], "lidar_pkts_" + std::to_string(i), 3));
        lidar_points_receivers_[i]->setPath(config_path_);
        lidar_packets_receivers_[i]->regRecvCallback(
            std::bind(&LidarPointsInterface::processMsopPackets, lidar_points_receivers_[i], std::placeholders::_1));
        lidar_packets_receivers_[i]->regRecvCallback(
            std::bind(&LidarPointsInterface::processDifopPackets, lidar_points_receivers_[i], std::placeholders::_1));
        break;

      case 5:  // packets from proto
        lidarpoints_run_flag_ = true;
        lidarpkts_run_flag_ = false;
        lidar_basic_config[i]["msg_source"] = 5;
        lidar_points_receivers_.emplace_back(
            configReceiver<LidarPointsInterface>(lidar_basic_config[i], "lidar_points_" + std::to_string(i), 3));
        if (send_packets_ros || send_packets_proto)
        {
          WARNING << "sensor_manager: Impossible to send packets because of the message source" << REND;
        }
        lidar_packets_receivers_.emplace_back(nullptr);
        break;

      default:
        ERROR << "Wrong LiDAR message source! Abort!" << REND;
        exit(-1);
    }
    lidar_points_receivers_[i]->regRecvCallback(
        std::bind(&SensorManager::localLidarPointsCallback, this, std::placeholders::_1));

    /*Transmitter*/
    if (send_packets_ros)
    {
      lidar_basic_config[i]["send_packets_ros"] = true;
      lidar_packets_ros_transmitters_.emplace_back(configTransmitter<LidarPacketsInterface>(
          lidar_basic_config[i], "lidar_pkts_" + std::to_string(i), send_packets_ros, false));
      lidar_packets_receivers_[i]->regRecvCallback(
          std::bind(&LidarPacketsInterface::send_msop, lidar_packets_ros_transmitters_[i], std::placeholders::_1));
      lidar_packets_receivers_[i]->regRecvCallback(
          std::bind(&LidarPacketsInterface::send_difop, lidar_packets_ros_transmitters_[i], std::placeholders::_1));
    }
    if (send_packets_proto)
    {
      lidar_basic_config[i]["send_packets_proto"] = true;
      lidar_packets_proto_transmitters_.emplace_back(configTransmitter<LidarPacketsInterface>(
          lidar_basic_config[i], "lidar_pkts_" + std::to_string(i), false, send_packets_proto));
      lidar_packets_receivers_[i]->regRecvCallback(
          std::bind(&LidarPacketsInterface::send_msop, lidar_packets_proto_transmitters_[i], std::placeholders::_1));
      lidar_packets_receivers_[i]->regRecvCallback(
          std::bind(&LidarPacketsInterface::send_difop, lidar_packets_proto_transmitters_[i], std::placeholders::_1));
    }
    if (send_points_ros)
    {
      lidar_basic_config[i]["send_points_ros"] = true;
      lidar_points_ros_transmitters_.emplace_back(configTransmitter<LidarPointsInterface>(
          lidar_basic_config[i], "lidar_points_" + std::to_string(i), send_points_ros, false));
      lidar_points_receivers_[i]->regRecvCallback(
          std::bind(&LidarPointsInterface::send, lidar_points_ros_transmitters_[i], std::placeholders::_1));
    }
    if (send_points_proto)
    {
      lidar_basic_config[i]["send_points_proto"] = true;
      lidar_points_proto_transmitters_.emplace_back(configTransmitter<LidarPointsInterface>(
          lidar_basic_config[i], "lidar_points_" + std::to_string(i), false, send_points_proto));
      lidar_points_receivers_[i]->regRecvCallback(
          std::bind(&LidarPointsInterface::send, lidar_points_proto_transmitters_[i], std::placeholders::_1));
    }
  }
  return ErrCode_Success;
}  // namespace sensor

template <class R>
R* SensorManager::construct(const std::string& device_type, const std::string& frame_id)
{
  uint16_t api_request = R::getApi();
  std::map<std::string, common::CommonBase*>::iterator sensor_itr;
  auto device_itr = sensors_.find(device_type);

  if (device_itr != sensors_.end())
  {
    sensor_itr = device_itr->second.find(frame_id);
    if (sensor_itr != device_itr->second.end())
      return dynamic_cast<R*>(sensor_itr->second);
  }
  R* ret;
  if (device_type == "BT708")
  {
    ret = localConstruct<R, GnssBT708>(api_request);
  }
  else if (device_type == "POSLVX")
  {
    ret = localConstruct<R, GnssPOSLVX>(api_request);
  }
  else if (device_type == "TL740D")
  {
    ret = localConstruct<R, ImuTL740D>(api_request);
  }
  else if (device_type == "HWT605")
  {
    ret = localConstruct<R, ImuHWT605>(api_request);
  }
  else if (device_type == "DY551")
  {
    ret = localConstruct<R, ImuDY551>(api_request);
  }
  else if (device_type == "XWG13668")
  {
    ret = localConstruct<R, InsXWG13668>(api_request);
  }
#ifdef X86_64
  else if (device_type == "BYD")
  {
    ret = localConstruct<R, OdomBYD>(api_request);
  }
  else if (device_type == "JILI")
  {
    ret = localConstruct<R, OdomJILI>(api_request);
  }
  else if (device_type == "BIEKE")
  {
    ret = localConstruct<R, OdomBIEKE>(api_request);
  }
#endif
  else if (device_type == "RS16" || device_type == "RS32" || device_type == "RSBP")
  {
    ret = localConstruct<R, LidarBase>(api_request);
  }
#ifdef ROS_FOUND
  else if (device_type == "imu_ros")
  {
    ret = localConstruct<R, ImuRosAdapter>(api_request);
  }
  else if (device_type == "gnss_ros")
  {
    ret = localConstruct<R, GnssRosAdapter>(api_request);
  }
  else if (device_type == "odom_ros")
  {
    ret = localConstruct<R, OdomRosAdapter>(api_request);
  }
  else if ((int)device_type.find("lidar_points") != -1 && (int)device_type.find("ros") != -1)
  {
    ret = localConstruct<R, LidarPointsRosAdapter>(api_request);
  }
  else if ((int)device_type.find("lidar_pkts") != -1 && (int)device_type.find("ros") != -1)
  {
    ret = localConstruct<R, LidarPacketsRosAdapter>(api_request);
  }
#endif
#ifdef PROTO_FOUND
  else if (device_type == "imu_proto")
  {
    ret = localConstruct<R, ImuProtoAdapter>(api_request);
  }
  else if (device_type == "gnss_proto")
  {
    ret = localConstruct<R, GnssProtoAdapter>(api_request);
  }
  else if (device_type == "odom_proto")
  {
    ret = localConstruct<R, OdomProtoAdapter>(api_request);
  }
  else if ((int)device_type.find("lidar_points") != -1 && (int)device_type.find("proto") != -1)
  {
    ret = localConstruct<R, LidarPointsProtoAdapter>(api_request);
  }
  else if ((int)device_type.find("lidar_pkts") != -1 && (int)device_type.find("proto") != -1)
  {
    ret = localConstruct<R, LidarPacketsProtoAdapter>(api_request);
  }
#endif
  else
  {
    ERROR << "Device type: " << device_type << " not found! Please check the device type !" << REND;
    exit(-1);
  }
  if (ret != nullptr)
  {
    sensors_[device_type].emplace(frame_id, ret);
  }

  return ret;
}

}  // namespace sensor
}  // namespace robosense
