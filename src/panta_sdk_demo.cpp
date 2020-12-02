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

#include "panta_sdk_demo/panta_sdk_demo.h"

namespace robosense
{
/**
 * @brief Deconstructor
 * @detail Delete the pointer map of proto base class
 */
SdkDemo::~SdkDemo()
{
#ifdef PROTO_FOUND
  for (auto &proto_itr : proto_map_) ///< only excute if protobuf found
  {
    delete proto_itr.second;
  }
#endif
}
/**
 * @brief  Initialize function
 * @detail Initialize the sensormanager module, preoprocessing module, 
 *         localization algorithm, perception algorithm, ROS publihser, Proto publihser
 * @param  config--the input YAML node
 * @param  use_ros--if true, ROS related functions will be activated
 *         (whether to send or receive massages through ROS need to be set separatedly)
 * @param  use_proto--if true, PROTO related functions will be activated
 *         (whether to send or receiver massgaes through Proto need to be set separatedly)
 * @param  config_path--the path of the config file
 *         (may be relative path or absolute path)
 */
void SdkDemo::init(const YAML::Node &config, bool use_ros, bool use_proto, std::string config_path)
{
  /*****Common Initialization****/
  setName("SdkDemo");
  run_flag_ = false;
  config_path_ = config_path;
  use_ros_ = use_ros;
  use_proto_ = use_proto;
  errorprinter_ptr_ = std::unique_ptr<common::ErrorPrinter>(new common::ErrorPrinter());

  /*****Sensor Manger Initialization****/
  sensorManager_ptr_ = std::unique_ptr<sensor::SensorManager>(new sensor::SensorManager());
  YAML::Node sensor_config = yamlSubNodeAbort(config, "sensor");
  initSensorManager(sensor_config);

  /*****PreProcessing Initialization****/
  preProcess_ptr_ = std::unique_ptr<preprocessing::PreProcessing>(new preprocessing::PreProcessing());
  YAML::Node preprocessing_config = yamlSubNodeAbort(config, "preprocessing");
  initPreProcessing(preprocessing_config);

  /*****Localization Initialization****/
#ifdef COMPILE_LOCALIZATION
  pose_guess_.fill(0);
  YAML::Node localization_config = yamlSubNodeAbort(config, "localization");
  bool tmp_run_localization;
  yamlRead<bool>(config["general"], "run_localization", tmp_run_localization, false);
  run_localization_ = tmp_run_localization;
  if (run_localization_)
  {
    localization_ptr_.reset(new localization::RSLocalization);
    initLocalization(localization_config);
  }
#endif

  /*****Perception Initialization****/
#ifdef COMPILE_PERCEPTION
  YAML::Node perception_config = yamlSubNodeAbort(config, "perception");
  yamlRead<bool>(config["general"], "run_perception", run_perception_, false);
  if (run_perception_)
  {
    perception_ptr_.reset(new perception::RSPerception<pcl::PointXYZI>);
    initPerception(perception_config);
  }
  if (perception_mode_ == WITHODOMETRY)
  {
    run_odometry_ = true;
    YAML::Node odometry_config = yamlSubNodeAbort(config, "odometry");
    odometry_ptr_.reset(new odometry::RSOdometry<pcl::PointXYZI>);
    initOdometry(odometry_config);
  }

#endif

  /*****ROS Initialization****/
  YAML::Node result_sender_config = yamlSubNodeAbort(config, "result_sender");
  if (use_ros_)
  {
#ifdef ROS_FOUND
    initRosPublisher(result_sender_config);
#else
    ERROR << "ROS not found but try to use_ros! Abort!" << REND;
    exit(-1);
#endif
  }

  /*****Proto Initialization****/
  if (use_proto_)
  {
#ifdef PROTO_FOUND
    initProtoSender(result_sender_config);
#else
    ERROR << "Proto not found but try to use_proto! Abort!" << REND;
    exit(-1);
#endif
  }
}

/**
 * @brief the start function
 * @detail 1,call start functions for sensor manger module & preprocessing module 
 *         2,if run_localization_ is true, strat localization module
 *         3,if run_perception_ is true, start  perception module
 */
void SdkDemo::start(void)
{
  run_flag_ = true;
  std::thread ros_thread_;
  if (use_ros_)
  {
#ifdef ROS_FOUND
    const auto &func = [this] {
      ros::MultiThreadedSpinner spinner(4);
      spinner.spin(); ///< if use ROS and ROS found, the main thread will stay in ROS::spin
    };
    ros_thread_ = std::thread(func);
#else
    ERROR << "ROS not found but try to use_ros! Abort!" << REND;
    exit(-1);
#endif
  }
  sensorManager_ptr_->start();
  preProcess_ptr_->start();
  const auto &func1 = [this] {
    sleep(10);
    error_set_.clear();
    while (run_flag_)
    {
      sleep(1);
      if (!error_set_.empty())
        WARNING << "****************************************************************" << REND;
      for (auto iter : error_set_)
      {
        errorprinter_ptr_->printErrorCode(iter);
      }
      if (!error_set_.empty())
        WARNING << "****************************************************************" << REND;
      error_set_.clear();
    }
  };
  error_handle_thread_.reset(new std::thread(func1));

#ifdef COMPILE_LOCALIZATION
  if (run_localization_)
  {
    Eigen::Matrix<double, 6, 1> pose_guess;
    pose_guess<<pose_guess_[0],pose_guess_[1],pose_guess_[2],0,0,0;
    localization_ptr_->start(localization_mode_,pose_guess);
    localization_thread_ = std::thread(std::bind(&SdkDemo::localizationThreadFunc, this));
  }
#endif
#ifdef COMPILE_PERCEPTION
  if (run_perception_)
  {
    perception_ptr_->start();
  }
  if (run_odometry_)
  {
    odometry_ptr_->start();
  }
#endif
  if (use_ros_)
  {
#ifdef ROS_FOUND
    ros_thread_.join();
    stop();
#endif
  }
  else
  {
    while (run_flag_) ///< if not use ROS, the main thread will stay in a while loop
    {
      sleep(1);
    }
  }
}

/**
 * @brief the stop function
 * @detail 1, stop the thread to get localization result
 *         2, call stop functions for each module, reset each pointer
 *         3, set run_flag_ to false, the whole progress stop
 */
bool SdkDemo::stop(void)
{
  if (run_flag_ == true)
  {
    run_flag_ = false;
    error_handle_thread_->join();
  }
  sensorManager_ptr_.reset();
  preProcess_ptr_.reset();
#ifdef COMPILE_LOCALIZATION
  if (localization_thread_.joinable())
  {
    run_localization_ = false;
    localization_thread_.join();
    localization_ptr_->stop();
  }
  localization_ptr_.reset();
#endif
#ifdef COMPILE_PERCEPTION
  if (run_perception_)
  {
    perception_ptr_->stop();
  }
  perception_ptr_.reset();
  if (run_odometry_)
  {
    run_odometry_ = false;
    odometry_ptr_->stop();
  }
  odometry_ptr_.reset();
#endif
  if (compensate_cloud_send_thread_flag_ == true)
  {
    compensate_cloud_send_thread_flag_ = false;
    compensate_cloud_send_cv_.notify_all();
    compensate_cloud_send_thread_->join();
  }
  if (uncompensate_cloud_send_thread_flag_ == true)
  {
    uncompensate_cloud_send_thread_flag_ = false;
    uncompensate_cloud_send_cv_.notify_all();
    uncompensate_cloud_send_thread_->join();
  }

  return true;
}

/**
 * @brief  sensor manager initialization function
 * @detail 1, init sensor manager
 *         2, register local excption callback on sensor manager
 */
void SdkDemo::initSensorManager(const YAML::Node &config)
{
  YAML::Node sensor_config = std::move(config);
  sensorManager_ptr_->init(sensor_config, use_ros_, use_proto_, config_path_);
  sensorManager_ptr_->regExceptionCallback(std::bind(&SdkDemo::localExceptionCallback, this, std::placeholders::_1));
}

/**
 * @brief  preprocessing initialization function
 * @detail 1, get the frame_id of fusion points and store it in fusion_frame_id
 *         2, get the sensor flags from sensormanager to check the status of each sensor
 *            e.g. if not use imu, the preprocessing will not start the imu thread
 *         3, init preprocessing module
 *         4, register local exception call back on preprocessing module
 *         5, register the callback functions of preprocessing module on sensor manager 
 *    
 *     (**** The pointer of sensor manager must be initialized before this function called!****)
 */
void SdkDemo::initPreProcessing(const YAML::Node &config)
{
  YAML::Node preprocessing_config = std::move(config);
  yamlReadAbort<std::string>(preprocessing_config["lidar"], "fusion_frame_id", fusion_frame_id_);
  preprocessing_config["imu"]["use_imu"] = sensorManager_ptr_->getUseImu();
  preprocessing_config["gnss"]["use_gnss"] = sensorManager_ptr_->getUseGnss();
  preprocessing_config["odom"]["use_odom"] = sensorManager_ptr_->getUseOdom();
  preprocessing_config["lidar"]["use_lidar"] = sensorManager_ptr_->getUseLidar();
  preProcess_ptr_->init(preprocessing_config);
  lidar_base_pose_ = preProcess_ptr_->getSensorTransform("lidar");
  vehicle_frame_id_ = preProcess_ptr_->getBaselinkFrameid();
  preProcess_ptr_->regExceptionCallback(std::bind(&SdkDemo::localExceptionCallback, this, std::placeholders::_1));
  sensorManager_ptr_->regRecvCallback(std::bind(&preprocessing::PreProcessing::imuCallback, preProcess_ptr_.get(), std::placeholders::_1));
  sensorManager_ptr_->regRecvCallback(std::bind(&preprocessing::PreProcessing::gnssCallback, preProcess_ptr_.get(), std::placeholders::_1));
  sensorManager_ptr_->regRecvCallback(std::bind(&preprocessing::PreProcessing::odomCallback, preProcess_ptr_.get(), std::placeholders::_1));
  sensorManager_ptr_->regRecvCallback(std::bind(&preprocessing::PreProcessing::lidarPointsCallback, preProcess_ptr_.get(), std::placeholders::_1));
}

/**
 * @brief  localization module initialization function, only be called if COMPILE_LOCALIZATION is set
 * @detail 1, get the localization mode and frequency from the yaml file
 *         2, register the local exception call back on localization module
 *         3, register the call back functions of localization module on preprocessing module
 *         4, depend on the mode, register the target lidar points call back function on preprocessing module
 */
#ifdef COMPILE_LOCALIZATION
void SdkDemo::initLocalization(const YAML::Node &config)
{
  YAML::Node localization_config = std::move(config);
  bool localization_with_motion_compensation;
  localization_config["common"]["vehicle_frame_id"] = vehicle_frame_id_;
  yamlRead<bool>(localization_config["common"], "localization_with_motion_compensation", localization_with_motion_compensation, true);
  yamlRead<int>(localization_config["common"], "localization_mode", localization_mode_, 0);
  yamlRead<double>(localization_config["common"], "pose_guess_0", pose_guess_[0], 0);
  yamlRead<double>(localization_config["common"], "pose_guess_1", pose_guess_[1], 0);
  yamlRead<double>(localization_config["common"], "pose_guess_2", pose_guess_[2], 0);
  yamlRead<std::string>(localization_config["common"], "map_frame_id", map_frame_id_, "map");
  yamlRead<int>(localization_config["common"], "send_localization_freq", localization_freq_, 30);
  localization_ptr_->regExceptionCallback(std::bind(&SdkDemo::localExceptionCallback, this, std::placeholders::_1));
  localization_ptr_->init(config_path_, localization_config);
  preProcess_ptr_->regRecvCallback(std::bind(&localization::RSLocalization::imuCallback, localization_ptr_.get(), std::placeholders::_1));
  preProcess_ptr_->regRecvCallback(std::bind(&localization::RSLocalization::gnssCallback, localization_ptr_.get(), std::placeholders::_1));
  preProcess_ptr_->regRecvCallback(std::bind(&localization::RSLocalization::odomCallback, localization_ptr_.get(), std::placeholders::_1));
  if (localization_with_motion_compensation)
    preProcess_ptr_->regRecvCallback(std::bind(&localization::RSLocalization::lidarCallback, localization_ptr_.get(), std::placeholders::_1), true, fusion_frame_id_);
  else
    preProcess_ptr_->regRecvCallback(std::bind(&localization::RSLocalization::lidarCallback, localization_ptr_.get(), std::placeholders::_1), false, fusion_frame_id_);
}
#endif

/**
 * @brief  perception module initialization function, only be called if COMPILE_PERCEPTION is set
 * @detail 1, get the perception mode from the yaml file
 *         2, register the local exception call back on perception module
 *         3, register lidar points call back function on preprocessing module (without motion compensation)
 *        
 */
#ifdef COMPILE_PERCEPTION
void SdkDemo::initPerception(const YAML::Node &config)
{
  YAML::Node perception_config = std::move(config);
  yamlRead<int>(perception_config["common"], "perception_mode", perception_mode_, 0);
#ifdef COMPILE_LOCALIZATION
  if (!run_localization_ && perception_mode_ == WITHLOCALIZATION)
  {
   // WARNING << " Localization algorithm not run, the perception mode auto changed to 0" << REND;
    perception_mode_ = ONLYLIDAR;
    perception_config["common"]["perception_mode"] = 0;
  }
#endif
  perception_ptr_->init(perception_config, lidar_base_pose_, config_path_);
  perception_ptr_->regExceptionCallback(std::bind(&SdkDemo::localExceptionCallback, this, std::placeholders::_1));
  if (perception_mode_ == ONLYLIDAR || perception_mode_ == V2R)
  {
    preProcess_ptr_->regRecvCallback(std::bind(&perception::RSPerception<pcl::PointXYZI>::lidarCallback, perception_ptr_.get(), std::placeholders::_1), false, fusion_frame_id_);
  }
#ifdef COMPILE_LOCALIZATION
  if (perception_mode_ == WITHLOCALIZATION)
  {
    auto func = [this](const common::LidarPointsMsg &msg) {
      common::VehicleStateMsg cur_state;
      if (localization_ptr_->getModuleStatus().localization_status == localization::loc_status::LOC_NORMAL)
      {
        if (localization_ptr_->getVehicleState(cur_state, msg.timestamp) == common::ErrCode_Success)
        {
          perception_ptr_->lidarCallback(msg);
          perception_ptr_->vehiclestateCallback(cur_state); ///< if the perception mode is WITHLOCALIZATION, the localization result will be sent to perception module
        }
      }
    };

    preProcess_ptr_->regRecvCallback(func, false, fusion_frame_id_);
  }
#endif
  if (perception_mode_ == WITHODOMETRY)
  {
    auto func = [this](const common::LidarPointsMsg &msg) {
      common::VehicleStateMsg cur_state;
      if (odometry_ptr_->getVehicleState(cur_state) == common::ErrCode_Success)
      {
        perception_ptr_->lidarCallback(msg);
        perception_ptr_->vehiclestateCallback(cur_state);
      }
    };
    preProcess_ptr_->regRecvCallback(func, false, fusion_frame_id_);
  }
}
void SdkDemo::initOdometry(const YAML::Node &config)
{
  YAML::Node odometry_config = config;
  odometry_ptr_->regExceptionCallback(std::bind(&SdkDemo::localExceptionCallback, this, std::placeholders::_1));
  odometry_ptr_->init(odometry_config);
  preProcess_ptr_->regRecvCallback(std::bind(&odometry::RSOdometry<pcl::PointXYZI>::imuCallback, odometry_ptr_.get(), std::placeholders::_1));
  preProcess_ptr_->regRecvCallback(std::bind(&odometry::RSOdometry<pcl::PointXYZI>::odomCallback, odometry_ptr_.get(), std::placeholders::_1));
  preProcess_ptr_->regRecvCallback(std::bind(&odometry::RSOdometry<pcl::PointXYZI>::lidarCallback, odometry_ptr_.get(), std::placeholders::_1),
                                   false, fusion_frame_id_);
}
#endif

/**
 * @brief  the ROS initialize function (Only be complied if ROS is found)
 * @detail 1, Initialize the ROS node handle
 *         2, Initialize each topics with the topic name in yaml file
 *         3, Initialize publishers depends on the parameters
 */
#ifdef ROS_FOUND
void SdkDemo::initRosPublisher(const YAML::Node &result_sender_config)
{
  YAML::Node ros_publisher_config = yamlSubNodeAbort(result_sender_config, "ros");
  nh_ = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle());

  /* Fusion pointcloud publisher*/
  YAML::Node pointcloud_publisher_config = yamlSubNodeAbort(ros_publisher_config, "pointcloud");
  bool send_compensated_cloud_ros;
  bool send_uncompensated_cloud_ros;
  yamlRead<bool>(pointcloud_publisher_config, "send_compensated_cloud_ros", send_compensated_cloud_ros, false);
  yamlRead<bool>(pointcloud_publisher_config, "send_uncompensated_cloud_ros", send_uncompensated_cloud_ros, false);

  if (send_compensated_cloud_ros) ///< if send_compensated_could_ros is set to true, the fusion point cloud with motion correct will be published on ROS
  {
    std::string compensated_cloud_ros_topic;
    yamlReadAbort<std::string>(pointcloud_publisher_config, "compensated_cloud_ros_topic", compensated_cloud_ros_topic);
    compensated_points_pub_ = nh_->advertise<sensor_msgs::PointCloud2>(compensated_cloud_ros_topic, 10, false);
    auto func = [this](const common::LidarPointsMsg &msg) {
      compensated_points_pub_.publish(toRosMsg(msg));
    };
    preProcess_ptr_->regRecvCallback(func, true, fusion_frame_id_);
  }
  if (send_uncompensated_cloud_ros) ///< if send_uncompensated_could_ros is set to true, the fusion point cloud without motion correct will be published on ROS
  {
    std::string uncompensated_cloud_ros_topic;
    yamlReadAbort<std::string>(pointcloud_publisher_config, "uncompensated_cloud_ros_topic", uncompensated_cloud_ros_topic);
    uncompensated_points_pub_ = nh_->advertise<sensor_msgs::PointCloud2>(uncompensated_cloud_ros_topic, 10, false);
    auto func = [this](const common::LidarPointsMsg &msg) {
      local_points_ = std::move(msg);
      uncompensated_points_pub_.publish(toRosMsg(msg));
    };
    preProcess_ptr_->regRecvCallback(func, false, fusion_frame_id_);
  }

  /* Sensor publisher*/
  YAML::Node sensor_publisher_config = yamlSubNode(ros_publisher_config, "sensor");
  bool send_processed_imu_ros;
  yamlRead<bool>(sensor_publisher_config, "send_processed_imu_ros", send_processed_imu_ros, false);
  if (send_processed_imu_ros)
  {
    std::string processed_imu_ros_topic;
    yamlReadAbort<std::string>(sensor_publisher_config, "processed_imu_ros_topic", processed_imu_ros_topic);
    processed_imu_pub_ = nh_->advertise<sensor_msgs::Imu>(processed_imu_ros_topic, 10, false);
    auto func = [this](const common::ImuMsg &msg) {
      processed_imu_pub_.publish(toRosMsg(msg));
    };

    preProcess_ptr_->regRecvCallback(func);
  }

/* Localization result publisher */
#ifdef COMPILE_LOCALIZATION
  if (run_localization_)
  {
    YAML::Node localization_publisher_config = yamlSubNodeAbort(ros_publisher_config, "localization");
    yamlRead<bool>(localization_publisher_config, "send_pos_ros", send_pos_ros_, false);
    yamlRead<bool>(localization_publisher_config, "send_map_ros", send_map_ros_, false);
    yamlRead<bool>(localization_publisher_config, "send_path_ros", send_path_ros_, false);
    if (send_pos_ros_) ///< if send_pos_ros_ is set to true, the localization result publisher will be initialized
    {
      std::string send_pos_ros_topic;
      yamlReadAbort<std::string>(localization_publisher_config, "send_pos_ros_topic", send_pos_ros_topic);
      localization_pos_pub_ = nh_->advertise<nav_msgs::Odometry>(send_pos_ros_topic, 10);
    }
    if (send_map_ros_) ///< if send_map_ros_ is set to true, the localization map publisher will be initialized
    {
      std::string send_map_ros_topic;
      yamlReadAbort<std::string>(localization_publisher_config, "send_map_ros_topic", send_map_ros_topic);
      localization_grid_map_pub_ = nh_->advertise<nav_msgs::OccupancyGrid>(send_map_ros_topic + "_grid", 10, true);
      localization_pointcloud_map_pub_ = nh_->advertise<sensor_msgs::PointCloud2>(send_map_ros_topic + "_pointcloud", 10, true);
    }
    if (send_path_ros_) ///< if send_path_ros_ is set to true, the localization path publisher will be initialized
    {
      std::string send_path_ros_topic;
      yamlReadAbort<std::string>(localization_publisher_config, "send_path_ros_topic", send_path_ros_topic);
      localization_path_pub_ = nh_->advertise<nav_msgs::Path>(send_path_ros_topic, 10);
    }
  }
#endif

/* Perception result publisher */
#ifdef COMPILE_PERCEPTION
  if (run_perception_)
  {
    YAML::Node perception_publisher_config = yamlSubNodeAbort(ros_publisher_config, "perception");
    bool send_obstacle_ros;
    bool send_freespace_ros;
    bool send_groundpoint_ros;
    int freespace_range;
    yamlRead<bool>(perception_publisher_config, "send_obstacle_ros", send_obstacle_ros, false);
    yamlRead<bool>(perception_publisher_config, "send_groundpoint_ros", send_groundpoint_ros, false);
    yamlRead<bool>(perception_publisher_config, "send_freespace_ros", send_freespace_ros, false);
    yamlRead<int>(perception_publisher_config, "send_freespace_ros_range", freespace_range, 60);
    rviz_drawer_.reset(new perception::DrawRviz);
    if (send_obstacle_ros)
    {
      std::string send_obstacle_ros_topic;
      yamlRead<std::string>(perception_publisher_config, "send_obstacle_ros_topic", send_obstacle_ros_topic, "rs_obstacle");
      perception_obstacle_pub_ = nh_->advertise<rs_perception::obstacle_set_ros_msg>(send_obstacle_ros_topic, 10);
      perception_pointcloud_pub_ = nh_->advertise<sensor_msgs::PointCloud2>(send_obstacle_ros_topic + "_points", 10);
      perception_rviz_pub_ = nh_->advertise<visualization_msgs::MarkerArray>(send_obstacle_ros_topic + "_rviz", 10);
      auto func = [this](const common::ObstacleMsg::Ptr &msg_ptr) {
        std_msgs::Header header;
        if (perception_mode_ == ONLYLIDAR || perception_mode_ == WITHODOMETRY || perception_mode_ == V2R)
        {
          header.frame_id = vehicle_frame_id_;
        }
        if (perception_mode_ == WITHLOCALIZATION)
        {
          header.frame_id = map_frame_id_;
        }
        header.seq = msg_ptr->seq;
        header.stamp = header.stamp.fromSec(msg_ptr->timestamp);
        perception_obstacle_pub_.publish(toRosMsg(msg_ptr));
        perception_pointcloud_pub_.publish(toRosMsg(local_points_));
        rviz_drawer_->show_percept(perception_rviz_pub_, header, msg_ptr->obstacles);
      };
      perception_ptr_->regObstacleCallback(func);
    }
    if (send_freespace_ros)
    {
      std::string send_freespace_ros_topic;
      yamlRead<std::string>(perception_publisher_config, "send_freespace_ros_topic", send_freespace_ros_topic, "rs_freespace");
      perception_freespace_pub_ = nh_->advertise<rs_perception::freespace_set_ros_msg>(send_freespace_ros_topic, 10);
      perception_rviz_pub_freespace_ = nh_->advertise<sensor_msgs::PointCloud2>(send_freespace_ros_topic + "_rviz", 10);
      auto func = [this, freespace_range](const common::FreeSpaceMsg::Ptr &msg_ptr) {
        std_msgs::Header header;
        header.frame_id = vehicle_frame_id_;
        header.seq = msg_ptr->seq;
        header.stamp = header.stamp.fromSec(msg_ptr->timestamp);
        perception_freespace_pub_.publish(toRosMsg(msg_ptr));
        rviz_drawer_->show_freespace(perception_rviz_pub_freespace_, header, msg_ptr->freespaces, freespace_range);
      };
      perception_ptr_->regFreeSpaceCallback(func);
    }
    if (send_groundpoint_ros)
    {
      std::string send_groundpoint_ros_topic;
      std::string ground_point_frame_id;
      yamlRead<std::string>(perception_publisher_config, "send_groundpoint_ros_topic", send_groundpoint_ros_topic, "rs_groundpoint");
      ground_points_pub_ = nh_->advertise<sensor_msgs::PointCloud2>(send_groundpoint_ros_topic, 10);
      if (perception_mode_ == ONLYLIDAR || perception_mode_ == WITHODOMETRY || perception_mode_ == V2R)
      {
        ground_point_frame_id = vehicle_frame_id_;
      }
      if (perception_mode_ == WITHLOCALIZATION)
      {
        ground_point_frame_id = map_frame_id_;
      }

      auto func = [this](const common::LidarPointsMsg &msg) {
        ground_points_pub_.publish(toRosMsg(msg));
      };
      perception_ptr_->regGroundPointsCallback(func, ground_point_frame_id);
    }
  }
#endif
}
#endif

/**
 * @brief  the Proto initialize function (Only be complied if Protobuf is found)
 * @detail 1, Initialize the socket sender
 */
#ifdef PROTO_FOUND
void SdkDemo::initProtoSender(const YAML::Node &result_sender_config)
{
  YAML::Node proto_sender_config = yamlSubNodeAbort(result_sender_config, "proto");
  /* Fusion points sender */
  YAML::Node pointcloud_sender_config = yamlSubNode(proto_sender_config, "pointcloud");
  bool send_compensated_cloud_proto;
  bool send_uncompensated_cloud_proto;
  yamlRead<bool>(pointcloud_sender_config, "send_compensated_cloud_proto", send_compensated_cloud_proto, false);
  yamlRead<bool>(pointcloud_sender_config, "send_uncompensated_cloud_proto", send_uncompensated_cloud_proto, false);
  if (send_compensated_cloud_proto)
  {
    compensate_cloud_send_thread_flag_ = true;
    compensate_cloud_send_thread_.reset(new std::thread(std::bind(&SdkDemo::send_compensate_cloud, this)));
    std::string compensated_cloud_proto_port;
    std::string cloud_proto_ip;
    yamlReadAbort<std::string>(pointcloud_sender_config, "compensated_cloud_proto_port", compensated_cloud_proto_port);
    yamlReadAbort<std::string>(pointcloud_sender_config, "cloud_proto_ip", cloud_proto_ip);
    common::ProtoBase *proto_base = new common::ProtoBase();
    proto_map_.emplace("compensated_cloud_proto", proto_base);
    proto_map_["compensated_cloud_proto"]->initSender(compensated_cloud_proto_port, cloud_proto_ip);
    auto func = [this](const common::LidarPointsMsg &msg) {
      compensate_proto_msg_ = toProtoMsg(msg);
      compensate_cloud_send_cv_.notify_all();
    };
    preProcess_ptr_->regRecvCallback(func, true, fusion_frame_id_);
  }
  if (send_uncompensated_cloud_proto)
  {
    uncompensate_cloud_send_thread_flag_ = true;
    uncompensate_cloud_send_thread_.reset(new std::thread(std::bind(&SdkDemo::send_uncompensate_cloud, this)));
    std::string uncompensated_cloud_proto_port;
    std::string cloud_proto_ip;
    yamlReadAbort<std::string>(pointcloud_sender_config, "uncompensated_cloud_proto_port", uncompensated_cloud_proto_port);
    yamlReadAbort<std::string>(pointcloud_sender_config, "cloud_proto_ip", cloud_proto_ip);
    common::ProtoBase *proto_base = new common::ProtoBase();
    proto_map_.emplace("uncompensated_cloud_proto", proto_base);
    proto_map_["uncompensated_cloud_proto"]->initSender(uncompensated_cloud_proto_port, cloud_proto_ip);
    auto func = [this](const common::LidarPointsMsg &msg) {
      uncompensate_proto_msg_ = toProtoMsg(msg);
      uncompensate_cloud_send_cv_.notify_all();
    };
    preProcess_ptr_->regRecvCallback(func, false, fusion_frame_id_);
  }

/* Localization result proto sender*/
#ifdef COMPILE_LOCALIZATION
  YAML::Node localization_sender_config = yamlSubNodeAbort(proto_sender_config, "localization");
  yamlRead<bool>(localization_sender_config, "send_pos_proto", send_pos_proto_, false);
  if (send_pos_proto_)
  {
    std::string send_pos_proto_port;
    std::string send_pos_proto_ip;
    yamlReadAbort<std::string>(localization_sender_config, "send_pos_proto_port", send_pos_proto_port);
    yamlReadAbort<std::string>(localization_sender_config, "send_pos_proto_ip", send_pos_proto_ip);
    common::ProtoBase *proto_base = new common::ProtoBase();
    proto_map_.emplace("send_pos_proto", proto_base);
    proto_map_["send_pos_proto"]->initSender(send_pos_proto_port, send_pos_proto_ip);
  }
#endif

/* Perception Result proto sender*/
#ifdef COMPILE_PERCEPTION
  if (run_perception_)
  {
    YAML::Node perception_sender_config = yamlSubNodeAbort(proto_sender_config, "perception");
    bool send_obstacle_proto;
    int perception_device_num;
    yamlRead<bool>(perception_sender_config, "send_obstacle_proto", send_obstacle_proto, false);
    yamlRead<int>(perception_sender_config, "perception_device_num", perception_device_num, 0);
    if (send_obstacle_proto)
    {
      std::string send_obstacle_proto_port;
      std::string send_obstacle_proto_ip;
      yamlReadAbort<std::string>(perception_sender_config, "send_obstacle_proto_port", send_obstacle_proto_port);
      yamlReadAbort<std::string>(perception_sender_config, "send_obstacle_proto_ip", send_obstacle_proto_ip);
      common::ProtoBase *proto_base = new common::ProtoBase();
      proto_map_.emplace("send_obstacle_proto", proto_base);
      proto_map_["send_obstacle_proto"]->initSender(send_obstacle_proto_port, send_obstacle_proto_ip);
      auto func = [this, perception_device_num](const common::ObstacleMsg::Ptr &msg_ptr) {
        int obj_id = 0;
        if (!msg_ptr->obstacles.empty())
        {
          for (auto iter : msg_ptr->obstacles)
          {
            Proto_msg::Obstacle proto_msg = toProtoMsg(iter);
            common::proto_MsgHeader proto_header;
            proto_header.msgType = 0; ///< DO NOT MODIFY!
            proto_header.frmNumber = msg_ptr->seq;
            proto_header.totalMsgCnt = msg_ptr->obstacles.size();
            proto_header.msgID = obj_id;
            proto_header.msgLen = proto_msg.ByteSize();
            proto_header.deviceNum = perception_device_num;
            proto_header.deviceTimeStampMs = (std::time_t)((msg_ptr->timestamp) * 1000);
            proto_header.flags = 1;
            obj_id++;
            void *buff = malloc(proto_header.msgLen);
            proto_msg.SerializeToArray(buff, proto_header.msgLen);
            if (proto_map_["send_obstacle_proto"]->sendProtoMsg(buff, proto_header) == -1)
            {
              errorprinter_ptr_->printErrorCode(common::ErrCode_PerceptionProtobufSendError);
              // ERROR << "SdkDemo: Send Message Data Failed, Error: " << errno << REND;
            }
            free(buff);
          }
        }
        else
        {
          common::proto_MsgHeader proto_header;
          proto_header.msgType = 99; ///< DO NOT MODIFY!
          proto_header.frmNumber = msg_ptr->seq;
          proto_header.totalMsgCnt = msg_ptr->obstacles.size();
          proto_header.msgID = obj_id;
          proto_header.msgLen = 0;
          proto_header.deviceNum = perception_device_num;
          proto_header.deviceTimeStampMs = (std::time_t)((msg_ptr->timestamp) * 1000);
          proto_header.flags = 1;
          void *buff = malloc(proto_header.msgLen);
          if (proto_map_["send_obstacle_proto"]->sendProtoMsg(buff, proto_header) == -1)
          {
            errorprinter_ptr_->printErrorCode(common::ErrCode_PerceptionProtobufSendError);
            // ERROR << "SdkDemo: Send Message Data Failed, Error: " << errno << REND;
          }
          free(buff);
        }
      };
      auto func1 = [this, perception_device_num](const common::FreeSpaceMsg::Ptr &msg_ptr) {
        if (!msg_ptr->freespaces.empty())
        {
          Proto_msg::FreeSpaces proto_msg = toProtoMsg(msg_ptr);
          common::proto_MsgHeader proto_header;
          proto_header.msgType = 1; ///< DO NOT MODIFY!
          proto_header.frmNumber = msg_ptr->seq;
          proto_header.totalMsgCnt = 1; ///< DO NOT MODIFY!
          proto_header.msgID = 0;       ///< DO NOT MODIFY!
          proto_header.msgLen = proto_msg.ByteSize();
          proto_header.deviceNum = perception_device_num;
          proto_header.deviceTimeStampMs = (std::time_t)((msg_ptr->timestamp) * 1000);
          proto_header.flags = 1;
          void *buff = malloc(proto_header.msgLen);
          proto_msg.SerializeToArray(buff, proto_header.msgLen);
          if (proto_map_["send_obstacle_proto"]->sendProtoMsg(buff, proto_header) == -1)
          {
            errorprinter_ptr_->printErrorCode(common::ErrCode_PerceptionProtobufSendError);
            // ERROR << "SdkDemo: Send Message Data Failed, Error: " << errno << REND;
          }
          free(buff);
        }
        else
        {
          common::proto_MsgHeader proto_header;
          proto_header.msgType = 99; ///< DO NOT MODIFY!
          proto_header.frmNumber = msg_ptr->seq;
          proto_header.totalMsgCnt = 0;
          proto_header.msgID = 0;
          proto_header.msgLen = 0;
          proto_header.deviceNum = perception_device_num;
          proto_header.deviceTimeStampMs = (std::time_t)((msg_ptr->timestamp) * 1000);
          proto_header.flags = 1;
          void *buff = malloc(proto_header.msgLen);
          if (proto_map_["send_obstacle_proto"]->sendProtoMsg(buff, proto_header) == -1)
          {
            errorprinter_ptr_->printErrorCode(common::ErrCode_PerceptionProtobufSendError);
            // ERROR << "SdkDemo: Send Message Data Failed, Error: " << errno << REND;
          }
          free(buff);
        }
      };
      perception_ptr_->regObstacleCallback(func);
      perception_ptr_->regFreeSpaceCallback(func1);
    }
  }
#endif
}
#endif

void SdkDemo::send_compensate_cloud()
{
#ifdef PROTO_FOUND
  std::unique_lock<std::mutex> lk(compensate_cloud_send_mutex_);
  std::mutex tmp_lk;
  while (compensate_cloud_send_thread_flag_)
  {
    compensate_cloud_send_cv_.wait(lk);
    tmp_lk.lock();
    void *buf = malloc(compensate_proto_msg_.ByteSize() + SPLIT_SIZE);
    compensate_proto_msg_.SerializeToArray(buf, compensate_proto_msg_.ByteSize());
    int pkt_num = ceil(1.0 * compensate_proto_msg_.ByteSize() / SPLIT_SIZE);
    common::proto_MsgHeader tmp_header;
    tmp_header.frmNumber = compensate_proto_msg_.seq();
    tmp_header.msgLen = SPLIT_SIZE;
    tmp_header.totalMsgCnt = pkt_num;
    tmp_header.totalMsgLen = compensate_proto_msg_.ByteSize();
    for (int i = 0; i < pkt_num; i++)
    {
      tmp_header.msgID = i;
      void *tmp_buf = malloc(SPLIT_SIZE);
      memcpy(tmp_buf, (uint8_t *)buf + i * SPLIT_SIZE, SPLIT_SIZE);
      if (proto_map_["compensated_cloud_proto"]->sendProtoMsg(tmp_buf, tmp_header) == -1)
      {
        ERROR << "SdkDemo: Send Compensate cloud Failed, Error: " << errno << REND;
      }
      free(tmp_buf);
      usleep(30);
    }
    free(buf);
    tmp_lk.unlock();
  }
#endif
}
void SdkDemo::send_uncompensate_cloud()
{
#ifdef PROTO_FOUND
  std::unique_lock<std::mutex> lk(uncompensate_cloud_send_mutex_);
  std::mutex tmp_lk;
  while (uncompensate_cloud_send_thread_flag_)
  {
    uncompensate_cloud_send_cv_.wait(lk);
    tmp_lk.lock();
    void *buf = malloc(uncompensate_proto_msg_.ByteSize() + SPLIT_SIZE);
    uncompensate_proto_msg_.SerializeToArray(buf, uncompensate_proto_msg_.ByteSize());
    int pkt_num = ceil(1.0 * uncompensate_proto_msg_.ByteSize() / SPLIT_SIZE);
    common::proto_MsgHeader tmp_header;
    tmp_header.frmNumber = uncompensate_proto_msg_.seq();
    tmp_header.msgLen = SPLIT_SIZE;
    tmp_header.totalMsgCnt = pkt_num;
    tmp_header.totalMsgLen = uncompensate_proto_msg_.ByteSize();
    for (int i = 0; i < pkt_num; i++)
    {
      tmp_header.msgID = i;
      void *tmp_buf = malloc(SPLIT_SIZE);
      memcpy(tmp_buf, (uint8_t *)buf + i * SPLIT_SIZE, SPLIT_SIZE);
      if (proto_map_["uncompensated_cloud_proto"]->sendProtoMsg(tmp_buf, tmp_header) == -1)
      {
        ERROR << "SdkDemo: Send Uncompensate cloud Failed, Error: " << errno << REND;
      }
      free(tmp_buf);
      usleep(30);
    }
    free(buf);
    tmp_lk.unlock();
  }
#endif
}

/**
 * @brief  Localization thread function
 * @detail Since the frequency to get localization function is not fix, so there is a separate thread to get the result of localization function
 *         If related parameters are set, the localization result can be sent through ROS or protobuf
 */
#ifdef COMPILE_LOCALIZATION
void SdkDemo::localizationThreadFunc(void)
{
#ifdef ROS_FOUND
  tf_broadcaster_ptr_.reset(new tf::TransformBroadcaster);
  if (use_ros_ && send_map_ros_)
  {
    common::GridMap grid_map;
    common::LidarPointsMsg pointcloud_map;
    nav_msgs::OccupancyGrid grid_ros;
    if (localization_ptr_->getMap(grid_map) == common::ErrCode_Success)
    {
      toRosMsg(grid_map, grid_ros);
      localization_grid_map_pub_.publish(grid_ros);
    }
    if (localization_ptr_->getMap(pointcloud_map) == common::ErrCode_Success)
    {
      localization_pointcloud_map_pub_.publish(toRosMsg(pointcloud_map));
    }
  }
#endif
  while (run_localization_)
  {
    const std::chrono::nanoseconds total_duration(static_cast<int>(1.0 / localization_freq_ * 1000000000.0));
    auto status = localization_ptr_->getModuleStatus().localization_status;
    auto start_time = std::chrono::system_clock::now();
    if (status == localization::loc_status::LOC_NORMAL)
    {
      common::VehicleStateMsg cur_state;
      if (localization_ptr_->getVehicleState(cur_state) == common::ErrCode_Success)
      {
        static double prev_time = -1.0;
        if (prev_time != cur_state.timestamp)
        {
          preProcess_ptr_->localizationResultCallback(cur_state);
          pubLocalizationRos(cur_state);
          pubLocalizationProto(cur_state);
        }
        prev_time = cur_state.timestamp;
      }
    }
    auto end_time = std::chrono::system_clock::now();
    std::chrono::nanoseconds exec_duration = end_time - start_time;
    if (total_duration > exec_duration)
    {
      std::this_thread::sleep_for(total_duration - exec_duration);
    }
  }
}

/**
 * @brief  Localization result sending function (ROS)
 * @detail Sned the path, position and TF to ROS
 */
void SdkDemo::pubLocalizationRos(const common::VehicleStateMsg &vstate)
{
#ifdef ROS_FOUND
  /**** Pub Path ****/
  if (use_ros_ && send_path_ros_)
  {
    toRosMsg(vstate, car_path_);
    localization_path_pub_.publish(car_path_);
  }
  /**** Pub Pos ****/
  if (use_ros_ && send_pos_ros_)
  {
    localization_pos_pub_.publish(toRosMsg(vstate));
  }
  /**** Pub TF ****/
  tf_broadcaster_ptr_->sendTransform(toRosTf(vstate));
#endif
}

/**
 * @brief  Localization result sending function (Proto)
 * @detail Sned the position through Proto
 */
void SdkDemo::pubLocalizationProto(const common::VehicleStateMsg &vstate)
{
#ifdef PROTO_FOUND
  if (use_proto_ && send_pos_proto_)
  {
    Proto_msg::VehicleState proto_msg = toProtoMsg(vstate);
    common::proto_MsgHeader proto_header;
    proto_header.msgLen = proto_msg.ByteSize();
    proto_header.frmNumber = vstate.seq;
    void *buff = malloc(proto_header.msgLen);
    proto_msg.SerializeToArray(buff, proto_header.msgLen);
    if (proto_map_["send_pos_proto"]->sendProtoMsg(buff, proto_header) == -1)
    {
      errorprinter_ptr_->printErrorCode(common::ErrCode_LocalizationProtobufSendError);
      //ERROR << "SdkDemo: Send Message Data Failed, Error: " << errno << REND;
    }
    free(buff);
  }
#endif
}
#endif
} // namespace robosense