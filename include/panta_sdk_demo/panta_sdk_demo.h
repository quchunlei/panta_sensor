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
#include <panta_common/common.h>
#include <panta_common/debug/error_printer.h>
#include <panta_sensor/sensor_manager/sensor_manager.h>
#include <panta_preprocessing/panta_preprocessing.h>
#include <panta_common/msg/rs_msg/vehiclestate_msg.h>
#ifdef PROTO_FOUND
#include <panta_common/proto/proto_base.hpp>
#include <panta_common/msg/proto_msg_translator.h>
#endif
#ifdef ROS_FOUND
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>
#include <panta_common/msg/ros_msg/freespace_ros_msg.h>
#include <panta_common/msg/ros_msg/freespace_set_ros_msg.h>
#include <panta_common/msg/ros_msg/obstacle_ros_msg.h>
#include <panta_common/msg/ros_msg/obstacle_set_ros_msg.h>
#endif
#define SPLIT_SIZE 5000
namespace robosense
{
/**
 * @brief Demo Class
 * @note Use to control the whole working flow
 */
class SdkDemo : virtual public common::CommonBase
{
  /* Constructor & Deconstructor */
public:
  SdkDemo() = default;
  ~SdkDemo();

  /* Public interface */
public:
  void init(const YAML::Node& config, bool use_ros, bool use_proto, std::string config_path);
  void start(void);
  bool stop(void);

  /* Initialization functions */
private:
  void initSensorManager(const YAML::Node& sensor_config);
  void initPreProcessing(const YAML::Node& preprocessing_config);
  void initLocalization(const YAML::Node& localization_config);
  void initPerception(const YAML::Node& perception_config);
  void initOdometry(const YAML::Node& odometry_config);
  void initRosPublisher(const YAML::Node& sdk_demo_config);
  void initProtoSender(const YAML::Node& sdk_demo_config);
  void pubLocalizationRos(const common::VehicleStateMsg& vstate);
  void pubLocalizationProto(const common::VehicleStateMsg& vstate);
  void send_compensate_cloud();
  void send_uncompensate_cloud();
  /* Callback function to handle the error code */
private:
  inline void localExceptionCallback(const common::ErrCode& code)
  {
    if ((code & 0xF00) >= 0x800)
      errorprinter_ptr_->printErrorCode(code);  ///< TODO: not only print the error but also handle it
    if ((code & 0xF00) != 0 && (code & 0xF00) < 0x800)
      error_set_.insert(code);
  }
  /* Config common varibles */
private:
  bool use_ros_;
  bool use_proto_;
  bool run_flag_;
  std::string config_path_;
  std::string fusion_frame_id_;
  std::string vehicle_frame_id_;
  std::string map_frame_id_;
  std::array<double, 6> lidar_base_pose_;
  common::LidarPointsMsg local_points_;
  std::unique_ptr<sensor::SensorManager> sensorManager_ptr_;
  std::unique_ptr<preprocessing::PreProcessing> preProcess_ptr_;
  std::unique_ptr<common::ErrorPrinter> errorprinter_ptr_;
  std::set<common::ErrCode> error_set_;
  std::unique_ptr<std::thread> error_handle_thread_;
  std::unique_ptr<std::thread> compensate_cloud_send_thread_;
  std::unique_ptr<std::thread> uncompensate_cloud_send_thread_;
  bool compensate_cloud_send_thread_flag_;
  bool uncompensate_cloud_send_thread_flag_;
  std::mutex compensate_cloud_send_mutex_;
  std::condition_variable compensate_cloud_send_cv_;
  std::mutex uncompensate_cloud_send_mutex_;
  std::condition_variable uncompensate_cloud_send_cv_;
  Proto_msg::LidarPoints compensate_proto_msg_;
  Proto_msg::LidarPoints uncompensate_proto_msg_;

  /* Config for Localization */
#ifdef COMPILE_LOCALIZATION
private:
  std::unique_ptr<localization::RSLocalization> localization_ptr_;
  std::atomic<bool> run_localization_;
  std::array<double, 6> pose_guess_;
  std::thread localization_thread_;
  int localization_mode_;
  void localizationThreadFunc(void);
  int localization_freq_;
  bool send_pos_ros_;
  bool send_map_ros_;
  bool send_path_ros_;
  bool send_pos_proto_;
#endif

  /* Config for Perception */
#ifdef COMPILE_PERCEPTION
private:
  std::unique_ptr<perception::RSPerception<pcl::PointXYZI>> perception_ptr_;
  enum mode
  {
    ONLYLIDAR = 0,
    WITHLOCALIZATION = 1,
    WITHODOMETRY = 2,
    V2R = 3,
  };
  bool run_perception_;
  int perception_mode_;

  /* Config for Odometry */
private:
  std::unique_ptr<odometry::RSOdometry<pcl::PointXYZI>> odometry_ptr_;
  std::atomic<bool> run_odometry_;
#endif

  /* Config ROS varibles */
#ifdef ROS_FOUND
  nav_msgs::Path car_path_;
  std::unique_ptr<tf::TransformBroadcaster> tf_broadcaster_ptr_;
  std::unique_ptr<ros::NodeHandle> nh_;
  ros::Publisher compensated_points_pub_;
  ros::Publisher uncompensated_points_pub_;
  ros::Publisher processed_imu_pub_;
  ros::Publisher localization_pos_pub_;
  ros::Publisher localization_grid_map_pub_;
  ros::Publisher localization_pointcloud_map_pub_;
  ros::Publisher localization_path_pub_;
  ros::Publisher perception_obstacle_pub_;
  ros::Publisher perception_freespace_pub_;
  ros::Publisher perception_rviz_pub_;
  ros::Publisher perception_rviz_pub_freespace_;
  ros::Publisher perception_pointcloud_pub_;
  ros::Publisher ground_points_pub_;
#ifdef COMPILE_PERCEPTION
  perception::DrawRviz::Ptr rviz_drawer_;

#endif
#endif

  /* Config Proto varibles */
#ifdef PROTO_FOUND
  std::map<std::string, common::ProtoBase*> proto_map_;
#endif
};
}  // namespace robosense
