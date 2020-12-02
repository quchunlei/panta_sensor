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
#include <iostream>
#include <thread>
#include <functional>
#include <future>
#include <condition_variable>
#include <time.h>
#include <Eigen/Dense>
#include <iomanip>
#include <set>
#include <omp.h>
#include <panta_common/debug/prompt.h>
#include <panta_common/debug/error_code.h>
#include <panta_common/msg/rs_msg/gnss_msg.h>
#include <panta_common/msg/rs_msg/imu_msg.h>
#include <panta_common/msg/rs_msg/lidar_points_msg.h>
#include <panta_common/msg/rs_msg/odom_msg.h>
#include <panta_common/msg/rs_msg/vehiclestate_msg.h>
#include <panta_common/interface/preprocessing/preprocessing_interface.h>

#include "panta_preprocessing/config_manager.h"
#include "panta_preprocessing/panta_time.h"
#include "panta_preprocessing/transform.h"
#include "panta_preprocessing/message_filter.h"

using robosense::common::ErrCode;
using robosense::common::GnssMsg;
using robosense::common::ImuMsg;
using robosense::common::LidarPointsMsg;
using robosense::common::OdomMsg;
using robosense::common::VehicleStateMsg;



typedef Eigen::Matrix<float, 6, 1> Vector6f;

namespace robosense
{
namespace preprocessing
{

// 没一类传感器使用一个线程
struct SensorThread
{
  std::thread m_thread;
  std::atomic<bool> start;
};

// 每一个消息使用一套互斥保护
struct SensorDataMutex
{
  std::mutex m_mutex;
  std::condition_variable m_cv;
  bool is_ready = false;  // true 新数据
};

struct SensorThreadState
{
  std::thread::id imu_thread_id;
  std::thread::id odom_thread_id;
  std::thread::id gnss_thread_id;
  std::thread::id lidar_thread_id;
  std::thread::id monitor_thread_id;
  bool imu_joinable = false;
  bool gnss_joinable = false;
  bool odom_joinable = false;
  bool lidar_joinable = false;
  bool monitor_joinable = false;
  bool imu_start = false;
  bool gnss_start = false;
  bool odom_start = false;
  bool lidar_start = false;
  bool monitor_start = false;
};

struct LidarCallback
{
  std::function<void(const LidarPointsMsg&)> callback;
  bool motion_correct;
  std::string frame_id;
};

class PreProcessing : public robosense::common::PreprocessInterface
{
  typedef pcl::PointXYZI PointT;

public:
  PreProcessing();

  ~PreProcessing();

  // version and dependency version
  static void info();

  ErrCode init(const std::string& _config_file);

  ErrCode init(const YAML::Node& _node);  // override;


  // 启动工作线程
  ErrCode start();  // override;

  // 停止工作线程
  ErrCode stop();  // override;

  ErrCode reset();  // override;

  /*
   * 在init()之后调用，
   * return x，y，z，roll，pitch， yaw
   */
  std::array<double, 6> getSensorTransform(const std::string sensor_type = "lid"
                                                                           "a"
                                                                           "r");
  std::string getBaselinkFrameid(void);
  // input
  void imuCallback(const ImuMsg& msg);  // override;

  void gnssCallback(const GnssMsg& msg);  // override;

  void odomCallback(const OdomMsg& msg);  // override;

void localizationResultCallback(const VehicleStateMsg& msg);


  void lidarPointsCallback(const LidarPointsMsg& msg);  // override;

  // output
  void regRecvCallback(
      const std::function<void(const ImuMsg&)> callback);  // override;

  void regRecvCallback(
      const std::function<void(const GnssMsg&)> callback);  // override;

  void regRecvCallback(
      const std::function<void(const OdomMsg&)> callback);  // override;

  void
  regRecvCallback(const std::function<void(const LidarPointsMsg&)> callback,
                  const bool& motion_correct,
                  const std::string& frame_id);  // override;

  // Error
  void regExceptionCallback(
      const std::function<void(const ErrCode&)> excallback);  // override;

  SensorThreadState getThreadState();

private:
  //-------------------configure---------------------------------------
  /**加载传感器标定参数
   */
  bool loadCalibConfig(const std::string& file);

  ErrCode init();
  SensorGroup sensor_group_;
  SensorStateGroup sensor_state_group_;
  PreProcConfig config_;

  //-----------------------thread--------------------------------
  void imuRun();
  void odomRun();
  void gnssRun();

  void lidarFusionRun();
  template <std::size_t index>  // start from 0
  void lidarRun();

  void monitorRun();

  SensorThread imu_thread_;
  SensorThread gnss_thread_;
  SensorThread odom_thread_;
  SensorThread lidar_thread_;
  SensorThread monitor_thread_;

  struct SensorMutex
  {
    SensorDataMutex imu;
    SensorDataMutex odom;
    SensorDataMutex gnss;
    std::array<SensorDataMutex, LIDAR_NO_MAX> lidars;
    SensorDataMutex lidar_fusion;
  } sensor_mutex_;
  std::set<ErrCode> set_error_;
  std::map<double, std::string> map_time_stamp_;

  //-------------------------register callback--------------------------------
  std::vector<std::function<void(const ImuMsg&)>> vec_callback_imu_;
  std::vector<std::function<void(const OdomMsg&)>> vec_callback_odom_;
  std::vector<std::function<void(const GnssMsg&)>> vec_callback_gnss_;
  std::vector<LidarCallback> vec_callback_lidar_all_;
  // back() is for fusion
  std::array<std::vector<std::function<void(const LidarPointsMsg&)>>,
             LIDAR_NO_MAX + 1>
      arr_vec_callback_lidar_no_correct_;

  // motion correction msg, back() is for fusion
  std::array<std::vector<std::function<void(const LidarPointsMsg&)>>,
             LIDAR_NO_MAX + 1>
      arr_vec_callback_lidar_correct_;

  std::vector<std::function<void(const ErrCode&)>> vec_callback_error_;

  ErrCode lidarMsgDispatch();
  // msg data buffer
  ImuMsg imu_msg_in_;
  GnssMsg gnss_msg_in_;
  OdomMsg odom_msg_in_;  // 车速计
  std::array<LidarPointsMsg, LIDAR_NO_MAX> arr_lidar_msg_in_;

  // LidarPointsMsg lidar_msg_fusion_;
  std::array<LidarPointsMsg, LIDAR_NO_MAX + 1> arr_lidar_msg_correct_;
  std::array<LidarPointsMsg, LIDAR_NO_MAX + 1> arr_lidar_msg_no_correct_;


  //------------------message synchronization filter -------------------------
  // template <typename T>
  void msgSyncFilter_1(const LidarPointsMsg& msg0);

  // template <typename T>
  void msgSyncFilter_2(const LidarPointsMsg& msg0, const LidarPointsMsg& msg1);
  // template <typename T>
  void msgSyncFilter_3(const LidarPointsMsg& msg0, const LidarPointsMsg& msg1,
                       const LidarPointsMsg& msg2);

  void msgSyncFilter_4(const LidarPointsMsg& msg0, const LidarPointsMsg& msg1,
                       const LidarPointsMsg& msg2, const LidarPointsMsg& msg3);

  void msgSyncFilter_5(const LidarPointsMsg& msg0, const LidarPointsMsg& msg1,
                       const LidarPointsMsg& msg2, const LidarPointsMsg& msg3,
                       const LidarPointsMsg& msg4);

  void msgSyncFilter_6(const LidarPointsMsg& msg0, const LidarPointsMsg& msg1,
                       const LidarPointsMsg& msg2, const LidarPointsMsg& msg3,
                       const LidarPointsMsg& msg4, const LidarPointsMsg& msg5);

  std::shared_ptr<MessageFilter<LidarPointsMsg>> message_filter_;


  // ------------------------在lidar坐标系下进行矫正--------------------------------
  void correctPointcloud(const pcl::PointCloud<PointT>::Ptr& raw_cloud,
                         pcl::PointCloud<PointT>::Ptr& corrected_cloud,
                         const float& car_vel, const float& angular_vel,
                         const float& diff_time,
                         const Vector6f& lidar_pose_vec);
  void correctPointcloud3D(const pcl::PointCloud<PointT>::Ptr& raw_cloud,
                           pcl::PointCloud<PointT>::Ptr& corrected_cloud,
                           const double& car_vel_x, const double& car_vel_y,
                           const double& car_vel_z, const double& angular_vel_x,
                           const double& angular_vel_y,
                           const double& angular_vel_z, const double& cur_yaw,
                           const float& diff_time,
                           const Vector6f& lidar_pose_vec);
  double getMotionDirection(const double& cur_to_pre_x,
                            const double& cur_to_pre_y, const double& cur_yaw);
  Eigen::Vector2f rigidMotionTransform(const float& car_vel,
                                       const float& angular_vel,
                                       const Vector6f& lidar_pose);
  void motionCorrectAndTransform(const LidarPointsMsg, const std::size_t index);

  std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>
      vec_lidar_to_baselink_tf_;

  double car_vel_;
  double imu_angular_vel_;
  std::array<double, 3> imu_angular_vel_3d_;
  std::array<double, 3> linear_velocity_3d_;
  std::atomic<bool> localization_result_flag_;
  double cur_yaw_;
};

}  // namespace preprocessing
}  // namespace robosense
